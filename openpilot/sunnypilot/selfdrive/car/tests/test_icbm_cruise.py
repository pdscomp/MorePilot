"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Tests for ICBM (non-pcmCruiseSpeed) cruise handling:
- setpoint reconciliation: around driver presses the dash is the source of truth, but only
  when the plan source is cruise and ICBM is not mid-move (single-writer setpoint)
- SLA button ownership: while SLA is active, +/- presses never increment v_cruise
- ICBM state machine reaction deadband and persistence timer
- the vEgo clip on SET- while overriding is disabled for ICBM cars
"""
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import VCruiseHelper, IMPERIAL_INCREMENT
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
  IntelligentCruiseButtonManagement, REACT_DEADBAND, RESTORE_QUIET_TIME)

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

MPH = CV.MPH_TO_KPH  # dash and v_cruise are tracked in kph; the CX-5 dash steps in whole mph


def make_car_state(dash_kph=0., gas_pressed=False, button_events=None, available=True, v_ego=0.):
  CS = car.CarState(cruiseState={"available": available, "speed": dash_kph * CV.KPH_TO_MS})
  CS.gasPressed = gas_pressed
  CS.vEgo = v_ego
  CS.buttonEvents = button_events or []
  return CS


class TestSetpointReconcile:
  """pcmCruise (stock ACC) car with ICBM enabled (pcmCruiseSpeed=False)."""

  def setup_method(self):
    Params().put_bool("CustomAccIncrementsEnabled", False)
    self.CP = car.CarParams(pcmCruise=True)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)

  def set_regime(self, source='cruise', icbm_state='inactive', sla_state='disabled', limit_kph=0.):
    LP_SP = custom.LongitudinalPlanSP()
    LP_SP.longitudinalPlanSource = source
    LP_SP.speedLimit.assist.state = sla_state
    LP_SP.speedLimit.resolver.speedLimit = limit_kph * CV.KPH_TO_MS
    LP_SP.speedLimit.resolver.speedLimitFinalLast = limit_kph * CV.KPH_TO_MS
    LP_SP.speedLimit.resolver.speedLimitLastValid = limit_kph > 0
    CC_SP = custom.CarControlSP()
    CC_SP.intelligentCruiseButtonManagement.state = icbm_state
    self.v_cruise_helper.update_speed_limit_assist(False, LP_SP, CC_SP)
    # the arbiter owns the session now: an "SLA active" regime is arbiter state, not
    # a longitudinalPlanSP echo
    arb = self.v_cruise_helper.cruise_arbiter
    arb.enabled = True
    arb.state = getattr(custom.LongitudinalPlanSP.SpeedLimit.AssistState, sla_state)
    arb._speed_limit_prev = arb._speed_limit  # regime setup is not a limit-change edge

  def run_frames(self, CS, n=1, enabled=True):
    for _ in range(n):
      self.v_cruise_helper.update_v_cruise(CS, enabled=enabled, is_metric=False)
      CS.buttonEvents = []

  def engage_at(self, dash_kph):
    # settle the enabled state machine with the dash at a fixed value
    self.run_frames(make_car_state(dash_kph=dash_kph), n=5, enabled=False)
    self.run_frames(make_car_state(dash_kph=dash_kph), n=5, enabled=True)
    assert abs(self.v_cruise_helper.v_cruise_kph - dash_kph) < 0.1

  def press(self, button_type, dash_kph):
    CS = make_car_state(dash_kph=dash_kph, button_events=[ButtonEvent(type=button_type, pressed=True)])
    self.run_frames(CS, n=2)
    CS = make_car_state(dash_kph=dash_kph, button_events=[ButtonEvent(type=button_type, pressed=False)])
    self.run_frames(CS, n=1)

  def test_adopts_trailing_ecu_increment(self):
    """The ECU applies its final long-press step right after release; v_cruise must adopt it."""
    self.engage_at(35 * MPH)

    self.press(ButtonType.accelCruise, dash_kph=35 * MPH)
    # ECU's trailing +5 mph step lands shortly after release
    self.run_frames(make_car_state(dash_kph=40 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 40 * MPH) < 0.5

  def test_sync_window_expires(self):
    """1 s after the last press the dash is no longer authoritative."""
    self.engage_at(35 * MPH)

    self.press(ButtonType.accelCruise, dash_kph=35 * MPH)
    self.run_frames(make_car_state(dash_kph=40 * MPH), n=20)
    # window closes 1 s after release; later dash moves (e.g. ICBM pushing it for SCC) don't leak in
    self.run_frames(make_car_state(dash_kph=40 * MPH), n=100)
    self.run_frames(make_car_state(dash_kph=30 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 40 * MPH) < 0.5

  def test_no_adoption_while_scc_limited(self):
    """When a limiter drives the plan, the dash is held away from v_cruise by design;
    a press must increment v_cruise, never adopt the limiter-held dash."""
    self.engage_at(45 * MPH)
    self.set_regime(source='sccVision')
    # smart cruise pushed the real dash down to 35 mph while v_cruise stays at 45 mph
    self.run_frames(make_car_state(dash_kph=35 * MPH), n=110)
    assert abs(self.v_cruise_helper.v_cruise_kph - 45 * MPH) < 0.1

    self.press(ButtonType.accelCruise, dash_kph=35 * MPH)
    self.run_frames(make_car_state(dash_kph=36 * MPH), n=20)
    # v_cruise took its own +1 mph increment, not the dash value
    assert abs(self.v_cruise_helper.v_cruise_kph - (45 * MPH + IMPERIAL_INCREMENT)) < 0.5

  def test_no_adoption_while_icbm_mid_move(self):
    """After an SLA abort the servo restores the dash; the press's settle window must not
    adopt the still-low dash while ICBM is stepping it back up."""
    self.engage_at(60 * MPH)
    self.set_regime(source='cruise', icbm_state='increasing')

    self.press(ButtonType.accelCruise, dash_kph=45 * MPH)
    self.run_frames(make_car_state(dash_kph=47 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - (60 * MPH + IMPERIAL_INCREMENT)) < 0.5

  def test_sla_owns_buttons_no_increment(self):
    """While SLA is active, +/- presses carry SLA semantics; v_cruise must not increment."""
    self.engage_at(60 * MPH)
    self.set_regime(source='speedLimitAssist', sla_state='active', limit_kph=45 * MPH)

    self.press(ButtonType.accelCruise, dash_kph=45 * MPH)
    assert abs(self.v_cruise_helper.v_cruise_kph - 60 * MPH) < 0.1

  def test_sla_settled_press_reanchors_to_dash(self):
    """Settled at a limit, a + press deactivates SLA (plannerd side) and the setpoint
    re-anchors to the ECU's dash response: stock button feel."""
    self.engage_at(60 * MPH)
    self.set_regime(source='speedLimitAssist', sla_state='active', limit_kph=45 * MPH)
    self.run_frames(make_car_state(dash_kph=45 * MPH), n=110)
    assert abs(self.v_cruise_helper.v_cruise_kph - 60 * MPH) < 0.1

    # press lands while SLA is still active: no increment (owned by SLA)
    self.press(ButtonType.accelCruise, dash_kph=45 * MPH)
    # SLA deactivates on the press, plan source returns to cruise, servo is idle;
    # the ECU stepped the dash to 46; adopt it inside the settle window
    self.set_regime(source='cruise', icbm_state='holding', sla_state='inactive')
    self.run_frames(make_car_state(dash_kph=46 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 46 * MPH) < 0.5

  def test_sla_abort_press_keeps_baseline(self):
    """Mid-decrease for SLA, a + press aborts: the baseline must survive untouched while
    the servo walks the dash back up."""
    self.engage_at(60 * MPH)
    self.set_regime(source='speedLimitAssist', sla_state='active', limit_kph=45 * MPH)

    # servo is halfway down (dash 52) when the driver presses +
    self.press(ButtonType.accelCruise, dash_kph=52 * MPH)
    # SLA deactivates; servo restores (increasing); dash still low during the settle window
    self.set_regime(source='cruise', icbm_state='increasing', sla_state='inactive')
    self.run_frames(make_car_state(dash_kph=53 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 60 * MPH) < 0.1

  def test_vego_clip_disabled_for_icbm(self):
    """SET- while on the gas decrements on the stock ECU; v_cruise must not jump up to vEgo."""
    self.engage_at(35 * MPH)

    CS = make_car_state(dash_kph=35 * MPH, gas_pressed=True, v_ego=30.,
                        button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=True)])
    self.run_frames(CS, n=2)
    CS = make_car_state(dash_kph=34 * MPH, gas_pressed=True, v_ego=30.,
                        button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=False)])
    self.run_frames(CS, n=1)
    self.run_frames(make_car_state(dash_kph=34 * MPH, gas_pressed=True, v_ego=30.), n=10)

    # 30 m/s = 108 kph; without the guard v_cruise would have clipped up to vEgo
    assert self.v_cruise_helper.v_cruise_kph < 60.


class TestServo:
  """ButtonActuator servo: limiter-scoped deadband, fast decisive down-moves, patient
  exact restores, hold planning from the per-car profile, tap fallback."""

  def make_icbm(self, brand=""):
    return IntelligentCruiseButtonManagement(car.CarParams(pcmCruise=True, brand=brand),
                                             custom.CarParamsSP(pcmCruiseSpeed=False))

  def setup_method(self):
    self.icbm = self.make_icbm()

  def run_frames(self, target_mph, cluster_mph, n=1, source='sccVision', icbm=None, is_metric=False):
    icbm = icbm or self.icbm
    sends = []
    for _ in range(n):
      CS = car.CarState(cruiseState={"speedCluster": cluster_mph * CV.MPH_TO_MS})
      CC = car.CarControl(enabled=True)
      LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * CV.MPH_TO_MS)
      LP_SP.longitudinalPlanSource = source
      icbm.run(CS, CC, LP_SP, is_metric=is_metric)
      sends.append(icbm.cruise_button)
    return sends

  def test_limiter_jitter_within_deadband_no_send(self):
    self.run_frames(35, 35, n=60)
    assert self.icbm.state == State.holding

    sends = self.run_frames(35 - (REACT_DEADBAND - 1), 35, n=100)
    assert self.icbm.state == State.holding
    assert all(s == SendButtonState.none for s in sends)

  def test_limiter_down_move_beyond_deadband(self):
    self.run_frames(35, 35, n=60)

    sends = self.run_frames(35 - REACT_DEADBAND, 35, n=100)
    assert self.icbm.state == State.decreasing
    assert any(s == SendButtonState.decrease for s in sends)

  def test_transient_glitch_filtered(self):
    """A short-lived target drop (e.g. one bad map sample) must not trigger buttons."""
    self.run_frames(45, 45, n=60)
    assert self.icbm.state == State.holding

    sends = self.run_frames(25, 45, n=20)  # glitch shorter than REACT_TIMER (0.3s)
    sends += self.run_frames(45, 45, n=100)
    assert all(s == SendButtonState.none for s in sends)
    assert self.icbm.state == State.holding

  def test_runs_to_exact_target(self):
    """Once moving, ICBM steps all the way to the target, not just inside the deadband."""
    self.run_frames(45, 45, n=60)
    cluster = 45.
    for _ in range(600):
      sends = self.run_frames(35, cluster, n=1)
      if sends[0] == SendButtonState.decrease:
        cluster -= 1  # dash responds ~1 mph per press
    assert cluster == 35.
    assert self.icbm.state == State.holding

  def test_restore_waits_for_quiet_target(self):
    """After a limiter dip ends, the restore up must wait out RESTORE_QUIET_TIME on cars
    whose profile declares decel_needs_stable_setpoint: curves arrive in trains, and
    these ECUs won't decel while the set speed is moving."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(55, 55, n=60, icbm=icbm)
    assert icbm.state == State.holding

    # target back at the driver's 60; less than the quiet time elapsed -> no buttons yet
    sends = self.run_frames(60, 55, n=int(RESTORE_QUIET_TIME / DT_CTRL) - 50, source='cruise', icbm=icbm)
    assert all(s == SendButtonState.none for s in sends)
    assert icbm.state == State.holding

    # quiet time satisfied -> restore fires and runs
    self.run_frames(60, 55, n=100, source='cruise', icbm=icbm)
    assert icbm.state == State.increasing

  def test_default_profile_restores_without_patience(self):
    """Cars without decel_needs_stable_setpoint keep their fast restores; the quiet
    window is per-car behavior, not a global regression."""
    self.run_frames(55, 55, n=60)
    assert self.icbm.state == State.holding

    self.run_frames(60, 55, n=60, source='cruise')
    assert self.icbm.state == State.increasing

  def test_restore_is_exact_to_one_unit(self):
    """The F2 ratchet: a 1 mph residual against the driver setpoint must self-heal (no
    deadband against a cruise-source target)."""
    self.run_frames(59, 59, n=60, source='cruise')
    assert self.icbm.state == State.holding

    self.run_frames(60, 59, n=int(RESTORE_QUIET_TIME / DT_CTRL) + 100, source='cruise')
    assert self.icbm.state == State.increasing

  def test_moving_target_resets_restore_quiet(self):
    """An up-target that keeps moving (another dip building) never triggers a restore."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(55, 55, n=60, icbm=icbm)
    for _ in range(4):
      self.run_frames(60, 55, n=100, source='cruise', icbm=icbm)
      self.run_frames(59, 55, n=100, source='cruise', icbm=icbm)
    assert icbm.state == State.holding

  def test_hold_planned_for_coarse_moves(self):
    """Mazda profile, imperial: a move spanning >= one snap step starts as a hold, drops
    to taps for the remainder, and lands exactly."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(60, 60, n=60, icbm=icbm)

    self.run_frames(45, 60, n=60, icbm=icbm)
    assert icbm.state == State.decreasing
    assert icbm.cruise_button == SendButtonState.decreaseHold

    # ECU snaps 60 -> 55 -> 50; remaining 5 still holds, at 49 the remainder is taps
    self.run_frames(45, 50, n=5, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decreaseHold
    self.run_frames(45, 49, n=5, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decrease

    self.run_frames(45, 45, n=5, icbm=icbm)
    assert icbm.state == State.holding

  def test_hold_falls_back_to_taps_when_dash_frozen(self):
    """If a synthesized hold never lands a step, long-press is disabled for the drive."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(60, 60, n=60, icbm=icbm)

    self.run_frames(45, 60, n=60, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decreaseHold

    # dash frozen past first_step + margin -> fault and tap from here on
    self.run_frames(45, 60, n=150, icbm=icbm)
    assert icbm.longpress_faulted
    assert icbm.cruise_button == SendButtonState.decrease

    # a later coarse move stays taps-only
    self.run_frames(60, 60, n=200, icbm=icbm)
    self.run_frames(40, 60, n=60, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decrease

  def test_metric_plans_taps_only(self):
    """The Mazda long-press grid is only characterized in mph; metric must not hold."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(60, 60, n=60, icbm=icbm, is_metric=True)

    self.run_frames(45, 60, n=60, icbm=icbm, is_metric=True)
    assert icbm.state == State.decreasing
    assert icbm.cruise_button == SendButtonState.decrease


class TestDecelOvershoot:
  """Down-only overshoot: command the dash below the planner target so the stock ACC
  delivers the requested deceleration (its decel scales with dash-vs-vEgo gap)."""

  def make_icbm(self, brand="mazda"):
    return IntelligentCruiseButtonManagement(car.CarParams(pcmCruise=True, brand=brand),
                                             custom.CarParamsSP(pcmCruiseSpeed=False))

  def run_frames(self, icbm, target_mph, v_ego_mph, a_target, n=1, source='sccVision', enabled=True):
    for _ in range(n):
      CS = car.CarState(vEgo=v_ego_mph * CV.MPH_TO_MS,
                        cruiseState={"speedCluster": target_mph * CV.MPH_TO_MS})
      CC = car.CarControl(enabled=True)
      LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * CV.MPH_TO_MS, aTarget=a_target)
      LP_SP.longitudinalPlanSource = source
      icbm.run(CS, CC, LP_SP, is_metric=False, decel_overshoot_enabled=enabled)

  def test_commands_below_target_when_decelerating(self):
    icbm = self.make_icbm()
    # planner wants -0.45 m/s^2 at 45 mph toward a 40 mph target: needs a ~6 mph gap below vEgo
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target <= 39, icbm.v_target
    assert icbm.v_target >= 37, icbm.v_target

  def test_deep_dip_is_a_no_op(self):
    """When the target is already far below vEgo the plant is saturated; never go deeper."""
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=20, v_ego_mph=45, a_target=-1.0, n=100)
    assert icbm.v_target == 20, icbm.v_target

  def test_releases_back_to_target(self):
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target < 40
    # decel demand ends; command must return to the target (slew-limited release)
    self.run_frames(icbm, target_mph=40, v_ego_mph=40, a_target=0.0, n=400)
    assert icbm.v_target == 40, icbm.v_target

  def test_cruise_source_never_overshoots(self):
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100, source='cruise')
    assert icbm.v_target == 40, icbm.v_target

  def test_mazda_only(self):
    icbm = self.make_icbm(brand="hyundai")
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target == 40, icbm.v_target

  def test_toggle_off_disables_overshoot(self):
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100, enabled=False)
    assert icbm.v_target == 40, icbm.v_target
