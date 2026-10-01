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
  IntelligentCruiseButtonManagement, DRIVER_PRESS_GRACE_T, REACT_DEADBAND, RESTORE_QUIET_TIME)

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

MPH = CV.MPH_TO_KPH  # dash and v_cruise are tracked in kph; the CX-5 dash steps in whole mph


def make_car_state(dash_kph=0., gas_pressed=False, button_events=None, available=True, v_ego=0.):
  # US-cluster semantics: the PCM echo and the displayed number are the same
  CS = car.CarState(cruiseState={"available": available,
                                 "speed": dash_kph * CV.KPH_TO_MS,
                                 "speedCluster": dash_kph * CV.KPH_TO_MS})
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
    self.v_cruise_helper.update_plan_regime(LP_SP, CC_SP)
    self.v_cruise_helper.update_speed_limit_assist(False, LP_SP)
    # the arbiter owns the session now: an "SLA active" regime is arbiter state, not
    # a longitudinalPlanSP echo
    arb = self.v_cruise_helper.cruise_arbiter
    arb.enabled = True
    arb.state = getattr(custom.LongitudinalPlanSP.SpeedLimit.AssistState, sla_state)
    arb._speed_limit_prev = arb._speed_limit  # regime setup is not a limit-change edge

  def run_frames(self, CS, n=1, enabled=True):
    for _ in range(n):
      self.v_cruise_helper.update_v_cruise(CS, enabled=enabled, is_metric=False)
      # production wires the reconcile via CardExt.update_v_cruise_post after plan_regime
      self.v_cruise_helper.reconcile_setpoint_with_dash(CS)
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

  def run_frames(self, target_mph, cluster_mph, n=1, source='sccVision', icbm=None, is_metric=False,
                 v_ego_mph=None, a_target=0., overshoot=False, session_state=SessionState.disabled,
                 v_ahead_min_mph=0., button_events=None):
    icbm = icbm or self.icbm
    sends = []
    for i in range(n):
      CS = car.CarState(cruiseState={"speedCluster": cluster_mph * CV.MPH_TO_MS})
      if v_ego_mph is not None:
        CS.vEgo = float(v_ego_mph * CV.MPH_TO_MS)
      if button_events and i == 0:
        CS.buttonEvents = button_events
      CC = car.CarControl(enabled=True)
      LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * CV.MPH_TO_MS)
      LP_SP.longitudinalPlanSource = source
      LP_SP.aTarget = float(a_target)
      LP_SP.smartCruiseControl.vision.vAheadMin = float(v_ahead_min_mph * CV.MPH_TO_MS)
      icbm.run(CS, CC, LP_SP, is_metric=is_metric, decel_overshoot_enabled=overshoot,
               session_state=session_state)
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

  def test_fast_stream_for_large_moves(self):
    """A move with real distance runs the stream, drops to taps for the small remainder,
    and lands exactly. The dash walks in 1 mph presses: that is all the ECU ever does
    with forged frames."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(60, 60, n=60, icbm=icbm)

    self.run_frames(45, 60, n=60, icbm=icbm)
    assert icbm.state == State.decreasing
    assert icbm.cruise_button == SendButtonState.decreaseHold

    # dash walks down 1 mph at a time; the stream holds until the remainder is small
    self.run_frames(45, 48, n=5, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decreaseHold
    self.run_frames(45, 47, n=5, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decrease

    self.run_frames(45, 45, n=5, icbm=icbm)
    assert icbm.state == State.holding

  def test_stream_falls_back_to_taps_when_dash_frozen(self):
    """If the dash never moves under the stream, this ECU is not registering it; taps
    are the proven fallback for the rest of the drive."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(60, 60, n=60, icbm=icbm)

    self.run_frames(45, 60, n=60, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decreaseHold

    # dash frozen past the stall window -> fault and tap from here on
    self.run_frames(45, 60, n=160, icbm=icbm)
    assert icbm.fast_faulted
    assert icbm.cruise_button == SendButtonState.decrease

    # a later large move stays taps-only
    self.run_frames(60, 60, n=200, icbm=icbm)
    self.run_frames(40, 60, n=60, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decrease

  def test_metric_uses_the_stream_too(self):
    """The stream carries no grid assumption (it is just presses), so metric users get
    the fast walk as well."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(60, 60, n=60, icbm=icbm, is_metric=True)

    self.run_frames(45, 60, n=60, icbm=icbm, is_metric=True)
    assert icbm.state == State.decreasing
    assert icbm.cruise_button == SendButtonState.decreaseHold


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
    # planner wants -0.45 m/s^2 at 45 mph toward a 40 mph target: gap_v asks ~8.5 mph below
    # vEgo, leading the steady-state inverse to pay back the dash walk
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target <= 37, icbm.v_target
    assert icbm.v_target >= 35, icbm.v_target

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


class TestOpLongGuardrails:
  """Under openpilot longitudinal the planner brakes for curves itself: the servo keeps
  the dash on the driver's setpoint and the overshoot lever stays off."""

  def make_icbm(self, op_long):
    return IntelligentCruiseButtonManagement(car.CarParams(pcmCruise=True, brand="mazda",
                                                           openpilotLongitudinalControl=op_long),
                                             custom.CarParamsSP(pcmCruiseSpeed=False))

  def run_frames(self, icbm, target_mph, v_ego_mph, a_target, n=1, source='sccVision'):
    for _ in range(n):
      CS = car.CarState(vEgo=v_ego_mph * CV.MPH_TO_MS, vCruise=60 * CV.MPH_TO_KPH,
                        cruiseState={"speedCluster": 60 * CV.MPH_TO_MS})
      CC = car.CarControl(enabled=True)
      LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * CV.MPH_TO_MS, aTarget=float(a_target))
      LP_SP.longitudinalPlanSource = source
      icbm.run(CS, CC, LP_SP, is_metric=False, decel_overshoot_enabled=True)

  def test_dash_ignores_curve_target_under_op_long(self):
    icbm = self.make_icbm(op_long=True)
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target == 60  # the driver's setpoint, never the 40 mph curve target

  def test_overshoot_lever_off_under_op_long(self):
    icbm = self.make_icbm(op_long=True)
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.overshoot_mph == 0.0

  def test_stock_cruise_still_walks_the_curve(self):
    icbm = self.make_icbm(op_long=False)
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target <= 40  # unchanged non-OP-long behavior: the dash follows the plan


class TestDecelOvershootIsALever:
  """The overshoot commands the dash BELOW vEgo to buy real decel from the stock ACC. It
  is a lever the servo pulls, not a destination, so it is only valid while the servo can
  actually pull it and while the limiter that asked for it is still live. Both halves
  were missing: a pending SLA confirm prompt banked a gap for its whole 5 s window and
  the timeout dumped it as a SET- burst (user report 2026-08-29)."""

  def make_icbm(self):
    return IntelligentCruiseButtonManagement(car.CarParams(pcmCruise=True, brand="mazda"),
                                             custom.CarParamsSP(pcmCruiseSpeed=False))

  def run_frames(self, *args, **kwargs):
    return TestServo.run_frames(self, *args, **kwargs)

  def test_no_wind_up_behind_a_confirm_prompt(self):
    """Layer 2: a limiter asking for decel while a prompt is open must not accumulate a
    gap the servo is forbidden to emit."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm, overshoot=True)

    sends = self.run_frames(40, 40, n=500, icbm=icbm, source='speedLimitAssist', v_ego_mph=41.3,
                            a_target=-0.5, overshoot=True, session_state=SessionState.preActive)
    assert icbm.overshoot_mph == 0., f"banked behind the freeze: {icbm.overshoot_mph}"
    assert all(s == SendButtonState.none for s in sends)

    # prompt times out with the limiter gone: nothing is owed, so nothing moves
    sends = self.run_frames(40, 40, n=200, icbm=icbm, source='cruise', v_ego_mph=41.3, overshoot=True)
    assert all(s == SendButtonState.none for s in sends), "stale gap dumped at the timeout"
    assert icbm.state == State.holding

  def test_freeze_does_not_blunt_a_real_limiter(self):
    """Layer 2 must cost nothing: if the limiter is still asking for decel when the prompt
    clears, the gap rebuilds at DECEL_OVERSHOOT_RISE and the descent still happens."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm, overshoot=True)
    self.run_frames(40, 40, n=500, icbm=icbm, source='speedLimitAssist', v_ego_mph=41.3,
                    a_target=-0.5, overshoot=True, session_state=SessionState.preActive)
    assert icbm.overshoot_mph == 0.

    sends = self.run_frames(40, 40, n=100, icbm=icbm, source='speedLimitAssist', v_ego_mph=41.3,
                            a_target=-0.5, overshoot=True)
    assert icbm.overshoot_mph > 2., f"gap did not rebuild: {icbm.overshoot_mph}"
    # tap or hold is the profile's call from the remaining distance; either is a descent
    down = (SendButtonState.decrease, SendButtonState.decreaseHold)
    assert any(s in down for s in sends), "real limiter decel was blunted"

  def test_residual_gap_after_source_flip_starts_no_descent(self):
    """Layer 3: the lever outlives its limiter by design (slow release), but a residual
    must not START a fresh descent once the plan is back on cruise. Set directly: the
    gate is a single boolean and the state that reaches it is what matters."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm)
    icbm.overshoot_mph = 5.  # left over from a curve that just ended

    # off-limiter the residual drops at the build rate; check inside the bleed window
    sends = self.run_frames(40, 40, n=40, icbm=icbm, source='cruise', v_ego_mph=41.3, overshoot=True)
    assert icbm.overshoot_mph > 0., "precondition: the residual is still bleeding off"
    assert icbm.state == State.holding, f"descended on a residual: {icbm.state}"
    assert all(s == SendButtonState.none for s in sends)
    sends = self.run_frames(40, 40, n=60, icbm=icbm, source='cruise', v_ego_mph=41.3, overshoot=True)
    assert icbm.overshoot_mph == 0., "the residual must clear at the build rate once on cruise"
    assert all(s == SendButtonState.none for s in sends)

  def test_plain_setpoint_correction_still_unconditional(self):
    """Layer 3 must stay narrow: with no overshoot in play, a dash sitting above the
    driver's setpoint is a plain residual (a dropped press) and still self-heals."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm, source='cruise')

    sends = self.run_frames(40, 42, n=100, icbm=icbm, source='cruise')
    assert any(s == SendButtonState.decrease for s in sends), "dash residual stranded high"


class TestRestoreResponsiveness(TestServo):
  """Route 126 fixes: the quiet timer keys on the raw plan target (the overshoot lever's
  decay is not plan motion), the vision lookahead replaces stillness when present, and a
  genuine driver SET+ press parks down-moves for a grace window."""

  def test_restore_not_stalled_by_overshoot_decay(self):
    """After a limiter release with a built-up overshoot gap, the restore must start about
    a quiet-window after the flip, not after the residual finishes bleeding off."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm, v_ego_mph=40., overshoot=True)
    # curve: deep decel demand builds the full gap and walks the dash down
    self.run_frames(30, 31, n=100, icbm=icbm, v_ego_mph=39., a_target=-1.2, overshoot=True)
    assert icbm.overshoot_mph > 5.

    # road straightens: source back to cruise, target back at the driver's 40
    first_up = None
    for i in range(400):
      sends = self.run_frames(40, 31, n=1, icbm=icbm, source='cruise', v_ego_mph=33., overshoot=True)
      if sends[0] == SendButtonState.increase or sends[0] == SendButtonState.increaseHold:
        first_up = i * DT_CTRL
        break
    assert first_up is not None, "restore never started"
    assert first_up < RESTORE_QUIET_TIME + 0.6, f"restore stalled {first_up:.2f}s behind the decay"

  def test_lookahead_dip_blocks_restore(self):
    """A dip below the target on the vision horizon holds the restore regardless of how
    quiet the target is: restoring between bends feeds the next apex."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm)
    sends = self.run_frames(40, 30, n=300, icbm=icbm, source='cruise', v_ahead_min_mph=25.)
    assert all(s == SendButtonState.none for s in sends)
    assert icbm.state == State.holding

  def test_lookahead_clear_skips_quiet_window(self):
    """With the horizon clear the profile is the churn oracle; stillness is redundant and
    the restore fires on the react timer alone."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm)
    first_up = None
    for i in range(200):
      sends = self.run_frames(40, 30, n=1, icbm=icbm, source='cruise', v_ahead_min_mph=255. / CV.MPH_TO_MS)
      if sends[0] in (SendButtonState.increase, SendButtonState.increaseHold):
        first_up = i * DT_CTRL
        break
    assert first_up is not None
    assert first_up < RESTORE_QUIET_TIME, f"lookahead-clear restore still waited {first_up:.2f}s"

  def test_dip_appearing_mid_restore_aborts(self):
    """The commit gate trails the profile; a dip appearing while stepping up must stop the
    restore instead of accelerating until the limiter takes the source."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm)
    self.run_frames(40, 34, n=150, icbm=icbm, source='cruise', v_ahead_min_mph=200.)
    assert icbm.state == State.increasing

    sends = self.run_frames(40, 35, n=100, icbm=icbm, source='cruise', v_ahead_min_mph=25.)
    assert icbm.state == State.holding
    assert all(s == SendButtonState.none for s in sends[5:])

  def test_driver_up_press_grace_blocks_down(self):
    """A genuine SET+ press parks synthesized down-moves for the grace window even while a
    limiter demands them; the servo resumes once the window expires."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm)
    press = [ButtonEvent(type=ButtonType.accelCruise, pressed=True)]
    release = [ButtonEvent(type=ButtonType.accelCruise, pressed=False)]
    self.run_frames(30, 40, n=5, icbm=icbm, button_events=press)
    self.run_frames(30, 45, n=1, icbm=icbm, button_events=release)

    quiet, resumed = [], []
    for i in range(int(DRIVER_PRESS_GRACE_T / DT_CTRL) + 200):
      sends = self.run_frames(30, 45, n=1, icbm=icbm, a_target=-1.0)
      (quiet if i * DT_CTRL < DRIVER_PRESS_GRACE_T - 0.1 else resumed).extend(sends)
    assert all(s == SendButtonState.none for s in quiet), "servo fought the driver inside the grace window"
    assert any(s in (SendButtonState.decrease, SendButtonState.decreaseHold) for s in resumed), \
      "servo never resumed after the grace window"

  def test_driver_down_press_ends_grace(self):
    """A SET- press is aligned intent and cancels the SET+ grace immediately."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm)
    self.run_frames(30, 40, n=5, icbm=icbm, button_events=[ButtonEvent(type=ButtonType.accelCruise, pressed=True)])
    self.run_frames(30, 45, n=1, icbm=icbm, button_events=[ButtonEvent(type=ButtonType.accelCruise, pressed=False)])
    self.run_frames(30, 45, n=3, icbm=icbm, button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=True)])
    self.run_frames(30, 45, n=1, icbm=icbm, button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=False)])
    assert icbm.down_grace_timer == 0

    sends = self.run_frames(30, 45, n=150, icbm=icbm)
    assert any(s in (SendButtonState.decrease, SendButtonState.decreaseHold) for s in sends)

  def test_driver_down_press_grace_blocks_up(self):
    """The mirror: after a genuine SET- press a refused re-anchor must not restore the
    baseline over the driver's head, even with the lookahead clear."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm, source='cruise')
    self.run_frames(40, 40, n=5, icbm=icbm, source='cruise',
                    button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=True)])
    self.run_frames(40, 35, n=1, icbm=icbm, source='cruise',
                    button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=False)])

    quiet, resumed = [], []
    for i in range(int(DRIVER_PRESS_GRACE_T / DT_CTRL) + 300):
      sends = self.run_frames(40, 35, n=1, icbm=icbm, source='cruise', v_ahead_min_mph=200.)
      (quiet if i * DT_CTRL < DRIVER_PRESS_GRACE_T - 0.1 else resumed).extend(sends)
    assert all(s == SendButtonState.none for s in quiet), "servo restored over the driver's SET-"
    assert any(s in (SendButtonState.increase, SendButtonState.increaseHold) for s in resumed), \
      "restore never resumed after the grace window"
