"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Tests for ICBM (non-pcmCruiseSpeed) cruise handling:
- dash re-sync: the real dash is the source of truth around driver button presses
- ICBM state machine reaction deadband and persistence timer
- the vEgo clip on SET- while overriding is disabled for ICBM cars
"""
from cereal import car, custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import VCruiseHelper, IMPERIAL_INCREMENT
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
  IntelligentCruiseButtonManagement, REACT_DEADBAND)

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


class TestDashSync:
  """pcmCruise (stock ACC) car with ICBM enabled (pcmCruiseSpeed=False)."""

  def setup_method(self):
    Params().put_bool("CustomAccIncrementsEnabled", False)
    self.CP = car.CarParams(pcmCruise=True)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)

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
    """When ICBM holds the dash away from v_cruise (smart cruise), a press must not clobber v_cruise."""
    self.engage_at(45 * MPH)
    # smart cruise pushed the real dash down to 35 mph while v_cruise stays at 45 mph
    self.run_frames(make_car_state(dash_kph=35 * MPH), n=110)  # past any leftover sync window
    assert abs(self.v_cruise_helper.v_cruise_kph - 45 * MPH) < 0.1

    self.press(ButtonType.accelCruise, dash_kph=35 * MPH)
    self.run_frames(make_car_state(dash_kph=36 * MPH), n=20)
    # v_cruise took its own +1 mph increment, not the dash value
    assert abs(self.v_cruise_helper.v_cruise_kph - (45 * MPH + IMPERIAL_INCREMENT)) < 0.5

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


class TestIcbmDeadband:
  def setup_method(self):
    self.CP = car.CarParams(pcmCruise=True)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    self.icbm = IntelligentCruiseButtonManagement(self.CP, self.CP_SP)

  def run_frames(self, target_mph, cluster_mph, n=1):
    sends = []
    for _ in range(n):
      CS = car.CarState(cruiseState={"speedCluster": cluster_mph * CV.MPH_TO_MS})
      CC = car.CarControl(enabled=True)
      LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * CV.MPH_TO_MS)
      self.icbm.run(CS, CC, LP_SP, is_metric=False)
      sends.append(self.icbm.cruise_button)
    return sends

  def test_within_deadband_no_send(self):
    # settle into holding at equality first
    self.run_frames(35, 35, n=60)
    assert self.icbm.state == State.holding

    sends = self.run_frames(35 + REACT_DEADBAND - 1, 35, n=100)
    assert self.icbm.state == State.holding
    assert all(s == SendButtonState.none for s in sends)

  def test_beyond_deadband_sends(self):
    self.run_frames(35, 35, n=60)

    sends = self.run_frames(35 + REACT_DEADBAND, 35, n=100)
    assert self.icbm.state == State.increasing
    assert any(s == SendButtonState.increase for s in sends)

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
