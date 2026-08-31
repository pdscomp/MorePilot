from types import SimpleNamespace
from typing import cast
from unittest.mock import MagicMock, patch

import pytest

from opendbc.car.structs import car
from openpilot.cereal import custom, messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.controls.controlsd import Controls, LAT_SMOOTH_SECONDS
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

RAW_LAGD_DELAY = 0.485
FIXED_LAGD_DELAY = 0.335


class FakeSubMaster:
  def __init__(self, **messages):
    self.messages = messages
    self.valid = {"lateralManeuverPlan": True}

  def __getitem__(self, name):
    return self.messages[name]

  def all_checks(self, _services):
    return False


@pytest.fixture
def params():
  with OpenpilotPrefix():
    yield Params()


def build_controls(params: Params, live: bool, *, torque: bool = True) -> Controls:
  params.put_bool("LagdToggle", live, block=True)
  params.put("LagdToggleDelay", 0.235, block=True)
  params.put("LagdValueCache", RAW_LAGD_DELAY, block=True)  # stale live value must not become the fixed delay
  params.put("CarParamsSP", custom.CarParamsSP.new_message().to_bytes(), block=True)

  controls = Controls.__new__(Controls)
  CP = car.CarParams.new_message()
  CP.lateralTuning.init("torque" if torque else "pid")
  CP.steerControlType = car.CarParams.SteerControlType.torque
  CP.minSteerSpeed = 0.0
  CP.steerAtStandstill = True
  CP.steerActuatorDelay = 0.1
  CP.openpilotLongitudinalControl = True
  ControlsExt.__init__(controls, CP, params)

  controls.curvature = 0.0
  controls.desired_curvature = 0.0
  controls.calibrated_pose = None
  controls.steer_limited_by_safety = False

  car_state = car.CarState.new_message()
  car_state.vEgo = 5.0
  car_state.steeringAngleDeg = 0.0
  car_state.vCruise = 50.0

  controls.sm = cast(messaging.SubMaster, FakeSubMaster(
    carState=car_state,
    vehicleParameters=SimpleNamespace(stiffnessFactor=1.0, steerRatio=15.0, angleOffsetDeg=0.0, roll=0.0),
    lateralTorqueParameters=SimpleNamespace(useParams=False),
    modelV2=MagicMock(),
    selfdriveState=SimpleNamespace(enabled=True, active=True),
    onroadEvents=[],
    longitudinalPlan=SimpleNamespace(aTarget=0.0, shouldStop=False),
    lateralManeuverPlan=SimpleNamespace(desiredCurvature=0.0),
    lateralDelay=SimpleNamespace(lateralDelay=RAW_LAGD_DELAY),
  ))
  controls.VM = MagicMock()
  controls.VM.calc_curvature.return_value = 0.0
  controls.CI = MagicMock()
  controls.CI.get_pid_accel_limits.return_value = (-1.0, 1.0)
  controls.LoC = MagicMock()
  controls.LoC.long_control_state = "pid"
  controls.LoC.update.return_value = 0.0
  controls.LaC = MagicMock()
  controls.LaC.update.return_value = (0.0, 0.0, MagicMock())
  controls.get_lat_active = MagicMock(return_value=True)
  controls.update_lateral_assist = MagicMock(return_value=(0.0, 1.0))
  controls.blinker_pause_lateral = MagicMock()
  controls.turn_assist = MagicMock()
  controls.lane_change_smoothing = MagicMock()
  return controls


def run_state_control(controls: Controls) -> float:
  with patch("openpilot.selfdrive.controls.controlsd.clip_curvature", return_value=(0.0, False)):
    controls.state_control()
  return cast(MagicMock, controls.LaC.update).call_args.args[-1]


@pytest.mark.parametrize(("live", "expected"), [(True, RAW_LAGD_DELAY), (False, FIXED_LAGD_DELAY)])
def test_selected_delay_reaches_lateral_controller_on_startup(params, live, expected):
  controls = build_controls(params, live)
  assert run_state_control(controls) == pytest.approx(expected + LAT_SMOOTH_SECONDS)


def test_live_to_fixed_ignores_stale_live_cache(params):
  controls = build_controls(params, live=True)
  assert run_state_control(controls) == pytest.approx(RAW_LAGD_DELAY + LAT_SMOOTH_SECONDS)

  params.put_bool("LagdToggle", False, block=True)
  controls._param_update_time = 0.0
  controls.get_params_sp(controls.sm)

  assert run_state_control(controls) == pytest.approx(FIXED_LAGD_DELAY + LAT_SMOOTH_SECONDS)


def test_non_torque_uses_current_lagd_delay(params):
  controls = build_controls(params, live=False, torque=False)
  cast(FakeSubMaster, controls.sm).messages["lateralDelay"].lateralDelay = 0.222

  assert run_state_control(controls) == pytest.approx(0.222 + LAT_SMOOTH_SECONDS)


def test_fixed_selector_ignores_stale_live_cache(params):
  params.put_bool("LagdToggle", False, block=True)
  params.put("LagdToggleDelay", 0.235, block=True)
  params.put("LagdValueCache", RAW_LAGD_DELAY, block=True)

  assert get_lat_delay(params, RAW_LAGD_DELAY, 0.1) == pytest.approx(FIXED_LAGD_DELAY)
