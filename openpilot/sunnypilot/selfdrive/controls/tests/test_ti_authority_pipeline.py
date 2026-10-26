import numpy as np
import pytest

from opendbc.car.mazda.tests.test_mazda_steering import (
  SWAPPED_EPS_FW, _ti_controller, _wire_torque,
  car_control, car_control_sp, frame, set_car_state,
)
from opendbc.car.mazda.values import MazdaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_mazda import ti_command, ti_feedback
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.selfdrive.controls.lib.steer_limit import classify
from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import MazdaTorqueV2Mode
from openpilot.sunnypilot.selfdrive.controls.tests.test_latcontrol_torque_v42 import (
  LAT_DELAY, LP, VM, make_cs, make_mazda_ti_lac,
)


@pytest.fixture
def params():
  with OpenpilotPrefix():
    yield Params()


@pytest.mark.parametrize("direction", [-1.0, 1.0])
@pytest.mark.parametrize("speed", [5.0, 14.2, 14.5, 31.64])
def test_v42_reaches_ti_peak_without_weakening_panda(params, direction, speed):
  lac = make_mazda_ti_lac(MazdaTorqueV2Mode.A)
  cc, cs = _ti_controller(car_fw=SWAPPED_EPS_FW)
  safety = libsafety_py.libsafety
  flags = MazdaSafetyFlags.STEER_TO_ZERO_EPS | MazdaSafetyFlags.TORQUE_INTERCEPTOR
  assert safety.set_safety_hooks(CarParams.SafetyModel.mazda, flags) == 0
  safety.init_tests()
  for _ in range(6):
    assert safety.safety_rx_hook(ti_feedback())
  safety.set_controls_allowed(True)
  safety.set_controls_allowed_lateral(True)
  limited = False
  previous = None
  samples = []
  for i in range(300):
    safety.set_timer((i + 1) * 10_000)
    assert safety.safety_rx_hook(ti_feedback())
    state = make_cs(v_ego=speed, lat_accel=0.0)
    output, _, ctl = lac.update(
      True, state, VM, LP, limited, direction * 6.0 / speed**2,
      None, False, LAT_DELAY,
    )
    set_car_state(cs, v_ego=speed)
    cs.ti_lkas_allowed = True
    cs.ti_lkas_rejected = 0
    report, sends = cc.update(
      car_control(long_active=False, lat_active=True, torque=float(output)),
      car_control_sp(), cs, (i + 1) * 10_000_000,
    )
    data = frame(sends, 0x249, 1)
    assert safety.safety_tx_hook(libsafety_py.make_CANPacket(0x249, 1, data))
    command = _wire_torque(sends, 0x249, 1)
    assert abs(command) <= 600
    assert report.torqueOutputCan == command
    assert np.float32(report.torque) == np.float32(command / 600.0)
    mismatch = abs(float(output) - report.torque) > 1e-2
    limit = classify(float(output), report.torque, previous, 12 / 600, 25 / 600, 1.0,
                     mismatch, ctl.error, lac.pid.i)
    limited, previous = limit.limited, report.torque
    samples.append(command)
  assert ctl.version == 42
  assert samples[-1] == -direction * 600
  assert all(q == samples[-1] for q in samples[-10:])
  assert ctl.saturated == (speed > lac.sat_check_min_speed)
  assert not safety.safety_tx_hook(ti_command(int(-direction * 601)))
