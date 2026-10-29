"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
from types import SimpleNamespace

import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v42 import LatControlTorque as V42
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v43 import LatControlTorque as V43
from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import MazdaTorqueV2Mode
from openpilot.sunnypilot.selfdrive.controls.tests.test_latcontrol_torque_v42 import (
  make_cs, make_mazda_ti_lac, make_plan_model, params, step,
)


@pytest.mark.parametrize("speed", [0.5, 2.0, 5.0, 7.5, 10.0, 12.0, 14.5, 30.0, 35.0])
def test_v43_matches_v42_a_outside_added_damping(params, speed):
  baseline = make_mazda_ti_lac(MazdaTorqueV2Mode.A, cls=V42)
  candidate = make_mazda_ti_lac(MazdaTorqueV2Mode.A, cls=V43)
  for i in range(160):
    cs = make_cs(speed, 0.08 * math.sin(i * 0.04), pressed=80 <= i < 95)
    logs = [step(lac, cs, 0.15 * math.sin(i * 0.05) / speed**2, active=i >= 40)
            for lac in (baseline, candidate)]
    assert (logs[0].version, logs[1].version) == (42, 43)
    for field in ("p", "i", "d", "f", "output", "desiredLateralAccel", "desiredLateralJerk"):
      assert getattr(logs[0], field) == pytest.approx(getattr(logs[1], field), abs=1e-9)
    assert candidate.pid.pos_limit == baseline.pid.pos_limit
    assert candidate.pid.neg_limit == baseline.pid.neg_limit


@pytest.mark.parametrize("active", [False, True])
def test_zero_speed_matches_v42(params, active):
  a, b = [make_mazda_ti_lac(MazdaTorqueV2Mode.A, cls=cls) for cls in (V42, V43)]
  cs = SimpleNamespace(vEgo=0.0, aEgo=0.0, steeringAngleDeg=0.0, steeringRateDeg=0.0, steeringPressed=False)
  la, lb = step(a, cs, 0.0, active), step(b, cs, 0.0, active)
  assert la.output == lb.output
  assert la.errorRate == lb.errorRate == 0.0


@pytest.mark.parametrize("speed,kd", [(14.5, 0.0), (17.5, 0.1), (20.0, 0.1), (25.0, 0.1), (30.0, 0.0)])
def test_schedule_and_continuity(params, speed, kd):
  lac = make_mazda_ti_lac(MazdaTorqueV2Mode.A, cls=V43)
  lac.pid.speed = speed
  assert lac.pid.k_d == pytest.approx(kd)
  for delta in (-1e-5, 1e-5):
    lac.pid.speed = speed + delta
    assert lac.pid.k_d == pytest.approx(kd, abs=5e-6)


@pytest.mark.parametrize("direction", [-1, 1])
def test_measurement_derivative_and_driver_override(params, direction):
  old, new = [make_mazda_ti_lac(MazdaTorqueV2Mode.A, cls=cls) for cls in (V42, V43)]
  for lac in (old, new):
    step(lac, make_cs(20.0), 0.0, active=False)
  for i in range(20):
    cs = make_cs(20.0, direction * i * 0.001)
    old_log, new_log = [step(lac, cs, 0.0) for lac in (old, new)]
  assert old_log.d == 0.0
  assert new_log.d * direction < 0.0
  assert new_log.d == pytest.approx(0.1 * new_log.errorRate, rel=1e-6)
  pressed = step(new, make_cs(20.0, direction * 0.03, pressed=True), 0.0)
  assert pressed.errorRate == pressed.d == 0.0
  released = step(new, make_cs(20.0, direction * 0.031), 0.0)
  assert released.d * direction < 0.0
  inactive = step(new, make_cs(20.0, direction * 0.04), 0.0, active=False)
  assert inactive.errorRate == 0.0
  for _ in range(250):
    settled = step(new, make_cs(20.0, direction * 0.04), 0.0)
  assert abs(settled.d) < 1e-6
  step(new, make_cs(20.0, direction * 0.04), 0.01, active=False)
  desired_step = step(new, make_cs(20.0, direction * 0.04), 0.02)
  assert desired_step.d == 0.0


def test_speed_sweep_braking_is_finite_and_continuous(params):
  lac = make_mazda_ti_lac(MazdaTorqueV2Mode.A, cls=V43)
  outputs = []
  for i in range(2201):
    speed = 32.0 - i * 0.01
    cs = make_cs(speed, 0.0005 * speed**2)
    log = step(lac, cs, 0.0005)
    assert math.isfinite(log.output) and abs(log.output) <= 1.0
    outputs.append(log.output)
  assert max(abs(b - a) for a, b in zip(outputs[200:], outputs[201:], strict=False)) < 0.05


@pytest.mark.parametrize("kind", ["fresh", "missing", "stale", "short", "divergent"])
def test_plan_fallback_matches_v42(params, kind):
  controllers = [make_mazda_ti_lac(MazdaTorqueV2Mode.A, cls=cls) for cls in (V42, V43)]
  # A fully diverged plan must clear the blend band (DIVERGENCE_BLEND_BP tops at 0.5 m/s^2);
  # 0.002 curvature at 20 m/s is a 0.8 m/s^2 request against the straight plan.
  desired_curvature = 0.002 if kind == "divergent" else 0.001
  for i in range(60):
    for lac in controllers:
      if kind != "missing":
        model = make_plan_model(1 if kind == "stale" else i, 0.0 if kind == "divergent" else 0.001, v_plan=20.0)
        if kind == "short":
          model.orientationRate.z = [0.0] * 10
        lac.extension.update_model_v2(model)
      step(lac, make_cs(20.0, 0.4), desired_curvature)
    assert controllers[0].plan_jerk_weight == controllers[1].plan_jerk_weight
  if kind in ("missing", "stale", "short", "divergent"):
    assert controllers[1].plan_jerk_weight == 0.0
  else:
    assert controllers[1].plan_jerk_weight == 1.0


@pytest.mark.parametrize("small,big,initial_big", [(43.0, 42.0, True), (42.0, 43.0, False)])
def test_real_handover_waits_and_primes(params, small, big, initial_big):
  from openpilot.sunnypilot.selfdrive.controls.controls_lateral_zp import ControlsLateralZP, TUNE_SWAP_INACTIVE_FRAMES
  from openpilot.sunnypilot.selfdrive.controls.tests.test_latcontrol_torque_v42 import make_ci
  from openpilot.sunnypilot.selfdrive.controls.tests.test_torque_tune_selection import FakeLaC, TestRealTorqueDispatch
  controls = TestRealTorqueDispatch._controls(params, small, big, True)
  controls.LaC = ControlsLateralZP.initialize_lateral_control(controls, FakeLaC('v1'), make_ci(), 0.01)
  if initial_big:
    ControlsLateralZP.select_lateral_control(controls, {'modelV2': SimpleNamespace(big=True)})
  current = controls.LaC
  incoming = controls._lac_by_size[not initial_big]
  for _ in range(100):
    step(incoming, make_cs(20.0, -0.4), -0.001)
  reference = make_mazda_ti_lac(MazdaTorqueV2Mode.A, cls=type(current))
  current.reset()
  reference.reset()
  for i in range(100):
    cs = make_cs(20.0, 0.03 * math.sin(i * 0.05))
    a, b = step(current, cs, 0.001), step(reference, cs, 0.001)
    ControlsLateralZP.note_lat_active(controls, True)
    ControlsLateralZP.select_lateral_control(controls, {'modelV2': SimpleNamespace(big=not initial_big)})
    assert controls.LaC is current
    assert a.output == pytest.approx(b.output, abs=1e-9)
  for _ in range(TUNE_SWAP_INACTIVE_FRAMES - 1):
    step(current, make_cs(20.0, 0.1), 0.001, active=False)
    ControlsLateralZP.note_lat_active(controls, False)
    ControlsLateralZP.select_lateral_control(controls, {'modelV2': SimpleNamespace(big=not initial_big)})
    assert controls.LaC is current
  ControlsLateralZP.note_lat_active(controls, False)
  ControlsLateralZP.select_lateral_control(controls, {'modelV2': SimpleNamespace(big=not initial_big)})
  assert controls.LaC is incoming
  # The existing immediate-reengagement edge has no incoming inactive priming.
  immediate = step(incoming, make_cs(20.0, 0.1), 0.001)
  assert math.isfinite(immediate.output)
  for _ in range(40):
    step(incoming, make_cs(20.0, 0.1), 0.001, active=False)
  assert incoming.previous_measurement == pytest.approx(0.1)
  assert math.isfinite(step(incoming, make_cs(20.0, 0.1), 0.001).output)
