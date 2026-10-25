"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Behavior tests for the v2 torque tune, run against the real controllers with a toy
# steering geometry (curvature proportional to steering angle) and a mocked car interface
# (torque == lat_accel / latAccelFactor). The load-bearing property is the first test:
# with its mechanisms quiescent (constant speed, friction 0, a plan that is not changing),
# v2 must be frame-for-frame identical to v0 — everything else is a deliberate, tested delta.

import math
from collections import deque
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from opendbc.car.mazda.values import MazdaFlags
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v2 import (
  LatControlTorque as LatControlTorqueV2,
  get_center_chatter_jerk_deadzone,
  MODEL_STALE_FRAMES,
  ti_lsf_scale,
)
from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import MazdaTorqueV2Mode

DT = 0.01
LAT_DELAY = 0.3
DELAY_FRAMES = int(LAT_DELAY / DT)
LAF = 2.5
CURV_PER_DEG = 2e-4  # toy geometry: curvature = -steeringAngleDeg * CURV_PER_DEG
MAZDA_TI_FLAGS = MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO | MazdaFlags.TORQUE_INTERCEPTOR

VM = SimpleNamespace(calc_curvature=lambda angle_rad, v_ego, roll: math.degrees(angle_rad) * CURV_PER_DEG)
LP = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)


def make_cp(friction=0.0):
  CP = car.CarParams.new_message(steerControlType="torque", steerLimitTimer=0.4)
  CP.lateralTuning.init('torque')
  CP.lateralTuning.torque.latAccelFactor = LAF
  CP.lateralTuning.torque.friction = friction
  return CP.as_reader()


def make_ci():
  CI = MagicMock()
  CI.torque_from_lateral_accel.return_value = lambda lataccel, tp: lataccel / tp.latAccelFactor
  CI.lateral_accel_from_torque.return_value = lambda torque, tp: torque * tp.latAccelFactor
  return CI


def make_lac(cls, friction=0.0, **kwargs):
  return cls(make_cp(friction=friction), custom.CarParamsSP.new_message().as_reader(), make_ci(), DT, **kwargs)


def make_mazda_ti_lac(mode, fingerprint="MAZDA_CX5", flags=MAZDA_TI_FLAGS):
  CP = car.CarParams.new_message(
    steerControlType="torque",
    steerLimitTimer=0.4,
    carFingerprint=fingerprint,
    steerAtStandstill=True,
    brand="mazda",
    flags=int(flags),
  )
  CP.lateralTuning.init("torque")
  CP.lateralTuning.torque.latAccelFactor = LAF
  return LatControlTorqueV2(
    CP.as_reader(), custom.CarParamsSP.new_message().as_reader(), make_ci(), DT,
    mazda_v2_mode=mode,
  )


def make_pair(friction=0.0):
  return make_lac(LatControlTorqueV0, friction=friction), make_lac(LatControlTorqueV2, friction=friction)


def make_cs(v_ego=15.0, lat_accel=0.0, pressed=False):
  """CarState whose measured lateral accel equals lat_accel at v_ego."""
  angle = -lat_accel / (CURV_PER_DEG * v_ego ** 2)
  return SimpleNamespace(vEgo=v_ego, aEgo=0.0, steeringAngleDeg=angle, steeringRateDeg=0.0, steeringPressed=pressed)


def step(lac, cs, desired_curvature, active=True, lp=LP, lat_delay=LAT_DELAY):
  _, _, pid_log = lac.update(active, cs, VM, lp, False, desired_curvature, None, False, lat_delay)
  return pid_log


@pytest.fixture
def params():
  with OpenpilotPrefix():
    yield Params()


class TestMazdaTorqueV2AB:
  @pytest.mark.parametrize(("v_ego", "a", "b"), [
    (0.0, 0.5, 0.5),
    (5.0, 0.5, 0.5),
    (6.7, 0.67, 0.636),
    (7.5, 0.75, 0.70),
    (9.0, 0.90, 0.88),
    (10.0, 1.0, 1.0),
    (30.0, 1.0, 1.0),
  ])
  def test_exact_scale_pins(self, v_ego, a, b):
    assert ti_lsf_scale(v_ego, MazdaTorqueV2Mode.A) == pytest.approx(a, abs=1e-3)
    assert ti_lsf_scale(v_ego, MazdaTorqueV2Mode.B) == pytest.approx(b, abs=1e-3)

  def test_modes_share_kd_and_maximum_authority(self, params):
    a = make_mazda_ti_lac(MazdaTorqueV2Mode.A)
    b = make_mazda_ti_lac(MazdaTorqueV2Mode.B)
    assert b.mazda_v2_mode == MazdaTorqueV2Mode.B
    assert a.KD_SCHEDULE == b.KD_SCHEDULE
    assert a.steer_rail_schedule == b.steer_rail_schedule
    assert a.steer_max == b.steer_max
    assert a.pid.pos_limit == pytest.approx(b.pid.pos_limit)
    assert a.pid.neg_limit == pytest.approx(b.pid.neg_limit)

  def test_flagless_cx5_b_is_a_but_not_applicable(self):
    flags = MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO
    lac = make_mazda_ti_lac(MazdaTorqueV2Mode.B, flags=flags)
    assert lac.mazda_v2_mode is None
    assert ti_lsf_scale(7.5, lac.mazda_v2_mode) == ti_lsf_scale(7.5, MazdaTorqueV2Mode.A)

  def test_native_2022_is_a_but_not_applicable(self):
    lac = make_mazda_ti_lac(MazdaTorqueV2Mode.B, fingerprint="MAZDA_CX5_2022")
    assert lac.mazda_v2_mode is None
    assert ti_lsf_scale(7.5, lac.mazda_v2_mode) == ti_lsf_scale(7.5, MazdaTorqueV2Mode.A)

  @pytest.mark.parametrize("mode", [MazdaTorqueV2Mode.A, MazdaTorqueV2Mode.B])
  def test_recorded_cohort_reports_effective_mode(self, mode):
    lac = make_mazda_ti_lac(mode)
    assert lac.mazda_v2_mode == mode


class TestLatControlTorqueV2:
  def test_unchanging_plan_matches_v0(self, params):
    """At constant speed with friction 0 and a plan that is not changing, the setpoint lead
    is exactly zero and v2's setpoint algebra collapses to v0's — outputs must match frame
    for frame while the measurement moves under both."""
    v0, v2 = make_pair()
    v_ego = 15.0
    desired = 2e-3
    # prime v2's curvature buffer with the constant plan while inactive, so the lead term is
    # zero from the first engaged frame; the on-request measurement leaves both rate filters at rest.
    # (v0 identity holds only on the zero-error manifold: v2's low-speed error boost scales any
    # nonzero error at every speed; that delta is pinned in test_low_speed_error_boost.)
    for _ in range(DELAY_FRAMES + 10):
      step(v0, make_cs(v_ego, desired * v_ego ** 2), desired, active=False)
      step(v2, make_cs(v_ego, desired * v_ego ** 2), desired, active=False)
    for i in range(300):
      out0, _, log0 = v0.update(True, make_cs(v_ego, desired * v_ego ** 2), VM, LP, False, desired, None, False, LAT_DELAY)
      out2, _, log2 = v2.update(True, make_cs(v_ego, desired * v_ego ** 2), VM, LP, False, desired, None, False, LAT_DELAY)
      assert log2.desiredLateralAccel == pytest.approx(log0.desiredLateralAccel, abs=1e-6), f"frame {i}"
      assert log2.error == pytest.approx(0.0, abs=1e-9), f"frame {i}"
      assert out2 == pytest.approx(out0, abs=1e-9), f"frame {i}"
    assert log0.version == 0
    assert log2.version == 2

  def test_no_phantom_jerk_when_decelerating_through_constant_curvature(self, params):
    """Mechanism 1: braking through a constant-curvature arc, the delayed request rescaled by
    the current v^2 equals the live request, so v2 sees zero desired jerk. v0's lat-accel
    buffer replays the higher old-speed values and reads a phantom jerk."""
    v0, v2 = make_pair()
    desired = 2e-3
    v_ego = 25.0
    for _ in range(150):  # settle in the curve at constant speed first
      step(v0, make_cs(v_ego, desired * v_ego ** 2), desired)
      step(v2, make_cs(v_ego, desired * v_ego ** 2), desired)
    v0_jerks, v2_jerks = [], []
    for _ in range(200):
      v_ego = max(v_ego - 0.04, 10.0)  # -4 m/s^2
      log0 = step(v0, make_cs(v_ego, desired * v_ego ** 2), desired)
      log2 = step(v2, make_cs(v_ego, desired * v_ego ** 2), desired)
      v0_jerks.append(abs(log0.desiredLateralJerk))
      v2_jerks.append(abs(log2.desiredLateralJerk))
    assert max(v2_jerks) < 1e-3
    assert max(v0_jerks) > 0.2

  def test_setpoint_leads_the_request_by_the_steering_delay(self, params):
    """v2 always steers to the delayed request plus one lat_delay of filtered planned jerk,
    so turn-in starts a steering delay early. v0 steers to the live request; the two differ
    through the transient and converge back to the request in steady state."""
    v0, v2 = make_pair()

    v_ego = 15.0
    transient_diff = 0.0
    for i in range(400):
      desired = 0.0 if i < 100 else 2e-3  # step turn-in
      log_v0 = step(v0, make_cs(v_ego), desired)
      log_v2 = step(v2, make_cs(v_ego), desired)
      request = desired * v_ego ** 2
      assert log_v0.desiredLateralAccel == pytest.approx(request, abs=1e-6)  # float32 log field
      if 100 <= i < 120:
        transient_diff = max(transient_diff, abs(log_v2.desiredLateralAccel - request))
    assert transient_diff > 0.05  # the lead reshapes the transient
    assert log_v2.desiredLateralAccel == pytest.approx(2e-3 * v_ego ** 2, abs=1e-3)  # steady state converges

  def test_extension_output_overrides_disabled(self, params):
    """v2 owns the friction shaping and integrator policy, so extension controllers that
    override the shared PID (jerk-aware, NNLC, any future sibling) are disabled no matter
    what the params say, and the PID limits stay in lat-accel space. v0 constructed with
    the same params keeps jerk-aware on (torque-space limits) — pinning that the disable
    is v2's, not a side effect."""
    params.put_bool("LateralJerkTorqueController", True, block=True)
    params.put_bool("NeuralNetworkLateralControl", True, block=True)
    v2 = make_lac(LatControlTorqueV2)
    assert not v2.extension.overrides_output
    assert v2.pid.pos_limit == pytest.approx(LAF)
    # update_limits must stay a no-op on the extension, or the per-frame speed-dep
    # override path would restore torque-space limits
    v2.update_limits()
    assert v2.pid.pos_limit == pytest.approx(LAF)

    v0 = make_lac(LatControlTorqueV0)
    assert v0.extension.overrides_output
    assert v0.pid.pos_limit == pytest.approx(v0.steer_max)

  def test_release_decay_is_one_shot(self, params):
    """Handing back the wheel decays the integrator once (x0.8), not every frame."""
    v2 = make_lac(LatControlTorqueV2)
    v2.pid.i = 1.0
    step(v2, make_cs(pressed=True), 0.0)  # frozen while pressed
    assert v2.pid.i == pytest.approx(1.0)
    step(v2, make_cs(pressed=False), 0.0)  # falling edge: one-shot decay, error is 0
    assert v2.pid.i == pytest.approx(0.8)
    step(v2, make_cs(pressed=False), 0.0)
    assert v2.pid.i == pytest.approx(0.8)

    v0 = make_lac(LatControlTorqueV0)
    v0.pid.i = 1.0
    step(v0, make_cs(pressed=True), 0.0)
    step(v0, make_cs(pressed=False), 0.0)
    assert v0.pid.i == pytest.approx(1.0)  # v0 has no release decay

  def test_unwind_freezes_integrator(self, params):
    """While the plan unwinds through center (setpoint rate < -1 m/s^3, |setpoint| < 0.3),
    the integrator holds instead of integrating the transient error."""
    v2 = make_lac(LatControlTorqueV2)
    v_ego = 15.0
    hold = 0.25 / v_ego ** 2  # steady setpoint 0.25 m/s^2, inside the near-zero band
    # settle the setpoint lead's jerk filter with the measurement on the request, so the
    # integrator enters the unwind at a value this test sets rather than a wound-up one
    for _ in range(300):
      step(v2, make_cs(v_ego, 0.25), hold)
    assert v2.prev_setpoint == pytest.approx(0.25)
    v2.pid.i = 0.05

    # ramp the plan to zero over 4 frames; the measurement stays on the old request, so a
    # live integrator would keep winding down against the error the unwind opens up
    i_during = []
    for k in range(10):
      step(v2, make_cs(v_ego, 0.25), hold * max(0.0, 1 - 0.25 * k))
      i_during.append(v2.pid.i)
    assert i_during[2] != pytest.approx(0.05)  # the pre-freeze frames did integrate
    assert all(i == pytest.approx(i_during[3]) for i in i_during[3:])  # frozen through the unwind

    for _ in range(8):
      step(v2, make_cs(v_ego, 0.25), 0.0)  # settled again: rate 0, integrating resumes
    assert v2.pid.i != pytest.approx(i_during[-1])

  def test_unwind_freezes_integrator_left_turn(self, params):
    """Mirror of the test above with a negative setpoint: exiting a LEFT turn the setpoint
    rate is POSITIVE, so unwind detection must be measured relative to the side being
    exited — a bare `rate < threshold` never fires here (the original sign bug)."""
    v2 = make_lac(LatControlTorqueV2)
    v_ego = 15.0
    hold = -0.25 / v_ego ** 2
    for _ in range(300):
      step(v2, make_cs(v_ego, -0.25), hold)
    assert v2.prev_setpoint == pytest.approx(-0.25)
    v2.pid.i = -0.05

    i_during = []
    for k in range(10):
      step(v2, make_cs(v_ego, -0.25), hold * max(0.0, 1 - 0.25 * k))
      i_during.append(v2.pid.i)
    assert i_during[2] != pytest.approx(-0.05)  # the pre-freeze frames did integrate
    assert all(i == pytest.approx(i_during[3]) for i in i_during[3:])  # frozen through the unwind

  def test_left_turn_entry_does_not_freeze(self, params):
    """Entering a left turn from center the setpoint rate is negative through the
    near-zero band — the raw-rate condition would wrongly freeze the integrator for the
    whole turn-in. Direction-relative unwind must keep integrating."""
    v2 = make_lac(LatControlTorqueV2)
    v_ego = 15.0
    target = -0.25 / v_ego ** 2
    for _ in range(300):
      step(v2, make_cs(v_ego, 0.0), 0.0)
    v2.pid.i = 0.0

    # ramp the plan into the left turn fast enough that the raw setpoint rate crosses
    # the -1 m/s^3 threshold while |setpoint| is still inside the 0.3 m/s^2 band;
    # the measurement lags at zero, so a live integrator winds negative
    i_vals = []
    for k in range(20):
      step(v2, make_cs(v_ego, 0.0), target * min(1.0, 0.1 * (k + 1)))
      i_vals.append(v2.pid.i)
    assert any(a != pytest.approx(b) for a, b in zip(i_vals, i_vals[1:], strict=False)), \
      "integrator must keep integrating on turn entry"
    assert i_vals[-1] < 0.0  # winding toward the left-turn error, not held at zero

  def test_integrator_active_at_creep_speeds(self, params):
    """v0 freezes the integrator below 5 m/s; v2 keys the freeze/reset to
    max(minSteerSpeed, 0.3) so it keeps working at creep speeds."""
    v0, v2 = make_pair()
    v_ego = 3.0
    desired = 0.05 / v_ego ** 2  # small error: the boosted low-speed P gain must not clip the output
    for _ in range(50):
      log0 = step(v0, make_cs(v_ego), desired)
      log2 = step(v2, make_cs(v_ego), desired)
    assert log0.i == pytest.approx(0.0)
    assert log2.i > 0.0

    step(v2, make_cs(0.2), desired)  # below the threshold: full reset
    assert v2.pid.i == pytest.approx(0.0)

  def test_inactive_priming_preserves_integrator_and_buffer(self, params):
    """While lateral is inactive the buffer and rate state track the live command (no
    re-engage shove) and the integrator is deliberately NOT cleared (MADS cycles lateral
    often; the release decay and unwind freeze replace a blunt reset). v0's stale buffer
    reads a large phantom jerk on the first active frame."""
    v0, v2 = make_pair()
    v2.pid.i = 0.5
    v_ego = 15.0
    desired = 2e-3
    for _ in range(2 * DELAY_FRAMES):
      step(v0, make_cs(v_ego, desired * v_ego ** 2), desired, active=False)
      step(v2, make_cs(v_ego, desired * v_ego ** 2), desired, active=False)
    assert v2.pid.i == pytest.approx(0.5)

    log0 = step(v0, make_cs(v_ego, desired * v_ego ** 2), desired, active=True)
    log2 = step(v2, make_cs(v_ego, desired * v_ego ** 2), desired, active=True)
    assert abs(log2.desiredLateralJerk) < 1e-6
    assert abs(log0.desiredLateralJerk) > 1.0

  def test_roll_compensation_fades_at_creep(self, params):
    """Below walking pace the road-crown feedforward fades out instead of unwinding a held
    wheel at pull-away. At 1.0 m/s the fade is 0.25 of v0's full compensation."""
    v0, v2 = make_pair()
    lp_roll = SimpleNamespace(angleOffsetDeg=0.0, roll=0.1)
    log0 = step(v0, make_cs(1.0), 0.0, lp=lp_roll)
    log2 = step(v2, make_cs(1.0), 0.0, lp=lp_roll)
    assert log0.f != 0.0
    assert log2.f == pytest.approx(0.25 * log0.f)

    log0 = step(v0, make_cs(15.0), 0.0, lp=lp_roll)
    log2 = step(v2, make_cs(15.0), 0.0, lp=lp_roll)
    assert log2.f == pytest.approx(log0.f)  # fully faded in above 2.5 m/s

  def test_center_chatter_deadzone_curve(self):
    """Full-strength at lane center, gone above 0.35 m/s^2 of demand."""
    assert get_center_chatter_jerk_deadzone(25.0, 0.0) == pytest.approx(0.18)
    assert get_center_chatter_jerk_deadzone(25.0, 0.5) == pytest.approx(0.0)
    assert get_center_chatter_jerk_deadzone(0.0, 0.0) == pytest.approx(0.08)

  def test_center_wobble_reduces_friction_activity(self, params):
    """With the wheel tracking the delayed command, planner wobble that flips v0's
    friction term stays inside the deadzone for v2."""
    v0, v2 = make_pair(friction=0.25)
    v_ego = 25.0
    history = deque([0.0] * (DELAY_FRAMES + 1), maxlen=DELAY_FRAMES + 1)
    v0_friction, v2_friction = [], []
    for i in range(600):
      # planner wobble around center: +-0.04 m/s^2 flipping every 0.4 s
      desired = math.copysign(0.04, math.sin(2 * math.pi * i / 80)) / v_ego ** 2
      measured_lat_accel = history[-DELAY_FRAMES] * v_ego ** 2  # wheel tracks the delayed command
      history.append(desired)
      log0 = step(v0, make_cs(v_ego, measured_lat_accel), desired)
      log2 = step(v2, make_cs(v_ego, measured_lat_accel), desired)
      if i > 200:
        request = desired * v_ego ** 2
        v0_friction.append(abs(log0.f - request))  # roll and offset are 0: f - request is the friction term
        v2_friction.append(abs(log2.f - request))
    assert sum(v2_friction) < 0.5 * sum(v0_friction)


class TestRailAwareSaturation:
  """A railed EPS must raise the saturation warning even though the carcontroller's
  ceiling clamp reports it as steer_limited_by_safety; sub-rail safety limiting
  (driver-torque narrowing) keeps its suppression, and platforms without a rail
  schedule keep stock semantics."""

  SAT_FRAMES = int(0.4 / DT) + 20  # steerLimitTimer plus margin

  @staticmethod
  def _step_sls(lac, cs, desired_curvature, sls):
    _, _, pid_log = lac.update(True, cs, VM, LP, sls, desired_curvature, None, False, LAT_DELAY)
    return pid_log

  def _run(self, lac, lat_accel_demand, sls, frames):
    cs = make_cs(v_ego=15.0, lat_accel=0.0)
    desired_curvature = lat_accel_demand / 15.0 ** 2
    log = None
    for _ in range(frames):
      log = self._step_sls(lac, cs, desired_curvature, sls)
    return log

  def test_railed_eps_raises_saturation(self, params):
    lac = make_lac(LatControlTorqueV2)
    lac.steer_rail_schedule = ([0.0, 30.0], [0.6, 0.6])
    log = self._run(lac, lat_accel_demand=4.5, sls=True, frames=self.SAT_FRAMES)
    assert log.saturated

  def test_not_saturated_before_timer(self, params):
    lac = make_lac(LatControlTorqueV2)
    lac.steer_rail_schedule = ([0.0, 30.0], [0.6, 0.6])
    log = self._run(lac, lat_accel_demand=4.5, sls=True, frames=5)
    assert not log.saturated

  def test_sub_rail_safety_limit_keeps_suppression(self, params):
    lac = make_lac(LatControlTorqueV2)
    lac.steer_rail_schedule = ([0.0, 30.0], [0.6, 0.6])
    # measurement tracks the demand: no error, output = ff = 0.2 of scale, under the rail
    cs = make_cs(v_ego=15.0, lat_accel=0.5)
    log = None
    for _ in range(self.SAT_FRAMES * 2):
      log = self._step_sls(lac, cs, 0.5 / 15.0 ** 2, sls=True)
    assert not log.saturated

  def test_no_schedule_keeps_stock_suppression(self, params):
    lac = make_lac(LatControlTorqueV2)
    assert lac.steer_rail_schedule is None
    log = self._run(lac, lat_accel_demand=6.0, sls=True, frames=self.SAT_FRAMES * 2)
    assert not log.saturated

  def test_no_schedule_full_scale_still_saturates_without_sls(self, params):
    lac = make_lac(LatControlTorqueV2)
    log = self._run(lac, lat_accel_demand=6.0, sls=False, frames=self.SAT_FRAMES)
    assert log.saturated


class TestRailLimitedPid:
  """The PID is limited to the torque the EPS will actually deliver, not the full steer scale,
  so its own directional anti-windup engages at the real rail. Delivered counts are unchanged
  (the carcontroller clamps there regardless); what changes is that a railed integrator can
  still unwind instead of only being frozen from outside."""

  RAIL = 0.5

  def _railed(self, rail=RAIL):
    lac = make_lac(LatControlTorqueV2)
    if rail is not None:
      lac.steer_rail_schedule = ([0.0], [rail])
    return lac

  def test_limits_track_the_rail(self, params):
    lac = self._railed()
    step(lac, make_cs(15.0), 0.0)
    assert lac.pid.pos_limit == pytest.approx(self.RAIL * LAF)
    assert lac.pid.neg_limit == pytest.approx(-self.RAIL * LAF)

  def test_no_schedule_keeps_full_scale_limits(self, params):
    lac = self._railed(rail=None)
    assert lac.steer_rail_schedule is None
    step(lac, make_cs(15.0), 0.0)
    assert lac.pid.pos_limit == pytest.approx(LAF)

  def test_limits_follow_speed(self, params):
    """A falling ceiling must tighten the limits as the car speeds up, not stay on whatever
    the schedule read at construction."""
    lac = make_lac(LatControlTorqueV2)
    lac.steer_rail_schedule = ([5.0, 25.0], [1.0, 0.5])
    step(lac, make_cs(5.0), 0.0)
    assert lac.pid.pos_limit == pytest.approx(LAF)
    step(lac, make_cs(25.0), 0.0)
    assert lac.pid.pos_limit == pytest.approx(0.5 * LAF)

  def test_output_never_exceeds_the_rail(self, params):
    lac = self._railed()
    v_ego = 15.0
    demand = 3.0 / v_ego ** 2  # 3.0 m/s^2: well past the 1.25 m/s^2 the rail allows
    for _ in range(DELAY_FRAMES + 10):
      step(lac, make_cs(v_ego), demand, active=False)
    for _ in range(50):
      out, _, _ = lac.update(True, make_cs(v_ego, 0.0), VM, LP, False, demand, None, False, LAT_DELAY)
      assert abs(out) <= self.RAIL + 1e-9

  def _rail_then_reverse(self, sls, seed_i=0.4):
    """Rail the output on a large demand, seed the integrator, then flip the measurement past
    the request so the error reverses. Returns the integrator after the reversal."""
    v_ego = 15.0
    demand = 3.0 / v_ego ** 2
    lac = self._railed()
    for _ in range(DELAY_FRAMES + 10):
      step(lac, make_cs(v_ego), demand, active=False)
    for _ in range(100):
      lac.update(True, make_cs(v_ego, 0.0), VM, LP, sls, demand, None, False, LAT_DELAY)
    lac.pid.i = seed_i
    for _ in range(100):
      lac.update(True, make_cs(v_ego, 4.0), VM, LP, sls, demand, None, False, LAT_DELAY)
    return lac.pid.i

  def test_railed_integrator_decays_toward_a_reversing_error(self, params):
    """The load-bearing property: with the PID limits on the rail, an integrator facing a
    reversed error decays back out -- and since the safety freeze went directional, it does
    so whether or not steer_limited_by_safety is asserted (it used to sit pinned at the seed
    through a whole corner whenever sls fired)."""
    limited = self._rail_then_reverse(sls=True)
    free = self._rail_then_reverse(sls=False)
    assert limited < 0.4 - 0.05              # decays even while sls is asserted
    assert free < 0.4 - 0.05                 # decays back out of the rail
    assert limited == pytest.approx(free)    # sls no longer changes a pure-decay trajectory

  def test_railed_integrator_still_cannot_wind_into_the_rail(self, params):
    """The other half: a standing error that pushes further into the rail must not wind up."""
    v_ego = 15.0
    demand = 3.0 / v_ego ** 2
    lac = self._railed()
    for _ in range(DELAY_FRAMES + 10):
      step(lac, make_cs(v_ego), demand, active=False)
    lac.pid.i = 0.0
    for _ in range(200):  # measurement stuck at zero: error stays large and positive
      lac.update(True, make_cs(v_ego, 0.0), VM, LP, False, demand, None, False, LAT_DELAY)
    # unconstrained, ki * dt * error * 200 frames would be ~1.8
    assert lac.pid.i < 0.05


class TestDirectionalSafetyFreeze:
  """steer_limited_by_safety fires on any command motion faster than the winddown slew (34%
  of active frames on the 2026-08-30 drive), so its integrator freeze must be directional:
  block integration that deepens |i|, keep decay toward a reversing error live (it was
  blocked on 12.7% of all frames). steeringPressed keeps the unconditional freeze."""

  def _primed(self, i_seed):
    v2 = make_lac(LatControlTorqueV2)
    for _ in range(DELAY_FRAMES + 10):
      step(v2, make_cs(15.0), 0.0, active=False)
    step(v2, make_cs(15.0), 0.0)
    v2.pid.i = i_seed
    return v2

  def _run_sls(self, v2, lat_accel, pressed=False, frames=20):
    for _ in range(frames):
      v2.update(True, make_cs(15.0, lat_accel, pressed=pressed), VM, LP, True, 0.0, None, False, LAT_DELAY)
    return v2.pid.i

  def test_sls_still_freezes_a_deepening_update(self, params):
    # error and integrator same-signed: integrating would deepen |i| -> frozen, as before
    i = self._run_sls(self._primed(0.1), lat_accel=-0.5)  # error = +0.5, i = +0.1
    assert i == pytest.approx(0.1)

  def test_sls_allows_integrator_decay(self, params):
    # error opposes the integrator: the update shrinks |i| and must not be blocked
    i = self._run_sls(self._primed(0.1), lat_accel=+0.5)  # error = -0.5, i = +0.1
    assert 0.0 < i < 0.1 - 1e-4

  def test_pressed_freeze_stays_bidirectional(self, params):
    # driver override owns the wheel: even an opposing error must not move the integrator
    i = self._run_sls(self._primed(0.1), lat_accel=+0.5, pressed=True)
    assert i == pytest.approx(0.1)


class TestSharedGainSchedule:
  """v2 keeps v0's gain schedule verbatim: the low-speed KP flatten (KP ~7 below 7.5 m/s,
  2026-08-30 routes 129-12c) tripled the felt vehicle weave at 2.5-5 m/s and was reverted.
  Any low-speed retune must come from a system-ID of the loop, not a schedule guess."""

  def test_kp_schedule_matches_v0_everywhere(self, params):
    v0, v2 = make_pair()
    for i in range(400):
      v0.pid.speed = v2.pid.speed = i * 0.1
      assert v2.pid.k_p == pytest.approx(v0.pid.k_p), f"{i * 0.1} m/s"


class TestLowSpeedDamping:
  """The 12e lateral maneuvers (9 m/s steps) showed 35-100% overshoot with the integrator
  frozen near zero: the EPS slew (12 counts/frame) leaves ~1 s of stale torque behind a
  railed command, and a pure P loop only unwinds after the crossing. kd = 0.3 s * KP(v),
  capped below 7.5 m/s and faded out by 14.5 m/s, is the phase lead validated against that
  route (closed-loop sim overshoot 1.63 -> 1.21; command off the rail a median 0.13 s
  earlier, model-free). v0 keeps KD = 0 and the shared -measurement_rate argument dead."""

  def test_kd_schedule_pins(self, params):
    _, v2 = make_pair()
    for v_ego, kd in [(2.0, 1.65), (7.5, 1.65), (9.0, 1.29), (10.0, 1.05), (14.5, 0.0), (25.0, 0.0)]:
      v2.pid.speed = v_ego
      assert v2.pid.k_d == pytest.approx(kd), f"{v_ego} m/s"

  def test_v0_kd_stays_zero(self, params):
    v0, _ = make_pair()
    for v_ego in [2.0, 9.0, 25.0]:
      v0.pid.speed = v_ego
      assert v0.pid.k_d == 0.0

  def test_damping_opposes_measurement_motion(self, params):
    """With the setpoint held and the measurement swinging toward it, the D term opposes the
    motion (positive rate -> negative d) and trims the command relative to a static run."""
    _, moving = make_pair()
    _, static = make_pair()
    v_ego = 9.0
    desired = 0.5 / v_ego ** 2
    for _ in range(DELAY_FRAMES + 10):
      step(moving, make_cs(v_ego), desired, active=False)
      step(static, make_cs(v_ego), desired, active=False)
    out_m = out_s = 0.0
    for i in range(50):
      lat = min(i * 0.02, 0.5)  # measurement sweeping up at 2 m/s^3
      log_m = step(moving, make_cs(v_ego, lat), desired)
      log_s = step(static, make_cs(v_ego, 0.0), desired)
      out_m, out_s = log_m.output, log_s.output
    assert moving.pid.d < -0.1
    assert static.pid.d == pytest.approx(0.0, abs=1e-9)
    # output is logged sign-inverted; the moving run commands less toward the turn
    assert abs(out_m) < abs(out_s)

  def test_no_damping_at_highway_speed(self, params):
    """Above the fade the D term is exactly zero however fast the measurement moves."""
    _, v2 = make_pair()
    v_ego = 20.0
    desired = 0.5 / v_ego ** 2
    for _ in range(DELAY_FRAMES + 10):
      step(v2, make_cs(v_ego), desired, active=False)
    for i in range(50):
      step(v2, make_cs(v_ego, min(i * 0.02, 0.5)), desired)
    assert v2.pid.d == 0.0


def make_plan_model(frame_id, curvature, v_plan=15.0):
  """Fake modelV2 carrying a curvature plan over T_IDXS (constant, or a callable of plan
  time). orientationRate.z = curvature * velocity, matching the on-wire convention."""
  ks = [curvature(t) if callable(curvature) else curvature for t in ModelConstants.T_IDXS]
  return SimpleNamespace(
    frameId=frame_id,
    orientation=SimpleNamespace(x=[0.0] * 33),
    orientationRate=SimpleNamespace(z=[k * v_plan for k in ks]),
    velocity=SimpleNamespace(x=[float(v_plan)] * 33),
  )


class TestPlanJerkSource:
  """The setpoint jerk comes from the model trajectory secant while the plan is coherent,
  and falls back to the request differencer (shipped v2 behavior) when it is not: no model,
  stale model, short arrays, curvature_limited, steeringPressed, or request/plan divergence.
  Every test above this class runs with no model attached and pins the fallback path."""

  def test_coherent_flat_plan_matches_v0(self, params):
    """A flat plan matching a flat request with the wheel on the request is the quiescent case
    on the plan path (w=1, plan jerk 0, zero tracking error): v2 must still be frame-for-frame
    identical to v0. (v2's low-speed error boost scales any nonzero error at every speed, so
    this identity only holds on the zero-error manifold.)"""
    v0, v2 = make_pair()
    v_ego, k = 15.0, 2e-3
    for i in range(DELAY_FRAMES + 10):
      step(v0, make_cs(v_ego, k * v_ego ** 2), k, active=False)
      v2.extension.update_model_v2(make_plan_model(i // 5, k, v_plan=v_ego))
      step(v2, make_cs(v_ego, k * v_ego ** 2), k, active=False)
    for i in range(300):
      if i % 5 == 0:
        v2.extension.update_model_v2(make_plan_model(1000 + i, k, v_plan=v_ego))
      out0, _, log0 = v0.update(True, make_cs(v_ego, k * v_ego ** 2), VM, LP, False, k, None, False, LAT_DELAY)
      out2, _, log2 = v2.update(True, make_cs(v_ego, k * v_ego ** 2), VM, LP, False, k, None, False, LAT_DELAY)
      assert out2 == pytest.approx(out0, abs=1e-9), f"frame {i}"
    assert v2.plan_jerk_weight == 1.0

  @pytest.mark.parametrize("turn_sign", [1.0, -1.0])
  def test_secant_equals_differencer_on_coherent_ramp(self, params, turn_sign):
    """The secant identity: with the request being the plan sampled at PLAN_REQUEST_T, the
    plan secant over [T - lat_delay, T] reproduces the differencer exactly for any plan
    shape — entry lead included. Both turn directions pin the sign convention."""
    v_ego = 15.0
    slope = turn_sign * 1.0 / v_ego ** 2  # 1 m/s^3 of lat accel per curvature-time second
    plan_request_t = max(LAT_DELAY + 0.075, 0.3)
    def world_k(tau):
      return slope * tau  # linear everywhere: T_IDXS interp is then exact
    lac_plan = make_lac(LatControlTorqueV2)
    lac_diff = make_lac(LatControlTorqueV2)
    for i in range(DELAY_FRAMES + 400):
      tau = i * DT
      request = world_k(tau + plan_request_t)
      lac_plan.extension.update_model_v2(make_plan_model(i, lambda t, tau=tau: world_k(tau + t), v_plan=v_ego))
      active = i >= DELAY_FRAMES
      log_p = step(lac_plan, make_cs(v_ego), request, active=active)
      log_d = step(lac_diff, make_cs(v_ego), request, active=active)
      if active:
        assert log_p.desiredLateralJerk == pytest.approx(log_d.desiredLateralJerk, abs=1e-6), f"frame {i}"
        assert log_p.desiredLateralAccel == pytest.approx(log_d.desiredLateralAccel, abs=1e-6), f"frame {i}"
        if turn_sign > 0 and world_k(tau) > 0:
          assert log_p.desiredLateralAccel > 0  # sign convention: setpoint follows the turn
    assert lac_plan.plan_jerk_weight == 1.0

  def test_handback_revision_does_not_snap(self, params):
    """The point of the plan path: at override release the request stream carries a plan
    revision (old dragged plan vs new re-centered plan), which the differencer reads as
    jerk and leads into — the hand-back snap. The plan secant never sees the revision, so
    the setpoint drains to the new request without overshooting past it."""
    v_ego = 15.0
    k_hold = 1.5 / v_ego ** 2  # dragged against the driver at 1.5 m/s^2
    results = {}
    for name, with_model in (('plan', True), ('diff', False)):
      lac = make_lac(LatControlTorqueV2)
      for i in range(DELAY_FRAMES + 100):  # pressed drag, plan and request coherent at k_hold
        if with_model:
          lac.extension.update_model_v2(make_plan_model(i, k_hold, v_plan=v_ego))
        step(lac, make_cs(v_ego, 1.5, pressed=True), k_hold, active=True)
        if with_model:
          assert lac.plan_jerk_weight == 0.0  # pressed frames belong to the differencer
      setpoints = []
      for i in range(100):  # release: the new plan and request re-center instantly
        if with_model:
          lac.extension.update_model_v2(make_plan_model(10_000 + i, 0.0, v_plan=v_ego))
        log = step(lac, make_cs(v_ego, 0.0), 0.0, active=True)
        setpoints.append(log.desiredLateralAccel)
      results[name] = min(setpoints)
    assert results['plan'] > -0.05, "plan path must not overshoot past the re-centered request"
    assert results['diff'] < -0.3, "differencer overshoot vanished; the scenario no longer exercises the snap"

  def test_divergence_gate_falls_back(self, params):
    """Request held while the plan collapses (turn-assist hold shape): the raw plan would
    unwind the held wheel, so the divergence blend must hand the jerk back to the
    differencer and keep the setpoint on the request."""
    v_ego = 15.0
    k_hold = 1.5 / v_ego ** 2
    lac = make_lac(LatControlTorqueV2)
    for i in range(DELAY_FRAMES + 50):
      lac.extension.update_model_v2(make_plan_model(i, k_hold, v_plan=v_ego))
      step(lac, make_cs(v_ego, 1.5), k_hold, active=True)
    log = None
    for i in range(100):  # plan collapses, request (assist hold) stays
      lac.extension.update_model_v2(make_plan_model(1000 + i, 0.0, v_plan=v_ego))
      log = step(lac, make_cs(v_ego, 1.5), k_hold, active=True)
      assert lac.plan_jerk_weight == 0.0, f"frame {i}"
      assert log.desiredLateralAccel > 1.4, f"frame {i}: the hold unwound"

  def test_stale_model_falls_back_without_a_step(self, params):
    """A hung modeld must not sustain a frozen plan slope: after MODEL_STALE_FRAMES the
    weight drops to zero, and on a coherent flat plan the setpoint does not step."""
    v_ego, k = 15.0, 2e-3
    lac = make_lac(LatControlTorqueV2)
    lac.extension.update_model_v2(make_plan_model(7, k, v_plan=v_ego))  # one frame, then silence
    for _ in range(DELAY_FRAMES + 10):  # prime the request buffer so both jerk sources read zero
      step(lac, make_cs(v_ego), k, active=False)
    prev = None
    weights = []
    for _ in range(DELAY_FRAMES + MODEL_STALE_FRAMES + 50):
      log = step(lac, make_cs(v_ego, k * v_ego ** 2), k, active=True)
      weights.append(lac.plan_jerk_weight)
      if prev is not None:
        assert abs(log.desiredLateralAccel - prev) < 1e-3
      prev = log.desiredLateralAccel
    assert weights[0] == 1.0
    assert weights[-1] == 0.0

  def test_short_arrays_fall_back(self, params):
    v_ego, k = 15.0, 2e-3
    lac = make_lac(LatControlTorqueV2)
    model = SimpleNamespace(frameId=1, orientation=SimpleNamespace(x=[0.0] * 33),
                            orientationRate=SimpleNamespace(z=[0.0] * 10), velocity=SimpleNamespace(x=[v_ego] * 10))
    lac.extension.update_model_v2(model)
    log = step(lac, make_cs(v_ego), k, active=True)
    assert lac.plan_jerk_weight == 0.0
    assert log.version == 2

  def test_lead_fades_to_v0_at_low_speed(self, params):
    """Below LEAD_SPEED_FADE_BP[0] the setpoint lead is fully faded and v2's setpoint is
    v0's (the live request), even through a step transient — any lead at parking speeds
    turns the low-speed KP schedule into rail-to-rail flapping."""
    v0, v2 = make_pair()
    v_ego = 4.0
    for _ in range(DELAY_FRAMES + 10):
      step(v0, make_cs(v_ego), 0.0, active=False)
      step(v2, make_cs(v_ego), 0.0, active=False)
    k_step = 1.0 / v_ego ** 2
    for i in range(100):
      out0, _, log0 = v0.update(True, make_cs(v_ego), VM, LP, False, k_step, None, False, LAT_DELAY)
      out2, _, log2 = v2.update(True, make_cs(v_ego), VM, LP, False, k_step, None, False, LAT_DELAY)
      assert log2.desiredLateralAccel == pytest.approx(log0.desiredLateralAccel, abs=1e-6), f"frame {i}"


class TestReleaseErrorRamp:
  """The release-edge error ramp: the P term used to land the whole hand-off error in one
  frame (P-step p90 2.24 within 100 ms measured on the 2026-08-29 override drive); the ramp
  eases the PID error in over RELEASE_ERROR_RAMP_T while feedforward stays immediate."""

  def _steady_error_run(self, v2, v_ego, lat_accel, press_frames):
    """Standing measurement offset; pressed for press_frames, then released. Returns
    (errors, ps, fs) for 40 frames after the release edge."""
    for _ in range(DELAY_FRAMES + 10):
      step(v2, make_cs(v_ego), 0.0, active=False)
    for _ in range(press_frames):
      step(v2, make_cs(v_ego, lat_accel, pressed=True), 0.0)
    errors, ps, fs = [], [], []
    for _ in range(40):
      log = step(v2, make_cs(v_ego, lat_accel), 0.0)
      errors.append(log.error)
      ps.append(log.p)
      fs.append(log.f)
    return errors, ps, fs

  def test_release_error_ramp_slopes_the_p_step(self, params):
    v2 = make_lac(LatControlTorqueV2)
    errors, ps, _ = self._steady_error_run(v2, v_ego=15.0, lat_accel=-0.5, press_frames=50)
    full = errors[-1]
    assert abs(full) > 0.4  # the standing error survives the run
    # first post-release frame carries ~dt/RAMP_T of the error, not all of it
    assert abs(errors[0]) < 0.1 * abs(full)
    assert abs(ps[0]) < 0.1 * abs(ps[-1])
    # monotone ramp-in, complete within RELEASE_ERROR_RAMP_T
    assert abs(errors[9]) < abs(errors[19]) < abs(errors[29])
    assert errors[35] == pytest.approx(full, rel=1e-6)

  def test_release_ramp_leaves_feedforward_alone(self, params):
    """The ramp eases the PID error only: feedforward (the curve hold the driver expects
    back immediately) must not dip at the release edge."""
    v2 = make_lac(LatControlTorqueV2)
    # nonzero desired so the FF is meaningful; measurement matches desired (no error)
    for _ in range(DELAY_FRAMES + 10):
      step(v2, make_cs(15.0), 2e-3, active=False)
    for _ in range(50):
      step(v2, make_cs(15.0, 2e-3 * 15.0 ** 2, pressed=True), 2e-3)
    logs = [step(v2, make_cs(15.0, 2e-3 * 15.0 ** 2), 2e-3) for _ in range(40)]
    assert logs[0].f == pytest.approx(logs[-1].f, abs=1e-9)

  def test_no_ramp_without_a_release_edge(self, params):
    """A steady active run never engages the ramp: error is full-scale from the start."""
    v2 = make_lac(LatControlTorqueV2)
    for _ in range(DELAY_FRAMES + 10):
      step(v2, make_cs(15.0), 0.0, active=False)
    log = step(v2, make_cs(15.0, -0.5), 0.0)
    assert abs(log.error) > 0.4

  def test_setpoint_lead_independent_of_the_rail(self, params):
    """The rail schedule must not touch the setpoint: the reverted lead taper fed last
    frame's output back into the setpoint through a 0.1-scale headroom window narrower
    than the output's own dither, and flapped (2026-08-30 routes 129-12c, corr(|d setpoint|,
    |d headroom|) 0.94). A railed and a scheduleless controller must form identical
    setpoints on the same railed demand."""
    v2a = make_lac(LatControlTorqueV2)
    v2a.steer_rail_schedule = ([0.0], [0.5])
    v2b = make_lac(LatControlTorqueV2)
    assert v2b.steer_rail_schedule is None
    v_ego = 15.0
    for _ in range(DELAY_FRAMES + 10):
      step(v2a, make_cs(v_ego), 0.0, active=False)
      step(v2b, make_cs(v_ego), 0.0, active=False)
    desired = 0.0
    for i in range(120):
      desired = min(desired + 2e-4, 1.5e-2)  # ramps toward 3.4 m/s^2, well past the 0.5 rail
      la = step(v2a, make_cs(v_ego), desired)
      lb = step(v2b, make_cs(v_ego), desired)
      assert la.desiredLateralAccel == pytest.approx(lb.desiredLateralAccel, abs=1e-9), f"frame {i}"


@pytest.mark.parametrize("pressed", [False, True])
def test_driver_override_damping_gate(params, pressed):
  lac = make_lac(LatControlTorqueV2)
  step(lac, make_cs(v_ego=10.0, lat_accel=0.0), 0.0, active=False)
  result = step(lac, make_cs(v_ego=10.0, lat_accel=0.3, pressed=pressed), 0.0)
  assert lac.measurement_rate_filter.x != 0.0
  if pressed:
    assert result.d == 0.0
  else:
    assert result.d != 0.0

  # Release resumes damping using continuously maintained measurement history.
  released = step(lac, make_cs(v_ego=10.0, lat_accel=0.6, pressed=False), 0.0)
  assert released.d != 0.0
