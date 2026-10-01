"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import get_friction
from opendbc.car.mazda.values import MazdaFlags
from opendbc.sunnypilot.car.interfaces import get_steer_rail_schedule
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED, MIN_STABLE_DELAY
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_base import sign
from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import MazdaTorqueV2Mode

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import (
  LatControlTorque as LatControlTorqueV0,
  FRICTION_THRESHOLD,
  LP_FILTER_CUTOFF_HZ,
)

# v2 keeps v0's error correction in lateral acceleration space and its extension
# boundary (speed-dependent torque owns the feedforward params) unchanged. It
# reworks the setpoint path (jerk sourced from the model trajectory, not the
# request differencer — see the setpoint jerk block comment below), the friction
# input shaping, and the integrator policy around them.

VERSION = 2

# distinct from opendbc.car.lateral.MAX_LATERAL_JERK and drive_helpers.MAX_LATERAL_JERK,
# which are curvature rate limits — this one only clips the shaped setpoint-lead jerk (the
# low-speed fade's raw-differencer leg is bounded by clip_curvature's 5 m/s^3 instead)
MAX_SETPOINT_LATERAL_JERK = 2.5  # m/s^3
UNWIND_JERK_THRESHOLD = -1.0  # m/s^3, setpoint rate below this while near zero means the plan is unwinding
UNWIND_LAT_ACCEL_NEAR_ZERO = 0.3  # m/s^2
MIN_LATERAL_CONTROL_SPEED = 0.3  # m/s
STEER_RELEASE_I_DECAY = 0.8  # one-shot integrator decay on steering-press release
# At the release edge the tracking error is whatever the driver left (p90 0.67 m/s^2 on the
# 2026-08-29 override drive) and the P term lands all of it within one frame (P swing p90
# 2.24 within 100 ms) — the felt "abrupt at first" hand-back. Easing the error in over the
# ramp turns the step into a slope; feedforward is untouched, so the curve hold the driver
# expects on release is immediate.
RELEASE_ERROR_RAMP_T = 0.3  # s

# StarPilot's low-speed error boost (their LOW_SPEED_X/Y, verbatim): the PID error is
# scaled by 1 + lsf/kp, ~+45% at 5 m/s fading to ~+3% at 30, closing low-speed tracking
# error faster than the KP ladder alone. Normalizing by the scheduled KP keeps the added
# proportional authority roughly absolute across the ladder instead of compounding with
# it. Applied to the PID error only — the friction input keeps the unboosted error, so
# the stiction kick is unchanged (replay-validated orthogonal, 2026-08-27).
LOW_SPEED_X = [0, 10, 20, 30]  # m/s
LOW_SPEED_Y = [12, 10.5, 8, 5]

# TI cars pair this boost with the capped kp ladder (latcontrol_torque_v0's
# TI_LOW_SPEED_KP_CAP): effective low-speed gain is kp + (lsf/v)^2, and with kp=4 the
# quadratic term dominates — 0-10 kph still spent 17% of active time at full rail on the
# owner's capped-build route (r1b), ending corrections in a railed push then a friction
# grab (his "jerks violently, then snaps to a halt"). Halve the boost under 5 m/s for
# Mazda TI CPs, blending back to stock by 10 m/s (36 kph) — the band he calls nailed.
TI_LSF_SCALE_A_BP = [0.0, 5.0, 10.0]
TI_LSF_SCALE_A_V = [0.5, 0.5, 1.0]
TI_LSF_SCALE_B_BP = [0.0, 5.0, 7.5, 10.0]
TI_LSF_SCALE_B_V = [0.5, 0.5, 0.7, 1.0]


def ti_lsf_scale(v_ego, mode: MazdaTorqueV2Mode | None = MazdaTorqueV2Mode.A):
  bp, values = ((TI_LSF_SCALE_B_BP, TI_LSF_SCALE_B_V)
                if mode == MazdaTorqueV2Mode.B
                else (TI_LSF_SCALE_A_BP, TI_LSF_SCALE_A_V))
  return float(np.interp(max(v_ego, 0.0), bp, values))

# Roll compensation and latAccelOffset are lateral-accel-domain corrections; below
# walking pace the desired lateral accel is ~0, so an unfaded road-crown term dominates
# the whole feedforward and actively unwinds a held wheel at pull-away.
FF_ROLL_OFFSET_FADE_BP = [0.5, 2.5]  # m/s
FF_ROLL_OFFSET_FADE_V = [0.0, 1.0]

# Small planner jerk changes around the lane center can repeatedly re-trigger the
# friction compensation term. Keep this correction out of the center band while
# leaving actual turn-in and unwind commands unchanged.
CENTER_CHATTER_JERK_DEADZONE_SPEED_BP = [0.0, 5.0, 12.0, 25.0]  # m/s
CENTER_CHATTER_JERK_DEADZONE_SPEED_V = [0.08, 0.12, 0.18, 0.18]  # m/s^3
CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP = [0.0, 0.18, 0.35]  # m/s^2
CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_V = [1.0, 1.0, 0.0]

# Setpoint jerk source. The request differencer (future - expected) reads plan REVISIONS —
# override hand-back, model flicker — as planned jerk and leads into them; the 1.2 Hz filter
# then carries the spike ~0.13 s past the swing. That is the hand-back snap (route a9: worst
# release-window output delta vs v0 was 1.45, ~5x the route's other release windows).
# The model trajectory separates road from revision: the request is the plan sampled at
# modeld's lat_action_t = lat_delay + PLAN_ACTION_OFFSET (floored by MIN_STABLE_DELAY), and
# expected is the request from one lat_delay ago, so while the plan is coherent the secant
#   (k_plan(T) - k_plan(T - lat_delay)) / lat_delay
# equals the differencer (exactly for curvature-linear plans; within ~0.1 m/s^3 on real
# road features, sharp S-flicks excepted) — but a revision moves both samples
# together and cancels instead of reading as jerk. The differencer stays as the fallback,
# blended in by request-vs-plan divergence: turn assist, lane-change smoothing and
# clip_curvature shape the request without touching the trajectory, and the raw plan would
# fight them (a turn-assist hold floors the request while the plan collapses — pure plan
# jerk would unwind the held wheel). In those shaped regimes the request is rate-limited
# smooth by construction, so the differencer cannot snap there; revisions happen in the
# coherent regime, where the plan path is active. The curvature curve is scaled by the live
# vEgo^2 for the same reason the request buffer stores curvature: plan velocities embed the
# planned speed change and would read as phantom jerk under braking.
PLAN_ACTION_OFFSET = DT_MDL + DT_MDL / 2  # modeld's frame_delay + action_delay (modeld.py lat_action_t)
DIVERGENCE_BLEND_BP = [0.2, 0.5]  # m/s^2, |request - plan|; measured coherent-drive divergence is ~0.01-0.03
DIVERGENCE_BLEND_V = [1.0, 0.0]
MODEL_STALE_FRAMES = 25  # fresh modelV2 lands every ~5 frames; a hung modeld must not sustain a frozen slope

# The setpoint lead itself fades out below driving speed, collapsing to v0's algebra
# (setpoint = live request): the raw differencer times lat_delay reconstructs the request
# from the delayed one exactly, so blending the shaped jerk toward it is a continuous morph
# between the two setpoints. At parking speeds the low-speed KP schedule turns ANY lead —
# plan-sourced or differencer — into rail-to-rail flapping (route a9 t=356, 4 m/s: worst
# release-window delta 1.45 with the lead vs 0.28 without), the wheel is fast relative to
# the plan there, and every measured lead pathology sits below 8 m/s. The friction input
# keeps the shaped jerk at all speeds (it is deadzoned, not lead-scaled).
LEAD_SPEED_FADE_BP = [4.0, 8.0]  # m/s
LEAD_SPEED_FADE_V = [0.0, 1.0]

# Low-speed damping on the measurement rate. The EPS slews torque at 12 counts/frame, so a
# rail-to-rail traverse takes ~2 s: through a low-speed step the command rails, the applied
# torque walks far behind it, and by the time the measurement reaches the setpoint the EPS
# still carries most of a second of stale torque — the loop sails through, then swings back
# (route 12e lateral maneuvers at 9 m/s: 35-100% overshoot on every 0.5 m/s^2 step, i frozen
# near zero throughout, so not windup). The D term is the phase lead that unwinds the command
# before the crossing: kd = 0.3 s * KP(v), the 0.3 s covering the fitted plant delay (0.15 s)
# plus the measurement-rate filter's own lag at 1.2 Hz. Validated on the 12e steps two ways:
# closed-loop sim against the fitted 9 m/s plant (overshoot 1.63 -> 1.21, rise unchanged) and
# a model-free replay (command leaves the rail a median 0.13 s earlier). Capped below 7.5 m/s
# — the naive product tracks KP toward 250 where the measured rate is mostly noise — and
# faded to zero by 14.5 m/s: above the rail falloff the loop is well damped, and on 20+ m/s
# transients the product term would exceed P (measured on routes 12d/12f), reshaping a
# highway feel that is not broken. v0 keeps KD = 0.
KD_INTERP_SPEEDS = [7.5, 10.0, 12.0, 14.5]  # m/s
KD_INTERP = [1.65, 1.05, 0.85, 0.0]


def get_center_chatter_jerk_deadzone(v_ego, setpoint):
  """Small-signal jerk deadzone for the friction input (see the constants above)."""
  center_weight = np.interp(abs(setpoint), CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP,
                            CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_V)
  if center_weight == 0.0:  # in a real turn most of the time: skip the second interp
    return 0.0
  speed_deadzone = np.interp(max(v_ego, 0.0), CENTER_CHATTER_JERK_DEADZONE_SPEED_BP,
                             CENTER_CHATTER_JERK_DEADZONE_SPEED_V)
  return float(speed_deadzone * center_weight)


class LatControlTorque(LatControlTorqueV0):
  # v0's __init__ calls update_limits() before this subclass's __init__ has run (and once
  # before the extension exists), so everything the override reads needs a class-level
  # default rather than a per-call guard.
  _rail_limit_scale = 1.0
  extension = None
  # Built into the PID by v0's constructor; the -measurement_rate error_rate v0 already
  # feeds it stops being a dead argument here.
  KD_SCHEDULE = [KD_INTERP_SPEEDS, KD_INTERP]

  def __init__(self, CP, CP_SP, CI, dt, mazda_v2_mode=MazdaTorqueV2Mode.A):
    super().__init__(CP, CP_SP, CI, dt)
    # Stores CURVATURE, scaled by the current v^2 on read — buffered lateral accel keeps the
    # old speed's v^2 and reads as phantom tracking error whenever speed changes in the delay
    # window. Same length as v0's (unused here) buffer so the delay clamp stays valid.
    self.curvature_request_buffer = deque([0.] * self.lat_accel_request_buffer_len, maxlen=self.lat_accel_request_buffer_len)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
    self._ti_lsf_scaled = CP.steerAtStandstill and CP.brand == 'mazda'
    mazda_v2_ab_eligible = (
      CP.brand == "mazda" and CP.carFingerprint == "MAZDA_CX5" and CP.steerAtStandstill and
      CP.flags & MazdaFlags.TORQUE_INTERCEPTOR
    )
    self.mazda_v2_mode = None
    if mazda_v2_ab_eligible:
      self.mazda_v2_mode = MazdaTorqueV2Mode.B if mazda_v2_mode == MazdaTorqueV2Mode.B else MazdaTorqueV2Mode.A
    self.low_speed_pid_threshold = max(CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED)
    self.prev_steering_pressed = False
    self.prev_setpoint = 0.0
    self._release_error_ramp = 1.0
    # Fraction of the steer scale the EPS actually delivers, by speed (None: full scale).
    # Feeds both the saturation alert below and the PID's own limits (see update_limits);
    # _rail_limit_scale holds the current fraction, refreshed per frame in update().
    self.steer_rail_schedule = get_steer_rail_schedule(CP)
    # Planned-curvature cache for the setpoint jerk source (see the block comment above the
    # constants). Rebuilt only when a new modelV2 frame arrives (~20 Hz), not at 100 Hz.
    self._plan_curvature = None
    self._prev_model_frame_id = -1
    self._frames_since_model = MODEL_STALE_FRAMES
    self.plan_jerk_weight = 0.0  # last blend weight, read by the replay harness and tests

    # The extension's override controllers (jerk-aware, NNLC) recompute error/feedforward/
    # friction in torque space and step the shared PID themselves, silently replacing v2's
    # friction shaping and integrator policy while the setpoint changes still flow through.
    # Disable them regardless of the params; NNLC is already excluded structurally (enabling
    # it disables EnforceTorqueControl, which forces v0) — this covers the params disagreeing.
    cloudlog.info("LatControlTorque v2: extension output overrides (jerk-aware/NNLC) disabled")
    self.extension.disable_output_overrides()
    self.update_limits()  # an override controller may have retuned the shared PID to torque-space limits

  def update_limits(self):
    """Limit the PID to the torque the EPS will actually deliver at this speed, not to the full
    steer scale. The carcontroller already clamps the command to the measured ceiling, so this
    costs no delivered count; what it buys is that the PID's OWN anti-windup engages at the real
    rail. That one is directional -- it blocks the integrator growing into the limit while still
    letting it decay back out -- whereas the only thing reaching a railed command today is
    controlsd's steer_limited_by_safety, which freezes in both directions at once. Measured: it
    holds 62-71% of low-speed frames, pinning the integrator through whole corners against a
    standing 0.4 m/s^2 error it is never allowed to absorb or unwind.

    Platforms with no rail schedule keep the full scale, so this is a no-op for them.
    """
    scale = self._rail_limit_scale * self.steer_max
    self.pid.set_limits(self.lateral_accel_from_torque(scale, self.torque_params),
                        self.lateral_accel_from_torque(-scale, self.torque_params))
    # torque-space extension controllers need +-steer_max instead; re-assert on every reset
    # path (None until v0's __init__ constructs it, see the class-level default)
    if self.extension is not None:
      self.extension.update_limits()

  def _integrator_deepened_while_limited(self, steer_limited_by_safety, error):
    """steer_limited_by_safety means the applied torque differs from the request by more than
    0.01 -- which, with the winddown slew matched to the EPS (12/frame), fires on ANY command
    motion faster than 1200 counts/s: 34% of active frames on the 2026-08-30 drive, not just
    genuine clamping. Freezing the integrator outright on it blocked updates that would have
    SHRUNK the integrator on 12.7% of all frames -- a standing stale-integrator bias. Freeze
    only integration that would deepen |i| (error and integrator same-signed); decay toward a
    reversing error stays live, mirroring the directional anti-windup the PID itself runs at
    the rail limits. steeringPressed keeps its unconditional freeze -- there the driver owns
    the wheel and the error is theirs, not the plant's."""
    return steer_limited_by_safety and error * self.pid.i >= 0.0

  def _update_plan_curvature(self):
    """Refresh the planned-curvature curve from the extension's modelV2 (populated by
    controlsd every frame) when a new model frame arrives, and track staleness. The
    ext-base model_valid keys on orientation.x and belongs to the override controllers,
    so the fields this controller actually reads are length-checked here instead."""
    model = self.extension.model_v2
    if model is None:
      self._plan_curvature = None
      return
    if model.frameId == self._prev_model_frame_id:
      self._frames_since_model += 1
      return
    self._prev_model_frame_id = model.frameId
    self._frames_since_model = 0
    rate = np.asarray(model.orientationRate.z)
    vel = np.asarray(model.velocity.x)
    if len(rate) == len(ModelConstants.T_IDXS) and len(vel) == len(rate):
      self._plan_curvature = rate / np.maximum(vel, MIN_SPEED)
    else:
      self._plan_curvature = None

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # The deliverable rail moves with speed, so the PID limits have to be refreshed here:
    # update_limits is otherwise only reached when the torque params themselves change.
    rail_scale = 1.0
    if self.steer_rail_schedule is not None:
      rail_scale = float(np.interp(CS.vEgo, self.steer_rail_schedule[0], self.steer_rail_schedule[1]))
    rail_changed = rail_scale != self._rail_limit_scale
    self._rail_limit_scale = rail_scale

    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params) or rail_changed:
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION

    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2

    if not active:
      output_torque = 0.0
      pid_log.active = False
      # Keep the request buffer and rate state primed with the live command instead of
      # letting them go stale. Re-engaging with a wound wheel against a stale buffer puts
      # the setpoint ~lat_delay behind the measurement, and the low-speed gains turn that
      # lag into an unwind shove. The integrator is deliberately NOT cleared here: MADS
      # cycles lateral often, and the release decay + unwind freeze below replace a blunt
      # reset.
      self.curvature_request_buffer.append(desired_curvature)
      self.previous_measurement = measurement
      self.measurement_rate_filter.x = 0.0
      self.jerk_filter.x = 0.0
      self.prev_setpoint = future_desired_lateral_accel
      self.plan_jerk_weight = 0.0
      self._release_error_ramp = 1.0
    else:
      # One-shot integrator decay on steering-press release, so hand-back after an
      # override is neutral instead of a kick
      if self.prev_steering_pressed and not CS.steeringPressed:
        self.pid.i *= STEER_RELEASE_I_DECAY
        self._release_error_ramp = 0.0
      self._release_error_ramp = min(1.0, self._release_error_ramp + self.dt / RELEASE_ERROR_RAMP_T)

      roll_offset_fade = float(np.interp(CS.vEgo, FF_ROLL_OFFSET_FADE_BP, FF_ROLL_OFFSET_FADE_V))
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY * roll_offset_fade
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.lat_accel_request_buffer_len))
      expected_lateral_accel = self.curvature_request_buffer[-delay_frames] * CS.vEgo ** 2
      self.curvature_request_buffer.append(desired_curvature)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation

      differencer_jerk = (future_desired_lateral_accel - expected_lateral_accel) / max(lat_delay, self.dt)
      self._update_plan_curvature()
      plan_weight = 0.0
      plan_jerk = 0.0
      # While the driver is pressing, every 20 Hz plan predicts the maneuver ending (peak
      # now, unwind ahead) and the next plan revises it away — the secant then leads
      # opposite to what is actually being commanded (route ab t=407: plan railed +1.0
      # against v0's -1.0 mid-override). The differencer tracks the commanded path, so it
      # owns pressed frames; the plan takes over at the release edge, which is exactly
      # where the differencer's revision spike lives.
      if (self._plan_curvature is not None and self._frames_since_model < MODEL_STALE_FRAMES and
          not curvature_limited and not CS.steeringPressed):
        plan_request_t = max(lat_delay + PLAN_ACTION_OFFSET, MIN_STABLE_DELAY)
        # the secant start is plan_request_t - lat_delay >= PLAN_ACTION_OFFSET, never negative
        plan_accel_request = float(np.interp(plan_request_t, ModelConstants.T_IDXS, self._plan_curvature)) * CS.vEgo ** 2
        plan_accel_expected = float(np.interp(plan_request_t - lat_delay, ModelConstants.T_IDXS, self._plan_curvature)) * CS.vEgo ** 2
        plan_jerk = (plan_accel_request - plan_accel_expected) / max(lat_delay, self.dt)
        plan_weight = float(np.interp(abs(future_desired_lateral_accel - plan_accel_request), DIVERGENCE_BLEND_BP, DIVERGENCE_BLEND_V))
      self.plan_jerk_weight = plan_weight
      # Both blends below are the same idiom: v0's differencer jerk plus a weighted delta
      # toward the shaped source. The raw differencer times lat_delay reconstructs the live
      # request from the delayed one exactly, so at zero weight each stage IS v0's setpoint
      # algebra — the shaped path is strictly an additive lead on top of it.
      raw_lateral_jerk = differencer_jerk + plan_weight * (plan_jerk - differencer_jerk)
      raw_lateral_jerk = min(max(raw_lateral_jerk, -MAX_SETPOINT_LATERAL_JERK), MAX_SETPOINT_LATERAL_JERK)
      # the filter is a convex combination of clipped inputs, so its output needs no second clip
      shaped_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)

      # first-order lead: delayed request plus one lat_delay of planned jerk, so turn-in
      # starts a steering delay early instead of chasing the request. The clip and low-pass
      # above keep a lagd mis-estimate from over-leading the setpoint. With the plan holding
      # still the shaped jerk equals the differencer and this collapses back to v0's setpoint;
      # below driving speed the lead fades out entirely (see LEAD_SPEED_FADE_BP).
      lead_weight = float(np.interp(CS.vEgo, LEAD_SPEED_FADE_BP, LEAD_SPEED_FADE_V))
      setpoint_jerk = differencer_jerk + lead_weight * (shaped_lateral_jerk - differencer_jerk)
      setpoint = expected_lateral_accel + setpoint_jerk * lat_delay

      measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
      self.previous_measurement = measurement

      # Freeze the integrator while the plan is unwinding through center: integrating the
      # transient error there is what sticks the wheel past straight. Unwind means the
      # setpoint magnitude is COLLAPSING toward zero, so the rate is measured relative to
      # the side being exited (prev_setpoint's sign) — a bare `rate < threshold` only
      # catches right-turn exits and wrongly freezes on every left-turn entry.
      setpoint_rate = (setpoint - self.prev_setpoint) / self.dt
      unwind_rate = setpoint_rate * sign(self.prev_setpoint)
      unwind_detected = unwind_rate < UNWIND_JERK_THRESHOLD and abs(setpoint) < UNWIND_LAT_ACCEL_NEAR_ZERO
      self.prev_setpoint = setpoint

      error = setpoint - measurement

      # low-speed error boost (see the constants above); interp the schedule directly
      # rather than reading pid.k_p, which still holds the previous frame's speed here
      low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) / max(CS.vEgo, MIN_SPEED)) ** 2
      if self._ti_lsf_scaled:
        low_speed_factor *= ti_lsf_scale(CS.vEgo, self.mazda_v2_mode)
      current_kp = np.interp(CS.vEgo, self.pid._k_p[0], self.pid._k_p[1])
      error *= 1.0 + low_speed_factor / max(current_kp, 1e-3)

      # upstream release ramp: scales error down while the plan is unwinding from the rail
      error *= self._release_error_ramp

      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)
      ff = gravity_adjusted_future_lateral_accel
      # latAccelOffset corrects roll compensation bias from device roll misalignment relative
      # to car roll; it fades with the roll compensation it corrects
      ff -= self.torque_params.latAccelOffset * roll_offset_fade

      # The friction term sees the deadzoned jerk in place of the raw jerk contribution, so
      # tiny planner wobble at lane center cannot sign-flip it; the PID error above is unchanged
      friction_jerk_deadzone = get_center_chatter_jerk_deadzone(CS.vEgo, setpoint)
      friction_jerk = math.copysign(max(abs(shaped_lateral_jerk) - friction_jerk_deadzone, 0.0), shaped_lateral_jerk)
      friction_error = expected_lateral_accel + friction_jerk * lat_delay - measurement
      ff += get_friction(friction_error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      # Keyed to max(minSteerSpeed, 0.3) instead of v0's vEgo < 5: the integrator keeps
      # working at creep speeds, where controlsd only allows lateral on steer-at-standstill
      # cars anyway
      if CS.vEgo < self.low_speed_pid_threshold:
        self.pid.reset()
      freeze_integrator = (self._integrator_deepened_while_limited(steer_limited_by_safety, error) or
                           CS.steeringPressed or
                           CS.vEgo < self.low_speed_pid_threshold or unwind_detected)
      if self.extension.overrides_output:
        # Unreachable while __init__ disables the output overrides; kept as the guard
        # against a future override controller double-integrating the shared PID.
        output_torque = 0.0
      else:
        output_lataccel = self.pid.update(pid_log.error,
                                         -measurement_rate,
                                          feedforward=ff,
                                          speed=CS.vEgo,
                                          freeze_integrator=freeze_integrator)
        output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # Lateral acceleration torque controller extension updates
      # Overrides pid_log.error and output_torque. Keyword-bound: the signature is long and
      # shared across controllers, and a positional call fails silently if a sync reorders it.
      pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log,
                                                     setpoint=setpoint,
                                                     measurement=measurement,
                                                     calibrated_pose=calibrated_pose,
                                                     roll_compensation=roll_compensation,
                                                     desired_lateral_accel=future_desired_lateral_accel,
                                                     actual_lateral_accel=measurement,
                                                     lateral_accel_deadzone=lateral_accel_deadzone,
                                                     gravity_adjusted_lateral_accel=gravity_adjusted_future_lateral_accel,
                                                     desired_curvature=desired_curvature,
                                                     actual_curvature=measured_curvature,
                                                     steer_limited_by_safety=steer_limited_by_safety,
                                                     output_torque=output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(setpoint_jerk)
      # The EPS delivers only rail(v) of the steer scale above the ceiling falloff, and the
      # carcontroller's honest clamp reports reaching it as steer_limited_by_safety — which
      # _check_saturation treats as not-saturated (its suppression is meant for driver-torque
      # narrowing). Compare against the measured rail and drop the suppression while on it,
      # so a railed EPS in a curve still raises the steering-limit warning. Driver-torque
      # narrowing keeps its suppression: it shrinks the window below the rail, it does not
      # push the command onto it. Platforms without a rail schedule keep stock semantics
      # (rail_scale is 1.0 there). The PID limits sit on this same rail (see update_limits),
      # so at_rail is exactly "the PID is at its own limit".
      at_rail = rail_scale * self.steer_max - abs(output_torque) < 1e-3
      alert_limited = steer_limited_by_safety and (self.steer_rail_schedule is None or not at_rail)
      pid_log.saturated = bool(self._check_saturation(at_rail, CS, alert_limited, curvature_limited))

    self.prev_steering_pressed = CS.steeringPressed

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
