"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_zp import LatControlTorqueExtZP


class LatControlTorqueExt(LatControlTorqueExtZP, NeuralNetworkLateralControl, LatControlTorqueExtOverride):
  def __init__(self, lac_torque, CP, CP_SP, CI):
    NeuralNetworkLateralControl.__init__(self, lac_torque, CP, CP_SP, CI)
    LatControlTorqueExtOverride.__init__(self, CP)
    LatControlTorqueExtZP.__init__(self, CP)

  def update(self, CS, VM, pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
             desired_lateral_accel, actual_lateral_accel, lateral_accel_deadzone, gravity_adjusted_lateral_accel,
             desired_curvature, actual_curvature, steer_limited_by_safety, output_torque):
    self._ff = ff
    self._pid = pid
    self._pid_log = pid_log
    self._setpoint = setpoint
    self._measurement = measurement
    self._roll_compensation = roll_compensation
    self._lateral_accel_deadzone = lateral_accel_deadzone
    self._desired_lateral_accel = desired_lateral_accel
    self._actual_lateral_accel = actual_lateral_accel
    self._desired_curvature = desired_curvature
    self._actual_curvature = actual_curvature
    self._gravity_adjusted_lateral_accel = gravity_adjusted_lateral_accel
    self._steer_limited_by_safety = steer_limited_by_safety
    self._output_torque = output_torque

    self.update_calculations(CS, VM, desired_lateral_accel)
    self.update_jerk_aware_torque_control(CS, roll_compensation, gravity_adjusted_lateral_accel)
    self.update_neural_network_feedforward(CS, params, calibrated_pose)

    return self._pid_log, self._output_torque

  def disable_speed_dep_torque(self):
    """The single speed-dep deactivation path. Restores the CP tune so the controller does not
    keep running on the last interpolated values, as upstream does when useParams is false."""
    if not self._speed_dep_active:
      return
    self._speed_dep_active = False
    tune = self.CP.lateralTuning.torque
    self.lac_torque.torque_params.latAccelFactor = tune.latAccelFactor
    self.lac_torque.torque_params.latAccelOffset = tune.latAccelOffset
    self.lac_torque.torque_params.friction = tune.friction
    self.lac_torque.update_limits()

  def update_speed_dep_torque(self, tp, tp_sp):
    """Apply torqued's per-bin values: learned values for valid bins, the car's TOML seeds or
    the global filtered values for the rest. tp is upstream's lateralTorqueParameters (the
    globals and useParams), tp_sp the fork's liveTorqueParametersSP published beside it
    (the bins), or None when that service has not checked out. useParams off, no fork
    message or no bins all mean torqued no longer stands behind the values (the manual
    override flips useParams mid-drive), and each deactivates through
    disable_speed_dep_torque rather than leaving stale tables."""
    if not tp.useParams or tp_sp is None or not tp_sp.speedBinCenters:
      self.disable_speed_dep_torque()
      return
    speed_bp = list(tp_sp.speedBinCenters)

    factors = list(tp_sp.speedBinLatAccelFactors)
    frictions = list(tp_sp.speedBinFrictions)
    valid_bp = list(tp_sp.speedBinValid)

    if self._speed_dep_car_cfg is None:
      from opendbc.sunnypilot.car.interfaces import get_speed_dep_config_for_car
      self._speed_dep_car_cfg = get_speed_dep_config_for_car(self.CP)
    cfg = self._speed_dep_car_cfg
    seed_lafs = cfg.get('laf_bp')
    seed_frictions = cfg.get('friction_bp')
    if (seed_lafs and seed_frictions and
        len(seed_lafs) == len(speed_bp) and len(seed_frictions) == len(speed_bp)):
      fallback_factors = seed_lafs
      fallback_frictions = seed_frictions
    else:
      global_factor = tp.latAccelFactorFiltered
      global_fric = tp.frictionCoefficientFiltered
      fallback_factors = [global_factor] * len(speed_bp)
      fallback_frictions = [global_fric] * len(speed_bp)

    self._speed_dep_active = True
    self._speed_dep_speed_bp = speed_bp
    self._speed_dep_lat_accel_factor_bp = [factors[i] if valid_bp[i] else fallback_factors[i] for i in range(len(speed_bp))]
    self._speed_dep_friction_bp = [frictions[i] if valid_bp[i] else fallback_frictions[i] for i in range(len(speed_bp))]

    # Per-count tables for platforms with a speed-dependent STEER_MAX (see the per-frame
    # interp in the override). Learned and seed values alike were measured under this car's
    # schedule, so one conversion covers both; rebuilt on every message as bin validity flips.
    schedule = cfg.get('steer_max_schedule')
    self._speed_dep_steer_max_schedule = schedule
    if schedule:
      sm_bp, sm_v = schedule
      steer_max_at_bins = [float(np.interp(c, sm_bp, sm_v)) for c in speed_bp]
      self._speed_dep_laf_per_count_bp = [laf / sm for laf, sm in zip(self._speed_dep_lat_accel_factor_bp, steer_max_at_bins, strict=True)]
      # friction is a normalized torque, so its counts are friction * STEER_MAX, the inverse of LAF's
      self._speed_dep_friction_per_count_bp = [fric * sm for fric, sm in zip(self._speed_dep_friction_bp, steer_max_at_bins, strict=True)]
    else:
      self._speed_dep_laf_per_count_bp = []
      self._speed_dep_friction_per_count_bp = []

    # global filtered values as the PID-limits baseline; the per-frame interp overwrites next frame
    self.lac_torque.torque_params.latAccelFactor = tp.latAccelFactorFiltered
    self.lac_torque.torque_params.latAccelOffset = tp.latAccelOffsetFiltered
    self.lac_torque.torque_params.friction = tp.frictionCoefficientFiltered
    self.lac_torque.update_limits()
