"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging

from opendbc.car import structs
from opendbc.sunnypilot.car.lateral_tune import get_steer_slew_schedule
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.locationd.speed_bin_learner import LIVE_TORQUE_PARAMETERS_SP_SERVICE
from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_smoothing import LaneChangeSmoothing, clip_curvature_rate
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v2 import LatControlTorque as LatControlTorqueV2
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v40 import LatControlTorque as LatControlTorqueV40
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v42 import LatControlTorque as LatControlTorqueV42
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v43 import LatControlTorque as LatControlTorqueV43
from openpilot.sunnypilot.selfdrive.controls.lib.steer_limit import classify
from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import (TI_TUNE_VERSIONS, resolved_mazda_v2_mode, resolved_tune_versions,
                                                                    stored_tune_versions, ti_tune_eligible)

# how long lateral control has to be inactive, continuously, before the torque tune follows
# a model change: a one-frame drop (a steer fault flicker) would otherwise swap and resume
# steering on a controller no inactive frame had primed
TUNE_SWAP_INACTIVE_FRAMES = round(0.5 / DT_CTRL)


# TI recommended-settings seeds. Unset params only — explicit user picks persist.
# Only keys that DIFFER from upstream defaults belong here; unset-only seeding a key to
# its stock default is a dead write. Stock already covers:
# LagdToggle=1, LateralJerkTorqueController=0, CustomTorqueParams=0, TorqueParamsOverrideEnabled=0.
TI_TUNE_SEEDS = {
  "LiveTorqueParamsToggle": True,       # "Self-Tune": defaults OFF upstream, we want it learning
  "LaneChangeSmoothing": 2,              # Medium pace; the key is an option index, not a bool
  "LowSpeedTurnAssist": True,           # TI cars steer at standstill, so low-speed assist works
  # NNLC deliberately unseeded: the UI treats EnforceTorqueControl+NNLC as mutually
  # exclusive and wipes BOTH when both are set, which would silently strand the car on
  # v0. v2 structurally ignores the NNLC overrides anyway. Toggling NNLC on manually
  # clears Enforce and gives the coherent v0+NNLC mode.
}
# No delay seed on either TI platform: the owner has preferred both modes at different
# times ("Live Delay" in Aug, fixed 0.05 in the current NNLC drives) — there is no proven
# universal winner, so the stock LagdToggle=1 default governs fresh installs and explicit
# user picks persist untouched.
TI_PLATFORMS = ("MAZDA_CX5_2022", "MAZDA_CX8_2022")


def seed_ti_defaults(params, CP) -> None:
  """TI cars get torque enforcement plus our recommended lateral defaults, seeded once.
  Unset params only — anything the user explicitly picked is left alone."""
  if not params.get_bool("TorqueInterceptorEnabled"):
    return
  if params.get("EnforceTorqueControl") is None:
    # enforcement off pins v0 via the resolver; the tune resolves to the declared 2.0
    # default once enforced. block=True: resolved_tune_version reads it right after, same call.
    params.put_bool("EnforceTorqueControl", True, block=True)
  if params.get("TorqueControlTune") is None:
    params.put("TorqueControlTune", 2.0, block=True)
  if CP.carFingerprint not in TI_PLATFORMS:
    return
  for key, value in TI_TUNE_SEEDS.items():
    if params.get(key) is None:
      params.put(key, value, block=True)


class ControlsLateralZP:
  """zoompilot's lateral additions to ControlsExt: a torque tune per model size, the steer-limit
  classifier, the speed-binned torque dispatch and lane-change smoothing."""

  def __init__(self, CP: structs.CarParams):
    self.lane_change_smoothing = LaneChangeSmoothing()

    # steer-limit classifier (lib/steer_limit.py): None on brands without carcontroller rate
    # limits and on angle-steered cars, where controlsd's flag is left untouched
    self._steer_slew_schedule = None
    if CP.steerControlType != structs.CarParams.SteerControlType.angle:
      self._steer_slew_schedule = get_steer_slew_schedule(CP)
    self._lat_active_last = False
    # frames in a row with CC.latActive false, as state_control decided it (note_lat_active);
    # the torque tune swap waits for TUNE_SWAP_INACTIVE_FRAMES of them
    self._inactive_frames = 0
    self._applied_torque_prev: float | None = None

  def initialize_lateral_control(self, lac, CI, dt):
    """One controller per model size, built once; every drive starts on the small model's."""
    # the enforce-off v0 forcing and the unset-param defaults both live in the resolver,
    # shared with the settings UIs so they gate on the tune that will actually run
    # TI cars default to tune v2, plus the torque-control enforcement the version
    # resolution hangs off of (without it every Mazda is pinned to v0 below).
    # Seeds fire only while the param is UNSET - an explicit user pick (including
    # enforce-off, or any tune version) is a set param and always persists.
    seed_ti_defaults(self.params, self.CP)

    versions = resolved_tune_versions(self.params, self.CP.lateralTuning.which() == 'torque', CP=self.CP)
    self._tune_versions = versions
    self._model_size = None
    self._active_tune_size = False
    cloudlog.info("controlsd: torque tune selection small=%s big=%s starting=small", versions[False], versions[True])
    for big, requested in stored_tune_versions(self.params).items():
      if requested in TI_TUNE_VERSIONS and versions[big] != requested:
        cloudlog.warning("controlsd: unsupported TI tune %.1f on this car; using v0", requested)

    def build(version):
      ti_tune = ti_tune_eligible(self.CP)
      if version == 40.0 and ti_tune:
        return LatControlTorqueV40(self.CP, self.CP_SP, CI, dt)
      if version == 42.0 and ti_tune:
        return LatControlTorqueV42(self.CP, self.CP_SP, CI, dt, mazda_v2_mode=resolved_mazda_v2_mode(self.params, self.CP))
      if version == 43.0 and ti_tune:
        return LatControlTorqueV43(self.CP, self.CP_SP, CI, dt)
      if version == 0.0:
        return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)
      if version == 2.0:
        return LatControlTorqueV2(self.CP, self.CP_SP, CI, dt)
      return lac

    built = {version: build(version) for version in dict.fromkeys(versions.values())}
    self._lac_by_size = {big: built[version] for big, version in versions.items()}
    self._lacs = tuple(built.values())
    return self._lac_by_size[False]

  def select_lateral_control(self, sm: messaging.SubMaster) -> None:
    """Runs at the end of every frame. modelV2.big says which model produced the frame; a
    change swaps self.LaC to the controller tuned for it, reset, but never while lateral
    control is active this frame. The idle controller holds whatever it had when it last
    steered (its request buffer, previous measurement, integrator): on the 2026-09-29 drives
    every big-to-small hand-back while steering stepped the commanded torque by 0.09 to 0.19
    of full scale in one tick. Since a hand-back no longer ends in a soft disable, the small
    model is carried by the tune that was steering until lateral has been inactive for
    TUNE_SWAP_INACTIVE_FRAMES in a row (a disengage, a blinker pause, a stop); a shorter drop
    keeps it. A swap to the big model only happens with nothing in control, and its second of
    no-entry is longer than the wait. From the next frame state_control runs the incoming
    controller, inactive until an engagement, which primes it as for any engagement, and
    pushes the live torque params, modelV2 and the lag into it. An engagement on the very
    frame after the swap would find it unprimed; priming it here would need the frame's
    CarState, which state_control has and this does not."""
    # The identity tracker only advances on a fresh, valid modelV2 message: the initial
    # default frame would otherwise log a phantom unknown->small and consume the first
    # real transition. Controller selection keeps using the last received value. Fakes
    # that only model the message default to fresh.
    fresh = bool(getattr(sm, 'updated', {}).get('modelV2', True)) and bool(getattr(sm, 'valid', {}).get('modelV2', True))
    big = bool(sm['modelV2'].big)
    if fresh and big != self._model_size:
      old = "unknown" if self._model_size is None else ("big" if self._model_size else "small")
      mono_ns = getattr(sm, 'logMonoTime', {}).get('modelV2', 0)
      cloudlog.info("controlsd: model size %s->%s modelMonoTime=%d", old, "big" if big else "small", mono_ns)
      self._model_size = big
    if len(self._lacs) == 1 or self._inactive_frames < TUNE_SWAP_INACTIVE_FRAMES:
      return
    lac = self._lac_by_size[big]
    if lac is self.LaC:
      return
    self.LaC = lac
    lac.reset()
    cloudlog.warning("controlsd: torque tune swap %s->%s for %s model; selectors small=%s big=%s",
                     self._tune_versions[self._active_tune_size], self._tune_versions[big], "big" if big else "small",
                     self._tune_versions[False], self._tune_versions[True])
    self._active_tune_size = big

  def get_lat_active(self, sm: messaging.SubMaster) -> bool:
    self._lat_active_last = self._get_lat_active(sm)
    return self._lat_active_last

  def reclassify_steer_limit(self, sm: messaging.SubMaster) -> None:
    """Runs after publish() has set this frame's steer_limited_by_safety from the raw torque
    mismatch. Replaces it with the classifier's directional, rail-aware flag (lib/steer_limit.py)
    before the next frame's LaC.update reads it. Torque tunes only; while lateral is inactive
    (get_lat_active's last value, the one publish() gated on) the flag is left as controlsd set it."""
    ext = getattr(self.LaC, 'extension', None)
    if ext is None or self._steer_slew_schedule is None:
      return
    if not self._lat_active_last:
      self._applied_torque_prev = None
      return
    v_ego = sm['carState'].vEgo
    applied = float(sm['carOutput'].actuatorsOutput.torque)
    bp, up, down = self._steer_slew_schedule
    rail_scale = ext.rail_scale_at(v_ego)
    limit = classify(ext.commanded_torque, applied, self._applied_torque_prev,
                     float(np.interp(v_ego, bp, up)), float(np.interp(v_ego, bp, down)),
                     rail_scale, self.steer_limited_by_safety, ext.last_error, ext.integrator)
    self.steer_limited_by_safety = limit.limited
    if hasattr(ext, 'set_actuator_state'):
      ext.set_actuator_state(applied, limit.at_rail)
    self._applied_torque_prev = applied

  def lane_change_curvature(self, sm: messaging.SubMaster, lat_active: bool, v_ego: float,
                            new_desired_curvature: float, prev_desired_curvature: float,
                            current_curvature: float | None = None) -> float:
    """new_desired_curvature held to lane-change smoothing's curvature rate, for upstream's
    clip_curvature to take (unchanged outside a smoothed lane change). The lateral maneuver
    mode's scripted commands pass through the stock clip. Called once a frame with
    CC.latActive, which select_lateral_control waits on."""
    self.note_lat_active(lat_active)
    if sm.valid['lateralManeuverPlan']:
      # clear the whole assist layer: state armed before maneuver mode (a held floor, a
      # lane-change arrest in progress) must not resume stale after it
      turn_assist = getattr(self, 'turn_assist', None)
      if turn_assist is not None:
        turn_assist.reset()
      self.lane_change_smoothing.reset()
      return new_desired_curvature
    # the low-speed turn assist shapes the request before the pace clamp, on cars that have it
    turn_assist = getattr(self, 'turn_assist', None)
    if turn_assist is not None and current_curvature is not None:
      new_desired_curvature = turn_assist.update(sm['carState'], lat_active, sm['modelV2'], new_desired_curvature, current_curvature)
    jerk_factor = self.lane_change_smoothing.update(sm['carState'], sm['modelV2'], lat_active, new_desired_curvature, prev_desired_curvature)
    return clip_curvature_rate(v_ego, prev_desired_curvature, new_desired_curvature, jerk_factor)

  def note_lat_active(self, lat_active: bool) -> None:
    self._inactive_frames = 0 if lat_active else self._inactive_frames + 1

  def dispatch_speed_bins(self, sm: messaging.SubMaster) -> None:
    """Speed-dependent torque: apply per-bin learned values to the lateral controllers."""
    if (self.CP.lateralTuning.which() == 'torque'
        and sm.updated.get('lateralTorqueParameters', False)
        and sm.all_checks(['lateralTorqueParameters'])):
      tp = sm['lateralTorqueParameters']
      # the learner publishes the bins beside every upstream message on the fork service;
      # one that has not checked out counts as no bins
      tp_sp = sm[LIVE_TORQUE_PARAMETERS_SP_SERVICE] if sm.all_checks([LIVE_TORQUE_PARAMETERS_SP_SERVICE]) else None
      # both sizes' controllers, so the idle one holds current bins when it takes over.
      # handles activation AND deactivation: useParams off or empty bins de-assert
      for lac in self._lacs:
        lac.extension.update_speed_dep_torque(tp, tp_sp)

  def run_lateral_zp(self, sm: messaging.SubMaster) -> None:
    """run_ext's tail: the steer-limit flag for the next frame, the speed bins, then the tune swap."""
    self.reclassify_steer_limit(sm)
    self.dispatch_speed_bins(sm)
    self.select_lateral_control(sm)
