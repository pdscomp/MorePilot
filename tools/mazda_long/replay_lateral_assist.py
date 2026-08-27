#!/usr/bin/env python3
"""Replay corpus episodes through the ported turn assist and lane-change smoothing.

Consumes the npz windows written by scan_lateral_events.py:

  st_*.npz  slow signaled turns  -> TurnAssistController, with the logged carState,
            model action, and 33-point plan. Reports what the hold/pre-wind/lead
            would have done, and flags any frame where the floored command exceeds
            the plan cap or opposes the blinker (must be zero).
  lc_*.npz  lane changes         -> LaneChangeSmoothing + clip_curvature in open
            loop, re-clipping the logged model action at several paces. Reports the
            applied-jerk reduction and how far the clamped command lags the model at
            the arrest (the overshoot proxy the pursuit must bound).

Open-loop caveat: the car in the log was driven without these features, so the model
never reacts to the assisted command. Direction-of-effect and bound checks are valid;
absolute magnitudes on-car will differ.

Usage:
  python tools/mazda_long/replay_lateral_assist.py '<events_dir>/*.npz' [--verbose]
"""
from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from openpilot.common.prefix import OpenpilotPrefix


def ffill_indices(t_cs, t_mdl):
  """Index of the latest mdl row at or before each cs timestamp."""
  idx = np.searchsorted(t_mdl, t_cs, side='right') - 1
  return np.clip(idx, 0, len(t_mdl) - 1)


def replay_turn(npz, TurnAssistController, log):
  cs, mdl, px, py = npz['cs'], npz['mdl'], npz['px'], npz['py']
  ta = TurnAssistController(NS(steerControlType='torque'))
  ta.get_params()
  assert ta.enabled
  midx = ffill_indices(cs[:, 0], mdl[:, 0])
  n_floored = n_lead = 0
  max_hold = 0.0
  viol_cap = viol_dir = 0
  for i, row in enumerate(cs):
    _, v, a, bl, br, _ang, pressed, torque, _dk, k_meas, active = row
    mi = midx[i]
    lc_state = log.LaneChangeState.laneChangeStarting if mdl[mi, 2] >= 2 else log.LaneChangeState.off
    model_v2 = NS(meta=NS(laneChangeState=lc_state),
                  position=NS(x=px[mi].tolist(), y=py[mi].tolist()))
    CS = NS(vEgo=v, aEgo=a, leftBlinker=bool(bl), rightBlinker=bool(br),
            steeringPressed=bool(pressed), steeringTorque=torque)
    lat_active = bool(active)
    cmd = mdl[mi, 1] if lat_active else k_meas
    out = ta.update(CS, lat_active, model_v2, cmd, k_meas)
    blinker_dir = float(br) - float(bl)
    if lat_active and abs(out) > abs(cmd) + 1e-6:
      if abs(ta.hold) > 1e-9:
        n_floored += 1
      if ta.lead_applied != 0.0:
        n_lead += 1
    max_hold = max(max_hold, abs(ta.hold))
    # plan/nudge sources are capped at 0.12; the model-command ratchet is bounded only
    # by what the model itself demanded, so the hard invariant is MAX_CURVATURE
    if abs(ta.hold) > 0.2001:
      viol_cap += 1
    if blinker_dir != 0.0 and ta.hold * blinker_dir < -1e-9:
      viol_dir += 1
  dur = cs[-1, 0] - cs[0, 0]
  return {'dur': dur, 'frames': len(cs), 'floored_s': n_floored * 0.01, 'lead_s': n_lead * 0.01,
          'max_hold': max_hold, 'viol_cap': viol_cap, 'viol_dir': viol_dir}


def replay_lane_change(npz, LaneChangeSmoothing, clip_curvature, log):
  cs, mdl = npz['cs'], npz['mdl']
  lcs = LaneChangeSmoothing()
  lcs.get_params()
  midx = ffill_indices(cs[:, 0], mdl[:, 0])
  state_map = {0: log.LaneChangeState.off, 1: log.LaneChangeState.preLaneChange,
               2: log.LaneChangeState.laneChangeStarting, 3: log.LaneChangeState.laneChangeFinishing}
  sim = cs[0, 8]  # start from the logged applied curvature
  sim_trace = []
  for i, row in enumerate(cs):
    _, v, _a, _bl, _br, _ang, _pressed, _torque, _dk, _k_meas, _active = row
    mi = midx[i]
    model_v2 = NS(meta=NS(laneChangeState=state_map[int(mdl[mi, 2])]))
    action = mdl[mi, 1]
    jf = lcs.update(NS(vEgo=v), model_v2, action, sim)
    sim, _ = clip_curvature(v, sim, action, 0.0, jf)
    sim_trace.append(sim)
  sim_trace = np.array(sim_trace)
  t, v = cs[:, 0], cs[:, 1]
  dt = np.diff(t)
  ok = dt > 1e-4
  in_lc = mdl[midx, 2] >= 2
  jerk = (np.abs(np.diff(sim_trace)[ok] / dt[ok]) * np.maximum(v[1:][ok], 1.0) ** 2)[in_lc[1:][ok]]
  action_ff = mdl[midx, 1]
  lag = np.abs(action_ff - sim_trace)[in_lc]
  return {'jerk_p99': float(np.percentile(jerk, 99)) if len(jerk) else 0.0,
          'max_lag': float(lag.max()) if len(lag) else 0.0,
          'end_lag': float(abs(action_ff[-1] - sim_trace[-1]))}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('patterns', nargs='+')
  ap.add_argument('--verbose', action='store_true')
  ap.add_argument('--paces', default='1,5,9')
  args = ap.parse_args()

  files = sorted(f for pat in args.patterns for f in glob.glob(pat))
  st_files = [f for f in files if Path(f).name.startswith('st_')]
  lc_files = [f for f in files if Path(f).name.startswith('lc_')]
  print(f"{len(st_files)} slow-turn episodes, {len(lc_files)} lane-change episodes")

  with OpenpilotPrefix('lat_assist_replay'):
    from openpilot.cereal import log
    from openpilot.common.params import Params
    from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
    from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_smoothing import LaneChangeSmoothing
    from openpilot.sunnypilot.selfdrive.controls.lib.turn_assist import TurnAssistController
    params = Params()
    params.put_bool('LowSpeedTurnAssist', True, block=True)
    params.put_bool('LaneChangeSmoothing', True, block=True)

    st_res = []
    skipped = 0
    for f in st_files:
      npz = np.load(f)
      if len(npz['mdl']) < 2 or len(npz['cs']) < 2:
        skipped += 1
        continue
      r = replay_turn(npz, TurnAssistController, log)
      st_res.append((Path(f).name, r))
      if args.verbose:
        print(f"ST {Path(f).name}: floored {r['floored_s']:.1f}s lead {r['lead_s']:.1f}s " +
              f"max_hold {r['max_hold']:.3f} viol cap/dir {r['viol_cap']}/{r['viol_dir']}")

    print(f"\n==== turn assist over real slow turns ==== ({skipped} skipped, no model rows)")
    if st_res:
      floored = np.array([r['floored_s'] for _, r in st_res])
      lead = np.array([r['lead_s'] for _, r in st_res])
      hold = np.array([r['max_hold'] for _, r in st_res])
      engaged = floored > 0.05
      print(f"episodes with a hold engaged: {engaged.sum()}/{len(st_res)}")
      print(f"floored seconds/episode: median {np.median(floored):.2f}  p90 {np.percentile(floored, 90):.2f}  max {floored.max():.2f}")
      print(f"lead seconds/episode:    median {np.median(lead):.2f}  p90 {np.percentile(lead, 90):.2f}  max {lead.max():.2f}")
      print(f"max hold magnitude:      median {np.median(hold):.3f}  max {hold.max():.3f} " +
            "(plan/nudge cap 0.120; model-sourced may exceed; hard bound 0.200)")
      viol_cap = sum(r['viol_cap'] for _, r in st_res)
      viol_dir = sum(r['viol_dir'] for _, r in st_res)
      print(f"cap violations {viol_cap}  direction violations {viol_dir}  (both must be 0)")

    lc_data = [npz for f in lc_files for npz in [np.load(f)] if len(npz['mdl']) >= 2 and len(npz['cs']) >= 2]
    paces = [int(p) for p in args.paces.split(',')]
    print("\n==== lane-change smoothing over real lane changes ====")
    for pace in paces:
      params.put("LaneChangeSmoothingPace", pace, block=True)
      res = [replay_lane_change(npz, LaneChangeSmoothing, clip_curvature, log) for npz in lc_data]
      if not res:
        continue
      jp = np.array([r['jerk_p99'] for r in res])
      lag = np.array([r['max_lag'] for r in res])
      endlag = np.array([r['end_lag'] for r in res])
      print(f"pace {pace}: applied jerk p99 median {np.median(jp):.2f} m/s^3 (was ~4.9 stock)  " +
            f"max in-change lag median {np.median(lag)*1e3:.2f} p90 {np.percentile(lag, 90)*1e3:.2f} e-3/m  " +
            f"end lag median {np.median(endlag)*1e3:.3f} e-3/m")


if __name__ == '__main__':
  main()
