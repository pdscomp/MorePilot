"""Contract-B differential gate (rebase plan 2026-10-06 section B): Mazda actuator units.

TI Mazdas keep the OLD actuator units end to end — boot and retune tune values, speed-dep
seed selection, learned-bin consumption, manual-override and NNLC scales, companion 0x243
bytes and reported actuators, and integer-exact healthy 0x249 wire counts across the vEgo
sweep. Native (non-TI) Mazdas intentionally take the target's flat-1200 pipeline, pinned here
against target semantics with the OLD stepped values named as the expected difference. Cached
old-layout flags are quarantined before they can select a native profile.

OLD expected traces live in mazda_actuator_units_old_traces.json, generated from the
untouched zoom-more worktree (OLD = 40785dfd67f048f439100fbfd6608114760aca89) in an isolated
process; mazda_actuator_trace.py's docstring has the exact regeneration command. Both sides
run the same driver — real get_car with mocked CAN I/O, both setup layers, repeated CP-aware
retune calls, the real LatControlTorque v0 stack, and the real CarController — so a diff here
is a behavior change, never a harness reimplementation.
"""

import base64
import json
from pathlib import Path

import numpy as np
import pytest

from opendbc.car import structs
from opendbc.car.structs import car
from opendbc.car.mazda.values import CAR, CarControllerParams, MazdaFlags, TORQUE_TUNES
from opendbc.sunnypilot.car import interfaces as sp_interfaces
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.selfdrive.car.interfaces import seed_car_defaults_offroad
from openpilot.sunnypilot.system import params_migration
from openpilot.sunnypilot.selfdrive.controls.tests import mazda_actuator_trace as driver

OLD_TRACES = driver.load(Path(__file__).with_name('mazda_actuator_units_old_traces.json'))

TI_SCENARIOS = ('swapped_cx5_ti', 'swapped_cx5_ti_manual_nnlc', 'cx5_2022_ti', 'cx9_2021_legacy_ti', 'cx8_2022_ti', 'swapped_cx5_ti_learned')
# The raw params.toml Mazda row (via substitute.toml) both pins share: OLD TI tune, unscaled.
OLD_RAW_LAF = 1.7601682915983443
OLD_RAW_FRICTION = 0.17713792194297195
TUNE_SCALE = CarControllerParams.TUNE_SCALE  # target's native scale (1200/800); TI stays 1.0


@pytest.fixture(scope='module')
def traces():
  return driver.run_all()


def _diffs(cand, old, path=''):
  if isinstance(old, dict) and isinstance(cand, dict):
    for k in sorted(set(old) | set(cand), key=str):
      yield from _diffs(cand.get(k), old.get(k), f"{path}.{k}" if path else str(k))
  elif isinstance(old, list) and isinstance(cand, list) and len(old) == len(cand):
    for i, (c, o) in enumerate(zip(cand, old, strict=True)):
      yield from _diffs(c, o, f"{path}[{i}]")
  elif cand != old:
    yield path, old, cand


def assert_same(cand, old, label):
  """Exact recursive equality: integer wire counts, bytes, float32 read-backs. No tolerance."""
  diffs = list(_diffs(cand, old))
  assert not diffs, f"{label}: {len(diffs)} differing leaves:\n" + "\n".join(f"  {p}: OLD={o!r} candidate={c!r}" for p, o, c in diffs[:40])


def assert_ti_trace_equal(cand, old, keys, label):
  """TI scenario against the OLD trace. OLD predates the get_tune_scale helper: its absence
  means the unscaled 1.0 the fork always applied, which the candidate must still return."""
  assert cand['tune_scale'] == 1.0, f"{label}: TI tune scale must stay 1.0"
  assert old['tune_scale'] in (None, 1.0)
  assert_same({k: cand[k] for k in keys}, {k: old[k] for k in keys}, label)


def companion_steer_max(scenario, v):
  """The companion 0x243 scale at v, from the traced controller params."""
  params = OLD_TRACES[scenario]['controller_params']
  lookup = params['steer_max_lookup']
  if lookup is not None:
    return round(float(np.interp(v, lookup[0], lookup[1])))
  return params['steer_max']


class TestTiKeepsOldUnits:
  def test_ti_boot_and_second_tune_call_keep_old_units(self, traces):
    """Boot (get_car + opendbc TI setup), the parent's setup layer, and repeated CP-aware
    retunes must all leave the OLD raw tune in place — no native TUNE_SCALE on a TI car, no
    double-scaling on a second call."""
    for name in TI_SCENARIOS:
      cand, old = traces[name], OLD_TRACES[name]
      assert_ti_trace_equal(cand, old, ('flags', 'tunes', 'controller_params'), name)
      # four captures: after get_car, after parent setup, after retune 2 and 3
      assert len({t['laf'] for t in cand['tunes']}) == 1, f"{name}: retune rescaled the tune"
      assert len({t['friction'] for t in cand['tunes']}) == 1
      assert cand['flags']['ti'] and cand['flags']['min_steer_speed'] == 0.0
      assert cand['flags']['safety_ti']
      # the raw substituted Mazda row, float32-read-back, never the native scaled tune
      assert cand['tunes'][0]['laf'] == np.float32(OLD_RAW_LAF), name
      assert cand['tunes'][0]['friction'] == np.float32(OLD_RAW_FRICTION), name
      assert cand['tunes'][0]['laf'] != np.float32(np.float32(OLD_RAW_LAF) * TUNE_SCALE), name

  def test_ti_manual_and_nnlc_keep_old_wire_counts(self, traces):
    """Manual override and NNLC share the tune-scale helper; on TI both are multiplier 1.0,
    so the written override params and the NNLC feedforward keep OLD units, and the wire
    counts they produce are integer-identical."""
    name = 'swapped_cx5_ti_manual_nnlc'
    cand, old = traces[name], OLD_TRACES[name]
    assert_ti_trace_equal(cand, old, ('override_written', 'nnlc_scale', 'lac_driven', 'wire_scripted'), name)
    # the driver's explicit override values land unscaled on TI
    assert cand['override_written']['laf'] == np.float32(1.9)
    assert cand['override_written']['friction'] == np.float32(0.12)
    assert cand['nnlc_scale'] == 1.0

  def test_ti_seed_lookup_does_not_follow_native_substitution(self, traces):
    """TI seed lookups read the ported ti_legacy tables, never the target's native entries:
    the swapped CX-5 body keeps OLD's empty entry (no native donor substitute), the CX-5 2022
    keeps seed version 1 (not the native v2), the CX-8 2022 keeps its seven-bin table."""
    for name in ('swapped_cx5_ti', 'cx5_2022_ti', 'cx9_2021_legacy_ti', 'cx8_2022_ti'):
      assert_same(traces[name]['speed_dep'], OLD_TRACES[name]['speed_dep'], f"{name}.speed_dep")
    assert traces['swapped_cx5_ti']['speed_dep'] == {}
    assert OLD_TRACES['swapped_cx5_ti']['speed_dep'] == {}
    assert traces['cx5_2022_ti']['speed_dep']['seed_version'] == 1
    assert 'seed_version' not in traces['cx8_2022_ti']['speed_dep']  # OLD: omitted == 0
    # the native substitute exists (loader-expanded) and is taken only without TI
    assert traces['native_swapped_cx5']['speed_dep'].get('seed_version') == 2
    native_cfg = sp_interfaces.get_speed_dep_config()
    assert 'ti_legacy' not in native_cfg
    assert native_cfg['MAZDA_CX5'].get('seed_version') == 2  # the CX-5 2022 table, substituted in
    assert native_cfg['MAZDA_CX5'].get('requires_steer_to_zero') is True

  def test_ti_learned_bins_keep_old_counts(self, traces):
    """Learned (and seed-fallback) bins drive the same counts as OLD at every swept speed,
    including the stepped companion scale's 14.2/14.5 m/s cliff: bins are normalized units
    learned under one scale each, and OLD interpolated them in per-count space."""
    for name in ('cx5_2022_ti', 'swapped_cx5_ti_learned'):
      assert_same(traces[name]['learned'], OLD_TRACES[name]['learned'], f"{name}.learned")

  def test_ti_companion_reporting_keeps_old_meaning(self, traces):
    """Every scripted (vEgo, demand) point: 0x249 counts, companion 0x243 bytes, and the
    reported actuators are integer/byte/float-exact against OLD — and the report still means
    the native companion command (apply_torque over the companion scale), not TI feedback."""
    for name in ('swapped_cx5_ti', 'swapped_cx5_ti_manual_nnlc', 'cx5_2022_ti', 'cx9_2021_legacy_ti', 'cx8_2022_ti'):
      assert_same(traces[name]['wire_scripted'], OLD_TRACES[name]['wire_scripted'], f"{name}.wire_scripted")
      for (v, _d), r in traces[name]['wire_scripted'].items():
        assert r['ti'] is not None, f"{name} {v}: TI car must send 0x249"
        assert r['rep_counts'] == r['cam'], f"{name} {v}: torqueOutputCan is the 0x243 command"
        # actuators.torque is a capnp Float32: compare against the float32-rounded division
        assert r['rep_torque'] == np.float32(r['cam'] / companion_steer_max(name, v)), f"{name} {v}: reported torque is normalized on the companion scale"
    # standstill-zero boundary: the TI wire zeroes below 1.0 m/s, the companion never does
    swapped = traces['swapped_cx5_ti']['wire_scripted']
    assert swapped[(0.0, 0.95)]['ti'] == swapped[(0.99, 0.95)]['ti'] == 0
    assert swapped[(1.0, 0.95)]['ti'] == 570


class TestNativeUsesTargetUnits:
  """The intentional contract-B difference: native Mazdas follow the target's flat-1200
  pipeline. The OLD stepped-scale values are the named expected trace difference."""

  def test_native_mazda_uses_target_units(self, traces):
    platforms = {'native_cx5_2022': CAR.MAZDA_CX5_2022, 'native_swapped_cx5': CAR.MAZDA_CX5, 'native_legacy_cx5': CAR.MAZDA_CX5}
    for name in ('native_cx5_2022', 'native_swapped_cx5', 'native_legacy_cx5'):
      cand = traces[name]
      assert cand['tune_scale'] == TUNE_SCALE, f"{name}: native Mazda runs the target scale"
      assert not cand['flags']['ti']
      params = cand['controller_params']
      # target's flat measured envelope for every native EPS-hardware Mazda
      assert params['steer_max'] == 1200, name
      assert params['steer_max_lookup'] is None, name
      assert params['eps_ceiling'] is not None, name
      assert params['delta_up'] == params['delta_down'] == 12, name
      # the tunes[0] capture is get_params alone: base row converted once by TUNE_SCALE
      raw_laf, raw_fric = TORQUE_TUNES.get(platforms[name], (OLD_RAW_LAF, OLD_RAW_FRICTION))
      tune0 = cand['tunes'][0]
      assert tune0['laf'] == np.float32(np.float32(raw_laf) * TUNE_SCALE), name
      assert tune0['friction'] == np.float32(np.float32(raw_fric) / TUNE_SCALE), name
      assert all(t == tune0 for t in cand['tunes']), f"{name}: retune must be idempotent"

    # named wire difference: above the OLD 14.5 m/s step the native companion is on the flat
    # 1200 scale — OLD stepped to 800 there. Below the step nothing changed.
    for name in ('native_cx5_2022', 'native_swapped_cx5'):
      cand, old = traces[name]['wire_scripted'], OLD_TRACES[name]['wire_scripted']
      assert cand[(6.5, 0.95)]['cam'] == old[(6.5, 0.95)]['cam'] == 1140
      assert cand[(14.2, 0.95)]['cam'] == old[(14.2, 0.95)]['cam'] == 648  # ceiling clip, both
      assert cand[(14.5, 0.95)]['cam'] == old[(14.5, 0.95)]['cam'] == 620
      assert cand[(21.0, 0.3)]['cam'] == 360 and old[(21.0, 0.3)]['cam'] == 240  # the named change
      assert cand[(35.0, -0.3)]['cam'] == -360 and old[(35.0, -0.3)]['cam'] == -240
      assert all(r['ti'] is None for r in cand.values()), f"{name}: no TI wire without TI"

    # target native seed tables: v2 8-bin table replaces OLD's stepped-scale v1; the swapped
    # body natively follows the donor substitute (OLD gave it nothing); a legacy-floor car
    # gets nothing on either branch.
    assert traces['native_cx5_2022']['speed_dep']['seed_version'] == 2
    assert len(traces['native_cx5_2022']['speed_dep']['speed_bp']) == 8
    assert OLD_TRACES['native_cx5_2022']['speed_dep']['seed_version'] == 1
    assert traces['native_swapped_cx5']['speed_dep']['seed_version'] == 2
    assert OLD_TRACES['native_swapped_cx5']['speed_dep'] == {}
    assert traces['native_legacy_cx5']['speed_dep'] == OLD_TRACES['native_legacy_cx5']['speed_dep'] == {}


class TestCachedFlagQuarantine:
  def test_cached_ti_flags_cannot_select_a_native_profile(self):
    """A CarParamsPersistent written by an OLD branch carries TI as bit 4, which the new
    namespace reads as LEGACY_FW_EPS — a TI car would reboot as a native legacy-EPS car with
    scaled tunes. The migration must quarantine it (archive + remove), never reinterpret."""
    with OpenpilotPrefix():
      params = Params()
      old_cp = structs.CarParams.new_message()
      old_cp.brand = 'mazda'
      old_cp.carFingerprint = 'MAZDA_CX5'
      old_cp.flags = 1 | 2 | 4  # v1 GEN1 | STEER_TO_ZERO | TORQUE_INTERCEPTOR
      old_cp.lateralTuning.init('torque')
      old_cp.lateralTuning.torque.latAccelFactor = OLD_RAW_LAF
      old_bytes = old_cp.to_bytes()
      params.put('CarParamsPersistent', old_bytes, block=True)

      params_migration.migrate_mazda_control_cache(params)

      # quarantined: no persistent CP survives to offroad seeding or safety
      assert params.get('CarParamsPersistent') is None
      assert params.get('CarParamsSPPersistent') is None
      seed_car_defaults_offroad(params)
      assert params.get('MazdaTorqueTuneSeeded') is None  # nothing seeded from stale flags

      # the archived bytes really were the hazard: under the new namespace that CP reads as a
      # native legacy-EPS car with no TI — the profile the quarantine exists to prevent
      meta_dir = Path(params.get_param_path()).parent
      archive = json.loads((meta_dir / '.mazda_control_cache_archive.json').read_text())
      assert archive['generations'][-1]['keys']['CarParamsPersistent']['data'] == base64.b64encode(old_bytes).decode()
      resurrected = messaging.log_from_bytes(old_bytes, car.CarParams)
      assert resurrected.flags & MazdaFlags.LEGACY_FW_EPS
      assert not resurrected.flags & MazdaFlags.TORQUE_INTERCEPTOR
