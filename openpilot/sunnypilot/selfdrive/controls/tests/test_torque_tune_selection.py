"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Which torque controller an unset TorqueControlTune selects. This is easy to get wrong by
# dropping `return_default=True` from the params read: params_keys.h declares a default, but
# a bare params.get() returns None for an unset param, and float(None) raises, or, guarded,
# can otherwise fall through to the upstream controller without an explicit error.
# The small model's declared default is upstream's 0.0; the steer-to-zero Mazdas are seeded to
# 2.0 by seed_mazda_torque_defaults instead, so other brands never inherit a tune fitted to
# that EPS. A big model's is v1 for every brand.
#
# The v0 constructor is patched out: these tests pin the branch that gets taken, not the
# controller's behavior, and building the real one pulls in NNLC model loading.

import re
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from opendbc.car.structs import car
from opendbc.car.mazda.values import MazdaFlags
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.selfdrive.controls import controls_lateral_zp
from openpilot.sunnypilot.selfdrive.controls.controls_lateral_zp import TUNE_SWAP_INACTIVE_FRAMES, ControlsLateralZP

class FakeLaC(str):
  """A controller stand-in that compares as its version label and counts resets."""
  resets = 0

  def reset(self):
    self.resets += 1


V0 = "v0"
V1 = "v1"  # stands in for the `lac` upstream controller controlsd passes in
V2 = "v2"
V40 = "v40"
V42 = "v42"
V43 = "v43"
BY_VERSION = {0.0: V0, 1.0: V1, 2.0: V2, 40.0: V40, 42.0: V42, 43.0: V43}
PARAMS_KEYS_H = Path(__file__).resolve().parents[5] / "openpilot" / "common" / "params_keys.h"


def declared_default(key: str = "TorqueControlTune") -> float:
  """The default params_keys.h declares, read from the header text so the test does not
  depend on the compiled params library being current."""
  m = re.search(r'\{"' + key + r'", \{[^}]*, "([0-9.]+)"\}\}', PARAMS_KEYS_H.read_text())
  assert m, f"{key} must declare a default in params_keys.h"
  return float(m.group(1))


def require_current_libparams(params, *keys: str) -> None:
  for key in keys:
    if float(params.get(key, return_default=True)) != declared_default(key):
      pytest.skip("libparams_c is stale against params_keys.h; rebuild with scons openpilot/common")


@pytest.fixture
def ctx(monkeypatch):
  monkeypatch.setattr(controls_lateral_zp, "LatControlTorqueV0", lambda *a, **k: FakeLaC(V0))
  monkeypatch.setattr(controls_lateral_zp, "LatControlTorqueV2", lambda *a, **k: FakeLaC(V2))
  monkeypatch.setattr(controls_lateral_zp, "LatControlTorqueV40", lambda *a, **k: FakeLaC(V40))
  monkeypatch.setattr(controls_lateral_zp, "LatControlTorqueV42", lambda *a, **k: FakeLaC(V42))
  monkeypatch.setattr(controls_lateral_zp, "LatControlTorqueV43", lambda *a, **k: FakeLaC(V43))
  with OpenpilotPrefix():
    params = Params()
    params.put_bool("EnforceTorqueControl", True, block=True)  # the enforce-off tests flip it
    CP = car.CarParams.new_message(steerControlType="torque", brand="mazda")
    CP.flags = int(MazdaFlags.TORQUE_INTERCEPTOR)
    CP.lateralTuning.init('torque')
    controls = SimpleNamespace(params=params, CP=CP.as_reader(),
                               CP_SP=custom.CarParamsSP.new_message().as_reader(),
                               _inactive_frames=TUNE_SWAP_INACTIVE_FRAMES)
    yield params, controls


def select(controls):
  """What controlsd does at startup: build and install the small model's controller."""
  controls.LaC = ControlsLateralZP.initialize_lateral_control(controls, FakeLaC(V1), MagicMock(), 0.01)
  return controls.LaC


def swap(controls, big: bool):
  ControlsLateralZP.select_lateral_control(controls, {'modelV2': SimpleNamespace(big=big)})


class TestTorqueTuneSelection:
  def test_declared_defaults(self):
    """Upstream's v0 for the small model, so other brands never inherit the Mazda tune through
    the default (the Mazdas are seeded, test_torque_defaults_seed.py); v1 for a big one."""
    assert declared_default("TorqueControlTune") == 0.0
    assert declared_default("TorqueControlTuneBig") == 1.0

  def test_unset_selects_the_declared_default(self, ctx):
    """An unset param must resolve through params_keys.h (v0 today), not through None."""
    params, controls = ctx
    params.remove("TorqueControlTune")
    require_current_libparams(params, "TorqueControlTune")
    assert select(controls) == BY_VERSION[declared_default()]

  @pytest.mark.parametrize(("version", "expected"), [(0.0, V0), (1.0, V1), (2.0, V2), (40.0, V40), (42.0, V42), (43.0, V43)])
  def test_explicit_version_is_honored(self, ctx, version, expected):
    params, controls = ctx
    params.put("TorqueControlTune", version, block=True)
    assert select(controls) == expected

  @pytest.mark.parametrize("version", [40.0, 42.0, 43.0])
  def test_ti_version_requires_final_cp_eligibility(self, ctx, version):
    params, controls = ctx
    CP = car.CarParams.new_message(steerControlType="torque")
    CP.lateralTuning.init("torque")
    controls.CP = CP.as_reader()
    params.put("TorqueControlTune", version, block=True)
    assert select(controls) == V0

  def test_missing_cp_ti_selection_falls_back_without_param_rewrite(self, ctx):
    from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import resolved_tune_versions
    params, _ = ctx
    params.put("TorqueControlTune", 40.0, block=True)
    params.put("TorqueControlTuneBig", 42.0, block=True)
    assert resolved_tune_versions(params, CP=None) == {False: 0.0, True: 0.0}
    assert params.get("TorqueControlTune") == 40.0
    assert params.get("TorqueControlTuneBig") == 42.0

  def test_every_declared_version_is_wired(self, ctx):
    """The versions file is what the UI selectors and the sunnylink schema offer, while
    initialize_lateral_control decides what is constructible. A version added to the file
    but not wired here would surface in every selector and silently run v1."""
    from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import load_versions

    wired = BY_VERSION
    declared = {float(info["version"]) for info in load_versions().values()}
    assert declared == set(wired), "declared tune versions must match the wired controllers"

    params, controls = ctx
    for version, expected in wired.items():
      params.put("TorqueControlTune", version, block=True)
      assert select(controls) == expected

  @pytest.mark.parametrize("version", [1.0, 2.0])
  def test_torque_control_not_enforced_still_uses_v0_for_torque_cars(self, ctx, version):
    """Pre-existing behavior worth pinning: torque-tuned cars get v0 even with the toggle off.
    For 2.0 this is also the structural NNLC exclusion: enabling NNLC disables
    EnforceTorqueControl (ui_state/_cleanup_unsupported_params), so a stored v2 selection can
    never construct the v2 controller alongside NNLC."""
    params, controls = ctx
    params.put_bool("EnforceTorqueControl", False, block=True)
    params.put("TorqueControlTune", version, block=True)
    assert select(controls) == V0

  def test_unset_big_tune_selects_its_declared_default(self, ctx):
    """An unset TorqueControlTuneBig resolves through params_keys.h (v1), not through the
    small tune and not through None."""
    params, controls = ctx
    params.put("TorqueControlTune", 2.0, block=True)
    params.remove("TorqueControlTuneBig")
    require_current_libparams(params, "TorqueControlTuneBig")
    select(controls)
    swap(controls, big=True)
    assert controls.LaC == BY_VERSION[declared_default("TorqueControlTuneBig")]

  def test_same_tune_for_both_sizes_never_swaps(self, ctx):
    params, controls = ctx
    params.put("TorqueControlTune", 2.0, block=True)
    params.put("TorqueControlTuneBig", 2.0, block=True)
    select(controls)
    swap(controls, big=True)
    assert controls.LaC == V2 and controls.LaC.resets == 0

  @pytest.mark.parametrize(("small", "big"), [(2.0, 1.0), (0.0, 2.0), (1.0, 0.0)])
  def test_lateral_control_follows_the_model_size(self, ctx, small, big):
    """Every drive starts on the small model's tune; the frame after modelV2.big flips runs
    the other size's controller, reset once, and a repeated frame does not churn it."""
    params, controls = ctx
    params.put("TorqueControlTune", small, block=True)
    params.put("TorqueControlTuneBig", big, block=True)
    assert select(controls) == BY_VERSION[small]

    swap(controls, big=True)
    assert controls.LaC == BY_VERSION[big] and controls.LaC.resets == 1
    swap(controls, big=True)
    assert controls.LaC.resets == 1

    swap(controls, big=False)
    assert controls.LaC == BY_VERSION[small] and controls.LaC.resets == 1

  def test_a_hand_back_while_steering_keeps_the_tune_that_steers(self, ctx):
    """2026-09-29: every big-to-small hand-back while steering stepped the commanded torque
    by 0.09 to 0.19 in one tick, from switching to the idle controller's stale state. The
    controller that steers carries the small model until lateral has been inactive for
    TUNE_SWAP_INACTIVE_FRAMES in a row: a one-frame drop or a 0.3 s one keeps it."""
    params, controls = ctx
    params.put("TorqueControlTune", 2.0, block=True)
    params.put("TorqueControlTuneBig", 1.0, block=True)
    select(controls)
    swap(controls, big=True)            # swapped in with nothing in control
    big = controls.LaC

    def frame(lat_active):
      ControlsLateralZP.note_lat_active(controls, lat_active)
      swap(controls, big=False)

    assert big == V1 and big.resets == 1
    for gap in (0, 1, 30, TUNE_SWAP_INACTIVE_FRAMES - 1):
      for _ in range(20):               # handed back while steering
        frame(True)
      for _ in range(gap):              # a steer fault flicker, a short blinker pause
        frame(False)
      frame(True)
      assert controls.LaC is big and big.resets == 1, f"swapped after a {gap}-frame gap"
    for _ in range(TUNE_SWAP_INACTIVE_FRAMES):   # half a second off: a disengage, a stop
      frame(False)
    assert controls.LaC == V2 and controls.LaC.resets == 1
    for _ in range(10):                 # and the next engagement is on the small tune
      frame(True)
    assert controls.LaC == V2 and controls.LaC.resets == 1

  def test_switching_while_inactive_is_unchanged(self, ctx):
    params, controls = ctx
    params.put("TorqueControlTune", 2.0, block=True)
    params.put("TorqueControlTuneBig", 1.0, block=True)
    select(controls)
    for big, tune in ((True, V1), (False, V2), (True, V1)):
      swap(controls, big=big)
      assert controls.LaC == tune

  def test_enforce_off_ignores_the_big_tune(self, ctx):
    """The enforce-off v0 forcing applies to both sizes."""
    params, controls = ctx
    params.put_bool("EnforceTorqueControl", False, block=True)
    params.put("TorqueControlTune", 1.0, block=True)
    params.put("TorqueControlTuneBig", 2.0, block=True)
    assert select(controls) == V0
    swap(controls, big=True)
    assert controls.LaC == V0 and controls.LaC.resets == 0

  def test_ui_default_matches_what_controls_runs(self, ctx):
    """For an unset param the MICI selector lights up the declared default (the widget itself
    is pinned by test_torque_tune_unset_shows_declared_default); that version must be the one
    initialize_lateral_control picks, or the UI claims a tune the car isn't running."""
    from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import versions_by_label

    params, controls = ctx
    params.remove("TorqueControlTune")

    shown = float(params.get("TorqueControlTune", return_default=True))
    assert shown in set(versions_by_label().values()), \
      "the declared default must be a version the selectors offer"
    assert BY_VERSION[shown] == select(controls)

  @pytest.mark.parametrize(("small", "big", "enforce", "expected"), [
    (2.0, 2.0, True, False), (2.0, 42.0, True, False), (42.0, 2.0, True, False), (42.0, 42.0, True, False),
    (2.0, 1.0, True, True), (0.0, 2.0, True, True), (2.0, 2.0, False, True),
    (42.0, 43.0, True, False), (43.0, 43.0, True, False), (43.0, 40.0, True, True),
    (40.0, 42.0, True, True), (42.0, 40.0, True, True), (1.0, 42.0, True, True), (40.0, 40.0, True, True)])
  def test_jerk_aware_has_effect_unless_every_size_runs_v2_or_v42(self, ctx, small, big, enforce, expected):
    """v2 forces the jerk-aware controller off, so its toggle only locks when both sizes run
    v2; with Enforce Torque Control off both run v0."""
    from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import jerk_aware_has_effect

    params, controls = ctx
    params.put_bool("EnforceTorqueControl", enforce, block=True)
    params.put("TorqueControlTune", small, block=True)
    params.put("TorqueControlTuneBig", big, block=True)
    assert jerk_aware_has_effect(params, controls.CP) is expected


def _with_fingerprint(controls, fingerprint: str):
  CP = car.CarParams.new_message(steerControlType="torque", carFingerprint=fingerprint)
  CP.lateralTuning.init('torque')
  controls.CP = CP.as_reader()


def _assert_seeds(params, seeds):
  for key, value in seeds.items():
    assert params.get(key) == value, key  # typed params layer: get returns bool/int/float


class TestTorqueTuneTiSeed:
  """TI cars get torque enforcement plus the recommended lateral bundle (incl. NNLC),
  seeded into unset params only. No delay seeds on either platform — the learner
  stays stock-default ON and user-changeable."""

  def test_ti_on_unset_seeds_v2_and_enforce(self, ctx):
    params, controls = ctx
    params.put_bool("TorqueInterceptorEnabled", True, block=True)
    assert select(controls) == V2
    assert params.get_bool("EnforceTorqueControl") is True
    assert params.get("TorqueControlTune", return_default=True) == 2.0  # seed persisted

  def test_ti_on_explicit_v0_persists(self, ctx):
    params, controls = ctx
    params.put_bool("TorqueInterceptorEnabled", True, block=True)
    params.put("TorqueControlTune", 0.0, block=True)
    assert select(controls) == V0
    assert params.get("TorqueControlTune", return_default=True) == 0.0

  def test_ti_on_explicit_v1_persists(self, ctx):
    params, controls = ctx
    params.put_bool("TorqueInterceptorEnabled", True, block=True)
    params.put("TorqueControlTune", 1.0, block=True)
    assert select(controls) == V1

  def test_ti_on_enforce_explicitly_off_stays_off(self, ctx):
    """A user who explicitly disabled enforcement keeps it — the v0 pin then applies."""
    params, controls = ctx
    params.put_bool("TorqueInterceptorEnabled", True, block=True)
    params.put_bool("EnforceTorqueControl", False, block=True)
    assert select(controls) == V0
    assert params.get_bool("EnforceTorqueControl") is False

  def test_ti_off_unset_seeds_nothing(self, ctx):
    params, controls = ctx
    params.remove("EnforceTorqueControl")  # the fixture sets it; this test needs it genuinely unset
    assert select(controls) == V0
    assert params.get("TorqueControlTune") is None
    assert params.get("EnforceTorqueControl") is None


  def test_cx8_gets_base_bundle_and_live_delay_left_alone(self, ctx):
    params, controls = ctx
    _with_fingerprint(controls, "MAZDA_CX8_2022")
    params.put_bool("TorqueInterceptorEnabled", True, block=True)
    select(controls)
    _assert_seeds(params, controls_lateral_zp.TI_TUNE_SEEDS)
    # owner-validated: "Live Delay" (lagd learner) beats a fixed manual delay, so the CX-8
    # gets NO delay seed — stock LagdToggle=1 default applies and stays user-changeable
    assert params.get("LagdToggle") is None
    assert params.get("LagdToggleDelay") is None

  def test_cx5_gets_base_bundle_without_delay_pair(self, ctx):
    params, controls = ctx
    _with_fingerprint(controls, "MAZDA_CX5_2022")
    params.put_bool("TorqueInterceptorEnabled", True, block=True)
    select(controls)
    _assert_seeds(params, controls_lateral_zp.TI_TUNE_SEEDS)
    assert params.get("LagdToggle") is None
    assert params.get("LagdToggleDelay") is None

  def test_non_seeded_platform_gets_enforce_but_no_bundle(self, ctx):
    params, controls = ctx  # fixture fingerprint is a non-seeded platform ("")
    params.put_bool("TorqueInterceptorEnabled", True, block=True)
    select(controls)
    assert params.get_bool("EnforceTorqueControl")
    assert params.get("LiveTorqueParamsToggle") is None

  def test_swapped_cx5_ti_seeds_tune_without_bundle(self, ctx):
    # Paul's 2020 CX-5 + donor EPS fingerprints MAZDA_CX5 (not MAZDA_CX5_2022), so it is
    # outside TI_PLATFORMS: TI seeds enforcement and the v2 tune, but not the bundle.
    # Its self-tune toggle arrives via the EPS-hardware seed (_seed_mazda_torque_defaults).
    params, controls = ctx
    _with_fingerprint(controls, "MAZDA_CX5")
    params.put_bool("TorqueInterceptorEnabled", True, block=True)
    assert select(controls) == V2
    assert params.get("TorqueControlTune", return_default=True) == 2.0
    for key in controls_lateral_zp.TI_TUNE_SEEDS:
      assert params.get(key) is None, key

  def test_cx8_2023_ti_seeds_tune_without_bundle(self, ctx):
    # Upstream's CX-8 2023 is a distinct platform from the owner's CX-8 2022, not a
    # spelling fix: it must not inherit the 2022's TI bundle by collapsing the names.
    params, controls = ctx
    _with_fingerprint(controls, "MAZDA_CX8_2023")
    params.put_bool("TorqueInterceptorEnabled", True, block=True)
    assert select(controls) == V2
    assert params.get("TorqueControlTune", return_default=True) == 2.0
    for key in controls_lateral_zp.TI_TUNE_SEEDS:
      assert params.get(key) is None, key


class TestRealTorqueDispatch:
  @staticmethod
  def _controls(params, small, big, ti):
    CP = car.CarParams.new_message(steerControlType="torque", carFingerprint="MAZDA_CX5",
                                   brand="mazda", steerAtStandstill=True, steerLimitTimer=0.4)
    if ti:
      CP.flags = int(MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO | MazdaFlags.TORQUE_INTERCEPTOR)
    CP.lateralTuning.init("torque")
    CP.lateralTuning.torque.latAccelFactor = 2.5
    params.put_bool("EnforceTorqueControl", True, block=True)
    params.put("TorqueControlTune", small, block=True)
    params.put("TorqueControlTuneBig", big, block=True)
    return SimpleNamespace(params=params, CP=CP.as_reader(), CP_SP=custom.CarParamsSP.new_message().as_reader(),
                           _inactive_frames=TUNE_SWAP_INACTIVE_FRAMES)

  @staticmethod
  def _version(lac):
    from openpilot.sunnypilot.selfdrive.controls.tests.test_latcontrol_torque_v42 import LAT_DELAY, LP, VM, make_cs
    return lac.update(True, make_cs(), VM, LP, False, 0.001, None, False, LAT_DELAY)[2].version

  def test_real_ti_dispatch_and_baseline_constructor(self):
    from openpilot.sunnypilot.selfdrive.controls.tests.test_latcontrol_torque_v42 import make_ci
    with OpenpilotPrefix():
      params = Params()
      def build(controls):
        controls.LaC = ControlsLateralZP.initialize_lateral_control(controls, FakeLaC(V1), make_ci(), 0.01)
        return controls.LaC
      controls = self._controls(params, 40.0, 42.0, True)
      build(controls)
      assert type(controls.LaC).__module__.endswith("latcontrol_torque_v40")
      assert type(controls._lac_by_size[True]).__module__.endswith("latcontrol_torque_v42")
      assert self._version(controls.LaC) == 40
      assert self._version(controls._lac_by_size[True]) == 42

      controls = self._controls(params, 2.0, 2.0, True)
      build(controls)
      assert type(controls.LaC).__module__.endswith("latcontrol_torque_v2")
      assert self._version(controls.LaC) == 2

      controls = self._controls(params, 40.0, 42.0, False)
      build(controls)
      assert type(controls.LaC).__module__.endswith("latcontrol_torque_v0")
      assert controls._lac_by_size[False] is controls._lac_by_size[True]
      assert self._version(controls.LaC) == 0
      assert params.get("TorqueControlTune") == 40.0
      assert params.get("TorqueControlTuneBig") == 42.0

  @pytest.mark.parametrize("small,big", [(43.0, 42.0), (42.0, 43.0), (43.0, 43.0), (42.0, 42.0)])
  def test_real_v43_dispatch_and_mode(self, small, big):
    from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import MazdaTorqueV2Mode
    from openpilot.sunnypilot.selfdrive.controls.tests.test_latcontrol_torque_v42 import make_ci
    with OpenpilotPrefix():
      params = Params()
      params.put("MazdaTorqueV2Mode", 1, block=True)
      controls = self._controls(params, small, big, True)
      controls.LaC = ControlsLateralZP.initialize_lateral_control(controls, FakeLaC(V1), make_ci(), 0.01)
      for size, version in ((False, small), (True, big)):
        lac = controls._lac_by_size[size]
        assert type(lac).__module__.endswith(f"latcontrol_torque_v{int(version)}")
        assert self._version(lac) == int(version)
        assert lac.mazda_v2_mode == (MazdaTorqueV2Mode.B if version == 42.0 else MazdaTorqueV2Mode.A)
      assert (controls._lac_by_size[False] is controls._lac_by_size[True]) == (small == big)


def test_model_size_and_tune_swap_logging(ctx, monkeypatch):
  params, controls = ctx
  params.put("TorqueControlTune", 43.0, block=True)
  params.put("TorqueControlTuneBig", 42.0, block=True)
  info, warning = MagicMock(), MagicMock()
  monkeypatch.setattr(controls_lateral_zp.cloudlog, "info", info)
  monkeypatch.setattr(controls_lateral_zp.cloudlog, "warning", warning)
  select(controls)
  class SubMaster:
    def __init__(self, big=True, updated=True, valid=True, mono=123456):
      self.logMonoTime = {'modelV2': mono}
      self.updated = {'modelV2': updated}
      self.valid = {'modelV2': valid}
      self._big = big
    def __getitem__(self, key):
      return SimpleNamespace(big=self._big)
  controls._inactive_frames = 0
  # a stale frame and then an invalid frame: neither advances identity or logs
  ControlsLateralZP.select_lateral_control(controls, SubMaster(updated=False, mono=0))
  assert controls._model_size is None
  ControlsLateralZP.select_lateral_control(controls, SubMaster(valid=False, mono=0))
  assert controls._model_size is None
  # first fresh valid frame logs exactly once; a repeat does not
  ControlsLateralZP.select_lateral_control(controls, SubMaster())
  ControlsLateralZP.select_lateral_control(controls, SubMaster())
  transitions = [call for call in info.call_args_list if 'model size' in call.args[0]]
  assert len(transitions) == 1
  assert transitions[0].args[1:] == ('unknown', 'big', 123456)
  assert controls.LaC == V43
  controls._inactive_frames = TUNE_SWAP_INACTIVE_FRAMES
  ControlsLateralZP.select_lateral_control(controls, SubMaster())
  assert controls.LaC == V42
  assert warning.call_args.args[1:] == (43.0, 42.0, 'big', 43.0, 42.0)
