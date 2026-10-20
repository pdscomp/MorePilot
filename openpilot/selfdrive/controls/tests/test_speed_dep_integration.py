"""Integration tests for speed-dependent torque — controller pipeline.

Tests the full data flow from torqued output through controlsd to the
torque_params used in the steering controller: per-frame latAccelFactor and friction
interpolation, sanity bounds, toggle-off behavior, and manual override.

Tests LatControlTorqueExtZP directly (the class that owns the
per-frame interpolation logic) rather than LatControlTorqueExt, which
inherits from NNLC and requires model files to init.
"""
import numpy as np
import pytest

from unittest.mock import MagicMock, patch  # noqa: TID251
from opendbc.sunnypilot.car.interfaces import get_speed_dep_config, get_speed_dep_config_for_car
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_zp import LatControlTorqueExtZP


class _TestExt(LatControlTorqueExtZP, LatControlTorqueExtOverride):
  """The production LatControlTorqueExt init chain: Override sets self.params, ext_zp owns the
  per-frame interpolation under test."""
  def __init__(self, CP):
    LatControlTorqueExtOverride.__init__(self, CP)
    LatControlTorqueExtZP.__init__(self, CP)

SPEED_DEP_CARS = get_speed_dep_config()

PATCH_PARAMS_OVERRIDE = 'openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override.Params'
PATCH_PARAMS_TORQUED_EXT = 'openpilot.sunnypilot.selfdrive.locationd.torqued_ext.Params'
PATCH_PARAMS_TORQUED = 'openpilot.selfdrive.locationd.torqued.Params'
PATCH_GET_SPEED_DEP_CONFIG = 'opendbc.sunnypilot.car.lateral_tune.get_speed_dep_config'  # the helper's home since the lateral_tune split

# Sample tables
SAMPLE_SPEED_BP = [6.5, 10.0, 15.0, 21.0, 26.5, 32.0, 37.5]
SAMPLE_LAT_ACCEL_FACTOR_BP = [2.39, 2.52, 2.71, 2.39, 2.28, 2.22, 2.21]
SAMPLE_FRICTION_BP = [0.177, 0.158, 0.131, 0.118, 0.113, 0.109, 0.108]


class TorqueParams:
  """Mutable stand-in for CarParams.LateralTorqueTuning builder."""
  def __init__(self, latAccelFactor=2.0, latAccelOffset=0.0, friction=0.15):
    self.latAccelFactor = latAccelFactor
    self.latAccelOffset = latAccelOffset
    self.friction = friction


@patch(PATCH_PARAMS_OVERRIDE)
def make_override(mock_params_cls, enforce=False, manual_override=False,
                  manual_lat_accel_factor='200', manual_friction='15'):
  """Create a LatControlTorqueExtOverride with mocked Params."""
  mock_inst = mock_params_cls.return_value
  mock_inst.get_bool.side_effect = lambda k: {
    'EnforceTorqueControl': enforce,
    'TorqueParamsOverrideEnabled': manual_override,
  }.get(k, False)
  mock_inst.get.side_effect = lambda k, **kw: {
    'TorqueParamsOverrideLatAccelFactor': manual_lat_accel_factor,
    'TorqueParamsOverrideFriction': manual_friction,
  }.get(k)

  CP = MagicMock()
  ovr = _TestExt(CP)
  return ovr


def activate_speed_dep(ovr, speed_bp=None, lat_accel_factor_bp=None, friction_bp=None):
  """Simulate update_speed_dep_torque setting tables on the override."""
  ovr._speed_dep_active = True
  ovr._speed_dep_speed_bp = speed_bp or list(SAMPLE_SPEED_BP)
  ovr._speed_dep_lat_accel_factor_bp = lat_accel_factor_bp or list(SAMPLE_LAT_ACCEL_FACTOR_BP)
  ovr._speed_dep_friction_bp = friction_bp or list(SAMPLE_FRICTION_BP)


class TestLafInterpolatedBySpeed:
  """torque_params.latAccelFactor must be speed-interpolated
  before torque_from_lateral_accel reads it."""

  def test_lat_accel_factor_set_to_interpolated_value(self):
    ovr = make_override()
    activate_speed_dep(ovr)
    tp = TorqueParams(latAccelFactor=999.0)  # sentinel

    ovr._last_vego = 10.0
    ovr.update_override_torque_params(tp)

    expected = float(np.interp(10.0, SAMPLE_SPEED_BP, SAMPLE_LAT_ACCEL_FACTOR_BP))
    assert tp.latAccelFactor == pytest.approx(expected, abs=1e-4), \
      f"latAccelFactor should be {expected}, got {tp.latAccelFactor}"

  def test_lat_accel_factor_differs_at_different_speeds(self):
    ovr = make_override()
    activate_speed_dep(ovr)

    tp = TorqueParams()
    ovr._last_vego = 6.5
    ovr.update_override_torque_params(tp)
    factor_low = tp.latAccelFactor

    tp = TorqueParams()
    ovr._last_vego = 37.5
    ovr.update_override_torque_params(tp)
    factor_high = tp.latAccelFactor

    assert factor_low != pytest.approx(factor_high, abs=0.01), \
      "latAccelFactor must differ between 6.5 m/s and 37.5 m/s"

  def test_lat_accel_factor_not_global_value(self):
    """latAccelFactor should NOT be the global scalar."""
    ovr = make_override()
    activate_speed_dep(ovr)
    global_factor = 2.0
    tp = TorqueParams(latAccelFactor=global_factor)

    ovr._last_vego = 6.5  # seed latAccelFactor at 6.5 is 2.39, not 2.0
    ovr.update_override_torque_params(tp)

    assert tp.latAccelFactor != pytest.approx(global_factor, abs=0.01), \
      "latAccelFactor should be speed-interpolated, not the global value"


class TestFrictionInterpolatedBySpeed:
  """torque_params.friction must be speed-interpolated
  before get_friction reads it."""

  def test_friction_set_to_interpolated_value(self):
    ovr = make_override()
    activate_speed_dep(ovr)
    tp = TorqueParams(friction=999.0)

    ovr._last_vego = 35.0
    ovr.update_override_torque_params(tp)

    expected = float(np.interp(35.0, SAMPLE_SPEED_BP, SAMPLE_FRICTION_BP))
    assert tp.friction == pytest.approx(expected, abs=1e-4)

  def test_friction_differs_at_different_speeds(self):
    ovr = make_override()
    activate_speed_dep(ovr)

    tp = TorqueParams()
    ovr._last_vego = 6.5
    ovr.update_override_torque_params(tp)
    fric_low = tp.friction

    tp = TorqueParams()
    ovr._last_vego = 37.5
    ovr.update_override_torque_params(tp)
    fric_high = tp.friction

    assert fric_low != pytest.approx(fric_high, abs=0.01)


class TestToggleOffClearsState:
  """_speed_dep_active must be cleared when bins disappear."""

  def test_inactive_by_default(self):
    ovr = make_override()
    assert not ovr._speed_dep_active

  def test_deactivated_does_not_modify_params(self):
    ovr = make_override()
    activate_speed_dep(ovr)
    # Deactivate
    ovr._speed_dep_active = False

    tp = TorqueParams(latAccelFactor=99.0, friction=99.0)
    ovr._last_vego = 15.0
    ovr.update_override_torque_params(tp)

    assert tp.latAccelFactor == 99.0, "Should not modify params when inactive"
    assert tp.friction == 99.0

  def test_empty_speed_bp_does_not_modify_params(self):
    ovr = make_override()
    activate_speed_dep(ovr)
    ovr._speed_dep_speed_bp = []  # empty

    tp = TorqueParams(latAccelFactor=99.0, friction=99.0)
    ovr._last_vego = 15.0
    ovr.update_override_torque_params(tp)

    assert tp.latAccelFactor == 99.0
    assert tp.friction == 99.0


class TestManualOverridePriority:
  """Manual override must take priority over speed-dep."""

  def test_manual_overwrites_speed_dep(self):
    ovr = make_override(enforce=True, manual_override=True,
                        manual_lat_accel_factor='350', manual_friction='25')
    activate_speed_dep(ovr)
    ovr._last_vego = 15.0

    tp = TorqueParams()
    # frame = -1, after +1 -> frame=0, 0 % 300 == 0 -> manual fires
    ovr.update_override_torque_params(tp)

    assert tp.latAccelFactor == pytest.approx(350.0, abs=0.1), \
      "Manual latAccelFactor should overwrite speed-dep"
    assert tp.friction == pytest.approx(25.0, abs=0.1), \
      "Manual friction should overwrite speed-dep"

  def test_manual_wins_every_frame(self):
    """The manual override must own the params on EVERY frame, not just the 3 s poll
    frame — the per-frame speed-dep interpolation used to out-write it 299/300 frames."""
    ovr = make_override(enforce=True, manual_override=True,
                        manual_lat_accel_factor='350', manual_friction='25')
    activate_speed_dep(ovr)
    ovr._last_vego = 15.0

    tp = TorqueParams()
    for frame in range(10):
      ovr.update_override_torque_params(tp)
      assert tp.latAccelFactor == pytest.approx(350.0), f"speed-dep out-wrote manual on frame {frame}"
      assert tp.friction == pytest.approx(25.0), f"speed-dep out-wrote manual friction on frame {frame}"

  def test_manual_toggle_off_mid_drive_returns_to_speed_dep(self):
    """Flipping the override off mid-drive hands the params back to speed-dep at the
    next 3 s poll."""
    ovr = make_override(enforce=True, manual_override=True,
                        manual_lat_accel_factor='350', manual_friction='25')
    activate_speed_dep(ovr)
    ovr._last_vego = 15.0

    tp = TorqueParams()
    ovr.update_override_torque_params(tp)
    assert tp.latAccelFactor == pytest.approx(350.0)

    ovr.params.get_bool.side_effect = lambda k: {'EnforceTorqueControl': True,
                                                 'TorqueParamsOverrideEnabled': False}.get(k, False)
    for _ in range(301):  # crosses the next poll frame
      ovr.update_override_torque_params(tp)

    expected_factor = float(np.interp(15.0, SAMPLE_SPEED_BP, SAMPLE_LAT_ACCEL_FACTOR_BP))
    assert tp.latAccelFactor == pytest.approx(expected_factor, abs=1e-4)

  def test_speed_dep_used_when_manual_off(self):
    ovr = make_override(enforce=True, manual_override=False)
    activate_speed_dep(ovr)
    ovr._last_vego = 15.0

    tp = TorqueParams()
    ovr.update_override_torque_params(tp)

    expected_factor = float(np.interp(15.0, SAMPLE_SPEED_BP, SAMPLE_LAT_ACCEL_FACTOR_BP))
    assert tp.latAccelFactor == pytest.approx(expected_factor, abs=1e-4), \
      "Without manual override, speed-dep should be used"


class TestChangeDetection:
  """update_override_torque_params should only return changed=True when values differ."""

  def test_no_change_returns_false(self):
    ovr = make_override()
    activate_speed_dep(ovr)
    ovr._last_vego = 15.0

    tp = TorqueParams()
    # First call sets values
    ovr.update_override_torque_params(tp)
    # Second call at same speed — no change
    changed = ovr.update_override_torque_params(tp)
    assert not changed, "Should return False when values haven't changed"

  def test_speed_change_returns_true(self):
    ovr = make_override()
    activate_speed_dep(ovr)

    tp = TorqueParams()
    ovr._last_vego = 6.5
    ovr.update_override_torque_params(tp)

    ovr._last_vego = 37.5  # big speed change -> values change
    changed = ovr.update_override_torque_params(tp)
    assert changed, "Should return True when values changed"


class TestLearnerSanityBounds:
  """Speed-bin sanity bounds must allow learning regardless of
  the 'Less Restrict' toggle."""

  @patch(PATCH_PARAMS_TORQUED_EXT)
  @patch(PATCH_PARAMS_TORQUED)
  def test_sanity_bounds_allow_learning_without_relaxed(self, mock_params_cls, mock_ext_cls):
    """With LiveTorqueParamsRelaxedToggle OFF (factor_sanity stays at torqued's 0.3
    default), speed bins must have +/-30% bounds, not (seed, seed)."""
    mock_params_cls.return_value.get.return_value = None
    mock_ext_cls.return_value.get_bool.side_effect = lambda k: {
      'EnforceTorqueControl': True,
      'LiveTorqueParamsToggle': True,
      'LiveTorqueParamsRelaxedToggle': False,
    }.get(k, False)
    mock_ext_cls.return_value.get.return_value = None

    from openpilot.selfdrive.locationd.torqued import TorqueEstimator
    CP = MagicMock()
    CP.brand = 'test'
    CP.flags = 0
    CP.minSteerSpeed = 0.0
    CP.carFingerprint = next(iter(SPEED_DEP_CARS)) if SPEED_DEP_CARS else 'FAKE'
    CP.lateralTuning.which.return_value = 'torque'
    CP.lateralTuning.torque.latAccelFactor = 2.0
    CP.lateralTuning.torque.friction = 0.15

    est = TorqueEstimator(CP)
    est._on_torque_point(0.1, 0.3, 10.0)  # trigger lazy init

    for i, (lo, hi) in enumerate(est.speed_bin_lat_accel_factor_bounds):
      assert hi > lo, f"Bin {i} latAccelFactor bounds ({lo:.3f}, {hi:.3f}) must allow a range"

    for i, (lo, hi) in enumerate(est.speed_bin_friction_bounds):
      assert hi > lo, f"Bin {i} friction bounds ({lo:.3f}, {hi:.3f}) must allow a range"

  @patch(PATCH_PARAMS_TORQUED_EXT)
  @patch(PATCH_PARAMS_TORQUED)
  def test_clip_allows_10pct_movement(self, mock_params_cls, mock_ext_cls):
    """A learned value 10% above seed should pass through np.clip with +/-30% bounds."""
    mock_params_cls.return_value.get.return_value = None
    mock_ext_cls.return_value.get_bool.side_effect = lambda k: {
      'EnforceTorqueControl': True,
      'LiveTorqueParamsToggle': True,
    }.get(k, False)
    mock_ext_cls.return_value.get.return_value = None

    from openpilot.selfdrive.locationd.torqued import TorqueEstimator
    CP = MagicMock()
    CP.brand = 'test'
    CP.flags = 0
    CP.minSteerSpeed = 0.0
    CP.carFingerprint = next(iter(SPEED_DEP_CARS)) if SPEED_DEP_CARS else 'FAKE'
    CP.lateralTuning.which.return_value = 'torque'
    CP.lateralTuning.torque.latAccelFactor = 2.0
    CP.lateralTuning.torque.friction = 0.15

    est = TorqueEstimator(CP)
    est._on_torque_point(0.1, 0.3, 10.0)

    seed_factor = est.speed_bin_filtered[0]['latAccelFactor'].x
    nudged = seed_factor * 1.10
    lo, hi = est.speed_bin_lat_accel_factor_bounds[0]
    clipped = np.clip(nudged, lo, hi)
    assert clipped == pytest.approx(nudged, abs=1e-6), \
      f"+10% nudge ({nudged:.3f}) should not be clipped by +/-30% bounds ({lo:.3f}, {hi:.3f})"


class TestToggleOffFallback:
  """When speed-dep is deactivated, controller must not use stale tables."""

  def test_deactivation_via_empty_bins(self):
    """Simulates toggle-off: update_speed_dep_torque receives empty bins."""
    ovr = make_override()
    activate_speed_dep(ovr)
    assert ovr._speed_dep_active

    # Simulate toggle-off (torqued sends empty speedBinCenters)
    ovr._speed_dep_active = False

    tp = TorqueParams(latAccelFactor=2.35, friction=0.12)
    ovr._last_vego = 15.0
    ovr.update_override_torque_params(tp)

    # Global values should pass through unmodified
    assert tp.latAccelFactor == 2.35
    assert tp.friction == 0.12

  def test_reactivation_after_deactivation(self):
    """Speed-dep can be re-enabled after being disabled."""
    ovr = make_override()
    ovr._speed_dep_active = False

    # Re-enable
    activate_speed_dep(ovr)
    assert ovr._speed_dep_active

    tp = TorqueParams()
    ovr._last_vego = 15.0
    ovr.update_override_torque_params(tp)

    expected_factor = float(np.interp(15.0, SAMPLE_SPEED_BP, SAMPLE_LAT_ACCEL_FACTOR_BP))
    assert tp.latAccelFactor == pytest.approx(expected_factor, abs=1e-4)


class TestUpdateSpeedDepTorqueFallback:
  """Tests for update_speed_dep_torque fallback logic (TOML seeds vs global filtered)."""

  @staticmethod
  def _make_mock_tp(use_params=True, global_laf=2.0, global_fric=0.15):
    """Upstream's lateralTorqueParameters: useParams and the global filtered values."""
    tp = MagicMock()
    tp.useParams = use_params
    tp.latAccelFactorFiltered = global_laf
    tp.frictionCoefficientFiltered = global_fric
    tp.latAccelOffsetFiltered = 0.0
    return tp

  @staticmethod
  def _make_mock_tp_sp(speed_bp, lafs, frictions, valid):
    """The fork's liveTorqueParametersSP: the per-bin values."""
    tp_sp = MagicMock()
    tp_sp.speedBinCenters = speed_bp
    tp_sp.speedBinLatAccelFactors = lafs
    tp_sp.speedBinFrictions = frictions
    tp_sp.speedBinValid = valid
    return tp_sp

  @staticmethod
  def _make_mock_self(fingerprint='TEST_CAR'):
    mock_self = MagicMock()
    mock_self.CP.carFingerprint = fingerprint
    # real scalars where get_speed_dep_config_for_car dereferences them
    mock_self.CP.brand = 'test'
    mock_self.CP.flags = 0
    mock_self.CP.minSteerSpeed = 0.0
    mock_self._speed_dep_active = False
    mock_self._speed_dep_car_cfg = None
    mock_self._speed_dep_speed_bp = []
    mock_self._speed_dep_lat_accel_factor_bp = []
    mock_self._speed_dep_friction_bp = []
    return mock_self

  # RETIRED test_toml_seeds_used_for_invalid_bins: controller-side TOML-seed substitution for
  # invalid bins, superseded by the learner-owned seed-fill contract — see the retirement map
  # below (test_a_calculable_bin_keeps_its_seed_until_valid + test_bins_taken_as_published +
  # test_mazda_actuator_units.py).

  # RETIRED with the learner-owned seed-fill contract (speed_bin_learner pre-fills every
  # published invalid bin with the car's TOML seeds, so the controller takes bins as
  # published — controller-side substitution never runs). Removed nodes and their retained
  # equivalents:
  # - test_global_filtered_fallback_when_no_config / test_friction_bp_missing_uses_global_fallback
  #   / test_laf_bp_length_mismatch_uses_global_fallback (controller substituted global filtered
  #   values for invalid bins) -> locationd/tests/test_torqued_speed_dep.py
  #   ::test_default_bins_seeded_with_offline_values covers the published seed state;
  #   test_speed_dep_ext_update.py::test_bins_taken_as_published covers the controller contract.
  # - test_mixed_valid_invalid_bins (controller substituted TOML seeds for invalid bins) ->
  #   test_torqued_speed_dep.py::test_a_calculable_bin_keeps_its_seed_until_valid (seed until
  #   valid), ::test_invalid_bin_is_not_fit (no refit), and
  #   test_mazda_actuator_units.py::test_ti_learned_bins_keep_old_counts (exact wire counts with
  #   seed-carrying invalid bins on the TI cohort).

  @patch(PATCH_GET_SPEED_DEP_CONFIG)
  def test_empty_bins_deactivate(self, mock_get_config):
    """Empty bins route through the single deactivation path (CP-tune restore included)."""
    mock_get_config.return_value = {}
    mock_self = self._make_mock_self()
    mock_self._speed_dep_active = True

    mock_tp = self._make_mock_tp()
    mock_tp_sp = self._make_mock_tp_sp([], [], [], [])

    LatControlTorqueExt.update_speed_dep_torque(mock_self, mock_tp, mock_tp_sp)

    mock_self.disable_speed_dep_torque.assert_called_once()

  @patch(PATCH_GET_SPEED_DEP_CONFIG)
  def test_use_params_off_deactivates(self, mock_get_config):
    """useParams flipping off mid-drive (manual override enabled) routes through the
    same deactivation path even with bins still present."""
    mock_get_config.return_value = {}
    mock_self = self._make_mock_self()
    mock_self._speed_dep_active = True

    mock_tp = self._make_mock_tp(use_params=False)
    mock_tp_sp = self._make_mock_tp_sp([6.5, 10.0], [2.0, 2.0], [0.1, 0.1], [True, True])

    LatControlTorqueExt.update_speed_dep_torque(mock_self, mock_tp, mock_tp_sp)

    mock_self.disable_speed_dep_torque.assert_called_once()

  @patch(PATCH_GET_SPEED_DEP_CONFIG)
  def test_no_fork_message_deactivates(self, mock_get_config):
    """tp_sp None (liveTorqueParametersSP not yet checked out) routes through the same
    deactivation path even with useParams still on."""
    mock_get_config.return_value = {}
    mock_self = self._make_mock_self()
    mock_self._speed_dep_active = True

    LatControlTorqueExt.update_speed_dep_torque(mock_self, self._make_mock_tp(), None)

    mock_self.disable_speed_dep_torque.assert_called_once()

  def test_disable_speed_dep_restores_cp_tune(self):
    """Mid-drive de-assert (useParams flipped off): the controller must return to the
    CP tune instead of keeping the last interpolated values forever."""
    mock_self = self._make_mock_self()
    mock_self._speed_dep_active = True
    tune = mock_self.CP.lateralTuning.torque
    tune.latAccelFactor = 2.5
    tune.latAccelOffset = 0.05
    tune.friction = 0.12

    LatControlTorqueExt.disable_speed_dep_torque(mock_self)

    assert mock_self._speed_dep_active is False
    assert mock_self.lac_torque.torque_params.latAccelFactor == 2.5
    assert mock_self.lac_torque.torque_params.latAccelOffset == 0.05
    assert mock_self.lac_torque.torque_params.friction == 0.12
    mock_self.lac_torque.update_limits.assert_called_once()

  def test_disable_speed_dep_noop_when_inactive(self):
    mock_self = self._make_mock_self()
    LatControlTorqueExt.disable_speed_dep_torque(mock_self)
    mock_self.lac_torque.update_limits.assert_not_called()


class TestSeedValidityGate:
  """Entries measured on a steer-to-zero EPS declare requires_steer_to_zero and must not
  apply to the same model with its stock EPS (different STEER_MAX schedule → mis-scaled LAF)."""

  @staticmethod
  def _cp(fingerprint, min_steer_speed):
    from types import SimpleNamespace
    return SimpleNamespace(carFingerprint=fingerprint, minSteerSpeed=min_steer_speed,
                           brand='test', flags=0)

  @patch(PATCH_GET_SPEED_DEP_CONFIG)
  def test_flagged_entry_suppressed_on_stock_eps(self, mock_get_config):
    mock_get_config.return_value = {'SWAP_CAR': {'requires_steer_to_zero': True, 'speed_bp': [10.0]}}
    assert get_speed_dep_config_for_car(self._cp('SWAP_CAR', 12.5)) == {}

  @patch(PATCH_GET_SPEED_DEP_CONFIG)
  def test_flagged_entry_applies_with_swap_eps(self, mock_get_config):
    cfg = {'requires_steer_to_zero': True, 'speed_bp': [10.0]}
    mock_get_config.return_value = {'SWAP_CAR': cfg}
    assert get_speed_dep_config_for_car(self._cp('SWAP_CAR', 0.0)) == cfg

  @patch(PATCH_GET_SPEED_DEP_CONFIG)
  def test_unflagged_entry_applies_regardless(self, mock_get_config):
    cfg = {'speed_bp': [15.0]}  # above the 12.5 floor, so the bin filter keeps it
    mock_get_config.return_value = {'PLAIN_CAR': cfg}
    assert get_speed_dep_config_for_car(self._cp('PLAIN_CAR', 12.5)) == cfg

  def test_real_toml_flags_are_as_intended(self):
    """The swap-chassis entries (KE chassis and friends) substitute the CX-5 2022 table
    learned behind the donor EPS and must carry the flag; the native CX-5 2022 and CX-9 2021
    tables stand on their own and must not (the CX-9's stock-EPS floor is handled by
    dropping the bins below it, not by suppressing the entry)."""
    cars = get_speed_dep_config()
    assert cars['MAZDA_CX5_KE'].get('requires_steer_to_zero') is True
    assert 'requires_steer_to_zero' not in cars['MAZDA_CX9_2021']
    assert 'requires_steer_to_zero' not in cars['MAZDA_CX5_2022']


class TestExtrapolationAtBoundaries:
  """np.interp clamps to edge values for speeds outside the bin range."""

  def test_speed_below_first_bin_clamps(self):
    ovr = make_override()
    activate_speed_dep(ovr)
    tp = TorqueParams()
    ovr._last_vego = 0.0
    ovr.update_override_torque_params(tp)
    assert tp.latAccelFactor == pytest.approx(SAMPLE_LAT_ACCEL_FACTOR_BP[0], abs=1e-4)
    assert tp.friction == pytest.approx(SAMPLE_FRICTION_BP[0], abs=1e-4)

  def test_speed_above_last_bin_clamps(self):
    ovr = make_override()
    activate_speed_dep(ovr)
    tp = TorqueParams()
    ovr._last_vego = 100.0
    ovr.update_override_torque_params(tp)
    assert tp.latAccelFactor == pytest.approx(SAMPLE_LAT_ACCEL_FACTOR_BP[-1], abs=1e-4)
    assert tp.friction == pytest.approx(SAMPLE_FRICTION_BP[-1], abs=1e-4)

  def test_speed_at_exact_bin_center(self):
    ovr = make_override()
    activate_speed_dep(ovr)
    for i, speed in enumerate(SAMPLE_SPEED_BP):
      tp = TorqueParams()
      ovr._last_vego = speed
      ovr.update_override_torque_params(tp)
      assert tp.latAccelFactor == pytest.approx(SAMPLE_LAT_ACCEL_FACTOR_BP[i], abs=1e-4)
      assert tp.friction == pytest.approx(SAMPLE_FRICTION_BP[i], abs=1e-4)
