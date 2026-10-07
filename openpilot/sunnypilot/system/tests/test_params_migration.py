"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import base64
import hashlib
import json

import pytest

from pathlib import Path

from openpilot.common.params import Params, ParamKeyFlag
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.system import params_migration
from openpilot.sunnypilot.system.params_migration import _migrate_model_bundle_slots, migrate_mazda_control_cache, \
  run_migration, stamp_mazda_control_cache_contract


class TestModelBundleSlotMigration(OpenpilotTestCase):
  """Pre-split, a chestnut user's big-model selection lived in the single ActiveBundle.
  The migration seeds both slots; per-source validation later drops whichever does not
  match its own manifest."""

  def test_seeds_chestnut_slot_from_active_bundle(self):
    params = Params()
    bundle = {"ref": "big", "minimumSelectorVersion": 18}
    params.put("ModelManager_ActiveBundle", bundle, block=True)
    _migrate_model_bundle_slots(params)
    assert params.get("ModelManager_ActiveBundleChestnut") == bundle
    assert params.get("ModelManager_ActiveBundle") == bundle

  def test_noop_when_chestnut_slot_already_set(self):
    params = Params()
    params.put("ModelManager_ActiveBundle", {"ref": "small"}, block=True)
    params.put("ModelManager_ActiveBundleChestnut", {"ref": "big"}, block=True)
    _migrate_model_bundle_slots(params)
    assert params.get("ModelManager_ActiveBundleChestnut") == {"ref": "big"}

  def test_noop_when_no_selection(self):
    params = Params()
    _migrate_model_bundle_slots(params)
    assert params.get("ModelManager_ActiveBundleChestnut") is None


# --- Mazda control-cache migration -------------------------------------------

# historical flag layouts; current-layout constants come from opendbc.car.mazda.values
V1_GEN1, V1_STZ, V1_TI, V1_LEGACY = 0x1, 0x2, 0x4, 0x8
V2_TI = 0x20

ARCHIVE_KEYS = ("CarParamsPersistent", "CarParamsSPPersistent", "CarParamsPrevRoute",
                "LiveTorqueParameters", "LiveTorqueParametersSP", "LiveParametersV2", "LiveDelay")


def _meta_dir(params) -> Path:
  return Path(params.get_param_path()).parent


def _archive_path(params) -> Path:
  return _meta_dir(params) / ".mazda_control_cache_archive.json"


def _contract_path(params) -> Path:
  return _meta_dir(params) / ".mazda_control_cache_contract.json"


def _read_contract(params) -> dict:
  return json.loads(_contract_path(params).read_text())


def _read_archive(params) -> dict:
  return json.loads(_archive_path(params).read_text())


def _mazda_cp(candidate, flags, min_steer_speed=None):
  """A real CarParams from get_params with the flags forced to the requested layout values."""
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.mazda.interface import CarInterface
  CP = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha_long=False, is_release=False, docs=False)
  CP.flags = flags
  if min_steer_speed is not None:
    CP.minSteerSpeed = min_steer_speed
  return CP


def _mazda_cp_bytes(candidate, flags, min_steer_speed=None) -> bytes:
  return _mazda_cp(candidate, flags, min_steer_speed).to_bytes()


def _mazda_cp_sp_bytes(CP, candidate) -> bytes:
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.mazda.interface import CarInterface
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  CP_SP = CarInterface.get_params_sp(CP, candidate, gen_empty_fingerprint(), [], alpha_long=False, is_release_sp=False, docs=False)
  return convert_to_capnp(CP_SP).to_bytes()


def _ltp_cache_bytes(version=1, valid=True) -> bytes:
  from openpilot.cereal import messaging
  msg = messaging.new_message('lateralTorqueParameters')
  ltp = msg.lateralTorqueParameters
  ltp.valid = valid
  ltp.version = version
  ltp.latAccelFactorFiltered = 2.6
  ltp.latAccelOffsetFiltered = 0.0
  ltp.frictionCoefficientFiltered = 0.12
  ltp.decay = 50.0
  ltp.points = [[0.1, -0.2]]
  return msg.to_bytes()


def _sp_cache_bytes(centers, seed_version=0, version=1) -> bytes:
  from openpilot.cereal import messaging
  msg = messaging.new_message('customReserved19')
  sp = msg.customReserved19
  sp.version = version
  sp.seedVersion = seed_version
  sp.speedBinCenters = centers
  sp.speedBinLatAccelFactors = [1.5] * len(centers)
  sp.speedBinFrictions = [0.1] * len(centers)
  return msg.to_bytes()


def _sha256(data: bytes) -> str:
  return hashlib.sha256(data).hexdigest()


class TestMazdaControlCacheMigration(OpenpilotTestCase):
  """Old-layout Mazda CP must never reach seeding/safety: v1 TI bit 4 reads as v2 LEGACY_FW_EPS
  and v1 legacy bit 8 as G46L_RADAR. The migration trusts only an exact digest+layout contract
  match, archives before any removal, and retains v1 TI learning only when every leg of the old
  restore contract checks out independently."""

  def _old_ti_install(self, params, candidate=None, with_caches=True):
    """A v1-branch TI garage state: old-layout blobs, interlock identity, matching caches."""
    from opendbc.car.mazda.values import CAR
    candidate = candidate or CAR.MAZDA_CX8_2022
    CP = _mazda_cp(candidate, V1_GEN1 | V1_STZ | V1_TI, min_steer_speed=0.0)
    cp_bytes = CP.to_bytes()
    params.put("CarParamsPersistent", cp_bytes, block=True)
    params.put("CarParamsSPPersistent", _mazda_cp_sp_bytes(CP, candidate), block=True)
    params.put("CarParamsPrevRoute", cp_bytes, block=True)
    params.put_bool("TorqueInterceptorEnabled", True, block=True)
    if with_caches:
      centers = [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]  # ti_legacy.MAZDA_CX8_2022, no floor drop at 0 min steer
      params.put("LiveTorqueParameters", _ltp_cache_bytes(), block=True)
      params.put("LiveTorqueParametersSP", _sp_cache_bytes(centers, seed_version=0), block=True)
    return cp_bytes

  def test_old_ti_bit4_learning_retained(self):
    params = Params()
    cp_bytes = self._old_ti_install(params)

    migrate_mazda_control_cache(params)

    assert params.get("CarParamsPersistent") is None
    assert params.get("CarParamsSPPersistent") is None
    assert params.get("CarParamsPrevRoute") == cp_bytes
    assert params.get("LiveTorqueParameters") is not None
    assert params.get("LiveTorqueParametersSP") is not None
    contract = _read_contract(params)
    assert contract["profile"] == "ti-legacy-v1"
    assert contract["blobs"]["CarParamsPersistent"] == {"sha256": None, "layout": None}
    assert contract["blobs"]["CarParamsSPPersistent"] == {"sha256": None, "layout": None}
    assert contract["blobs"]["CarParamsPrevRoute"] == {"sha256": _sha256(cp_bytes), "layout": "mazda-flags-v1"}

  def test_retained_learning_survives_card_stamp_and_second_boot(self):
    from opendbc.car.mazda.values import CAR
    params = Params()
    cp_bytes = self._old_ti_install(params)
    ltp_bytes = params.get("LiveTorqueParameters")
    migrate_mazda_control_cache(params)

    # card: fresh v2 CP/SP written (persistent was empty, so PrevRoute is not advanced), stamp last
    new_cp = _mazda_cp(CAR.MAZDA_CX8_2022, V1_GEN1 | V1_STZ | V2_TI, min_steer_speed=0.0)
    new_cp_bytes = new_cp.to_bytes()
    params.put("CarParamsPersistent", new_cp_bytes, block=True)
    params.put("CarParamsSPPersistent", _mazda_cp_sp_bytes(new_cp, CAR.MAZDA_CX8_2022), block=True)
    stamp_mazda_control_cache_contract(params, new_cp)

    contract = _read_contract(params)
    assert contract["profile"] == "ti-legacy-v1"
    assert contract["blobs"]["CarParamsPersistent"] == {"sha256": _sha256(new_cp_bytes), "layout": "mazda-flags-v2"}
    # the retained historical blob keeps its v1 provenance through the stamp
    assert contract["blobs"]["CarParamsPrevRoute"] == {"sha256": _sha256(cp_bytes), "layout": "mazda-flags-v1"}

    # second boot: trusted contract, nothing moves, no new archive generation
    migrate_mazda_control_cache(params)
    assert params.get("CarParamsPersistent") == new_cp_bytes
    assert params.get("CarParamsPrevRoute") == cp_bytes
    assert params.get("LiveTorqueParameters") == ltp_bytes
    assert len(_read_archive(params)["generations"]) == 1

  def test_old_legacy_bit8_cache_quarantined(self):
    """v1 legacy-EPS native (bit 8) reads as a v2 radar dialect: archive and quarantine."""
    from opendbc.car.mazda.values import CAR
    params = Params()
    CP = _mazda_cp(CAR.MAZDA_CX5, V1_GEN1 | V1_LEGACY)
    cp_bytes = CP.to_bytes()
    params.put("CarParamsPersistent", cp_bytes, block=True)
    params.put("CarParamsSPPersistent", _mazda_cp_sp_bytes(CP, CAR.MAZDA_CX5), block=True)
    params.put("CarParamsPrevRoute", cp_bytes, block=True)
    params.put("LiveTorqueParameters", _ltp_cache_bytes(), block=True)
    params.put("LiveTorqueParametersSP", _sp_cache_bytes([6.5, 10.0, 15.0, 21.0, 26.5, 32.0, 37.5]), block=True)

    migrate_mazda_control_cache(params)

    assert params.get("CarParamsPersistent") is None
    assert params.get("CarParamsSPPersistent") is None
    assert params.get("LiveTorqueParameters") is None
    assert params.get("LiveTorqueParametersSP") is None
    assert params.get("CarParamsPrevRoute") == cp_bytes  # parseable: kept for fingerprint-gated readers
    contract = _read_contract(params)
    assert contract["profile"] == "native-v2"
    assert contract["blobs"]["CarParamsPrevRoute"]["layout"] == "mazda-flags-v1"

  def test_ti_cache_with_wrong_seed_layout_quarantined(self):
    """TI identity holds, but the fork cache's bins match no ported ti_legacy layout."""
    params = Params()
    self._old_ti_install(params)
    params.put("LiveTorqueParametersSP", _sp_cache_bytes([3.0, 6.0], seed_version=0), block=True)

    migrate_mazda_control_cache(params)

    assert params.get("LiveTorqueParameters") is None
    assert params.get("LiveTorqueParametersSP") is None

  def test_ti_retention_requires_matching_prev_route(self):
    """torqued restores against CarParamsPrevRoute: a different fingerprint there breaks the
    restore key, so the learning cannot be called continuous."""
    from opendbc.car.mazda.values import CAR
    params = Params()
    self._old_ti_install(params)
    params.put("CarParamsPrevRoute", _mazda_cp_bytes(CAR.MAZDA_CX9_2021, V1_GEN1 | V1_TI | V1_LEGACY), block=True)

    migrate_mazda_control_cache(params)

    assert params.get("LiveTorqueParameters") is None
    assert params.get("LiveTorqueParametersSP") is None

  def test_malformed_cp_fails_closed(self):
    params = Params()
    garbage = b"\x00\x01not-a-carparams"
    params.put("CarParamsPersistent", garbage, block=True)
    params.put("LiveTorqueParameters", _ltp_cache_bytes(), block=True)

    migrate_mazda_control_cache(params)

    assert params.get("CarParamsPersistent") is None
    assert params.get("LiveTorqueParameters") is None
    contract = _read_contract(params)
    assert contract["profile"] is None
    assert all(record == {"sha256": None, "layout": None} for record in contract["blobs"].values())
    gen = _read_archive(params)["generations"][0]["keys"]
    assert base64.b64decode(gen["CarParamsPersistent"]["data"]) == garbage

  def test_missing_contract_new_layout_quarantined(self):
    """A current-layout CP with no contract has no provenance; fail closed rather than guess."""
    from opendbc.car.mazda.values import CAR, MazdaFlags
    params = Params()
    CP = _mazda_cp(CAR.MAZDA_CX5, int(MazdaFlags.GEN1 | MazdaFlags.LEGACY_FW_EPS))
    params.put("CarParamsPersistent", CP.to_bytes(), block=True)
    params.put("LiveTorqueParameters", _ltp_cache_bytes(), block=True)

    migrate_mazda_control_cache(params)

    assert params.get("CarParamsPersistent") is None
    assert params.get("LiveTorqueParameters") is None

  def test_conflicting_bundle_fails_closed_and_bundle_preserved(self):
    from opendbc.car.mazda.values import CAR
    params = Params()
    bundle = {"brand": "toyota", "platform": "TOYOTA_COROLLA", "name": "Corolla", "model": "Corolla"}
    params.put("CarPlatformBundle", bundle, block=True)
    params.put("CarParamsPersistent", _mazda_cp_bytes(CAR.MAZDA_CX5, V1_GEN1 | V1_TI | V1_STZ), block=True)

    migrate_mazda_control_cache(params)

    assert params.get("CarParamsPersistent") is None
    assert params.get("CarPlatformBundle") == bundle

  def test_empty_install_noop(self):
    params = Params()
    migrate_mazda_control_cache(params)
    assert not _contract_path(params).exists()
    assert not _archive_path(params).exists()

  def test_non_mazda_untouched(self):
    from opendbc.car.mazda.values import CAR
    params = Params()
    CP = _mazda_cp(CAR.MAZDA_CX5, V1_GEN1 | V1_TI)  # old bits set: would be quarantined if read as Mazda
    CP.brand = "toyota"
    CP.carFingerprint = "TOYOTA_COROLLA"
    cp_bytes = CP.to_bytes()
    ltp_bytes = _ltp_cache_bytes()
    params.put("CarParamsPersistent", cp_bytes, block=True)
    params.put("CarParamsPrevRoute", cp_bytes, block=True)
    params.put("LiveTorqueParameters", ltp_bytes, block=True)

    migrate_mazda_control_cache(params)

    assert params.get("CarParamsPersistent") == cp_bytes
    assert params.get("CarParamsPrevRoute") == cp_bytes
    assert params.get("LiveTorqueParameters") == ltp_bytes
    assert not _contract_path(params).exists()
    assert not _archive_path(params).exists()

  def test_archive_round_trip_including_absent_keys(self):
    from opendbc.car.mazda.values import CAR
    params = Params()
    CP = _mazda_cp(CAR.MAZDA_CX5, V1_GEN1 | V1_LEGACY)
    cp_bytes = CP.to_bytes()
    sp_bytes = _mazda_cp_sp_bytes(CP, CAR.MAZDA_CX5)
    ltp_bytes = _ltp_cache_bytes()
    delay_bytes = b"delay-blob"
    params.put("CarParamsPersistent", cp_bytes, block=True)
    params.put("CarParamsSPPersistent", sp_bytes, block=True)
    params.put("CarParamsPrevRoute", cp_bytes, block=True)
    params.put("LiveTorqueParameters", ltp_bytes, block=True)
    params.put("LiveDelay", delay_bytes, block=True)
    # LiveTorqueParametersSP and LiveParametersV2 stay absent

    migrate_mazda_control_cache(params)

    keys = _read_archive(params)["generations"][0]["keys"]
    assert set(keys) == set(ARCHIVE_KEYS)
    for name, original in (("CarParamsPersistent", cp_bytes), ("CarParamsSPPersistent", sp_bytes),
                           ("CarParamsPrevRoute", cp_bytes), ("LiveTorqueParameters", ltp_bytes),
                           ("LiveDelay", delay_bytes)):
      assert keys[name]["sha256"] == _sha256(original)
      assert base64.b64decode(keys[name]["data"]) == original  # byte-for-byte recovery
    assert keys["LiveTorqueParametersSP"] is None
    assert keys["LiveParametersV2"] is None

  def test_malformed_archive_aborts_before_removal(self):
    params = Params()
    self._old_ti_install(params)
    _archive_path(params).write_text("[]")

    with pytest.raises(RuntimeError, match="malformed Mazda cache archive"):
      migrate_mazda_control_cache(params)

    assert params.get("CarParamsPersistent") is not None
    assert params.get("LiveTorqueParameters") is not None

  def test_interrupted_card_stamp_requarantines(self):
    """Migration retained v1 learning, card wrote fresh v2 CP/SP but died before the stamp:
    the stale contract mismatches and the next boot quarantines again, keeping both archives."""
    from opendbc.car.mazda.values import CAR
    params = Params()
    self._old_ti_install(params)
    migrate_mazda_control_cache(params)
    assert params.get("LiveTorqueParameters") is not None

    new_cp = _mazda_cp(CAR.MAZDA_CX8_2022, V1_GEN1 | V1_STZ | V2_TI, min_steer_speed=0.0)
    params.put("CarParamsPersistent", new_cp.to_bytes(), block=True)
    params.put("CarParamsSPPersistent", _mazda_cp_sp_bytes(new_cp, CAR.MAZDA_CX8_2022), block=True)
    # no stamp: crash between the persistent writes and the contract write

    migrate_mazda_control_cache(params)

    assert params.get("CarParamsPersistent") is None
    assert params.get("CarParamsSPPersistent") is None
    assert params.get("LiveTorqueParameters") is None  # fail closed: retained learning does not survive a broken stamp
    generations = _read_archive(params)["generations"]
    assert len(generations) == 2
    assert generations[0]["keys"]["CarParamsPersistent"] is not None  # first generation never overwritten

  def test_old_clear_all_preserves_external_archive(self):
    """Old-branch Params::clearAll deletes every unknown file under the params dir; the archive
    and contract live beside it and must survive a downgrade."""
    params = Params()
    self._old_ti_install(params)
    migrate_mazda_control_cache(params)

    unknown_marker = Path(params.get_param_path()) / "SomeLegacyMarker"
    unknown_marker.write_text("x")
    params.clear_all(ParamKeyFlag.ALL)

    assert not unknown_marker.exists()
    assert params.get("CarParamsPrevRoute") is None  # every known key wiped
    assert json.loads(_archive_path(params).read_text())["generations"]  # external files survive
    assert json.loads(_contract_path(params).read_text())["version"] == 2

  def test_downgrade_reupgrade_digest_mismatch(self):
    """v1 branch rewrites the persistent CP under the old layout; the surviving contract's v2
    digests mismatch on re-upgrade and the set is quarantined again."""
    from opendbc.car.mazda.values import CAR
    params = Params()
    CP = _mazda_cp(CAR.MAZDA_CX5, V1_GEN1 | V1_STZ | V2_TI)
    params.put("CarParamsPersistent", CP.to_bytes(), block=True)
    stamp_mazda_control_cache_contract(params, CP)
    assert params.get("CarParamsPersistent") is not None

    # downgrade: clearAll wipes the params dir, old branch writes a v1-layout CP
    params.clear_all(ParamKeyFlag.ALL)
    params.put("CarParamsPersistent", _mazda_cp_bytes(CAR.MAZDA_CX5, V1_GEN1 | V1_STZ | V1_TI), block=True)
    params.put_bool("TorqueInterceptorEnabled", True, block=True)

    migrate_mazda_control_cache(params)

    assert params.get("CarParamsPersistent") is None
    assert _read_archive(params)["generations"]

  def test_second_boot_noop(self):
    from opendbc.car.mazda.values import CAR
    params = Params()
    CP = _mazda_cp(CAR.MAZDA_CX5, V1_GEN1 | V1_STZ | V2_TI)
    cp_bytes = CP.to_bytes()
    params.put("CarParamsPersistent", cp_bytes, block=True)
    params.put("CarParamsSPPersistent", _mazda_cp_sp_bytes(CP, CAR.MAZDA_CX5), block=True)
    ltp_bytes = _ltp_cache_bytes()
    params.put("LiveTorqueParameters", ltp_bytes, block=True)
    stamp_mazda_control_cache_contract(params, CP)
    contract_bytes = _contract_path(params).read_bytes()

    migrate_mazda_control_cache(params)

    assert _contract_path(params).read_bytes() == contract_bytes  # untouched
    assert not _archive_path(params).exists()
    assert params.get("CarParamsPersistent") == cp_bytes
    assert params.get("LiveTorqueParameters") == ltp_bytes

  def test_card_stamp_non_mazda_writes_nothing(self):
    from opendbc.car.mazda.values import CAR
    params = Params()
    CP = _mazda_cp(CAR.MAZDA_CX5, 0)
    CP.brand = "toyota"
    stamp_mazda_control_cache_contract(params, CP)
    assert not _contract_path(params).exists()

  def test_user_settings_untouched(self):
    params = Params()
    bundle = {"brand": "mazda", "platform": "MAZDA_CX5", "name": "CX-5", "model": "CX-5"}
    params.put("CarPlatformBundle", bundle, block=True)
    params.put("TorqueControlTune", 1.7, block=True)
    params.put("MazdaTorqueTuneSeeded", 2.0, block=True)
    params.put_bool("EnforceTorqueControl", True)
    params.put_bool("IsMetric", True)
    params.put("CalibrationParams", b"calib", block=True)
    self._old_ti_install(params)

    migrate_mazda_control_cache(params)

    assert params.get("CarPlatformBundle") == bundle
    assert params.get("TorqueControlTune") == 1.7
    assert params.get("MazdaTorqueTuneSeeded") == 2.0
    assert params.get_bool("EnforceTorqueControl")
    assert params.get_bool("IsMetric")
    assert params.get_bool("TorqueInterceptorEnabled")  # TI/interlock identity preserved
    assert params.get("CalibrationParams") == b"calib"

  def test_manager_ordering_clear_then_migrate_then_seed(self):
    """The manager_init sequence without registration: transient CP keys are cleared before
    migrations, so the migration never archives them, and seeding finds no old-layout CP."""
    from opendbc.car.mazda.values import CAR
    from openpilot.sunnypilot.selfdrive.car.interfaces import seed_car_defaults_offroad
    params = Params()
    CP = _mazda_cp(CAR.MAZDA_CX5, V1_GEN1 | V1_LEGACY)
    old_bytes = CP.to_bytes()
    params.put("CarParamsPersistent", old_bytes, block=True)
    params.put("CarParamsPrevRoute", old_bytes, block=True)
    params.put("LiveTorqueParameters", _ltp_cache_bytes(), block=True)
    params.put("CarParams", old_bytes, block=True)       # transient: old branch's live keys
    params.put("CarParamsCache", old_bytes, block=True)

    for flag in (ParamKeyFlag.CLEAR_ON_MANAGER_START, ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION,
                 ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION, ParamKeyFlag.CLEAR_ON_IGNITION_ON):
      params.clear_all(flag)
    run_migration(params)
    seed_car_defaults_offroad(params)

    assert params.get("CarParams") is None and params.get("CarParamsCache") is None
    assert params.get("CarParamsPersistent") is None  # quarantined before seeding
    assert set(_read_archive(params)["generations"][0]["keys"]) == set(ARCHIVE_KEYS)  # no transient keys archived
    assert params.get("MazdaTorqueTuneSeeded") is None  # seed inert without a persistent CP
    assert not params.get_bool("EnforceTorqueControl")
    assert params.get("TorqueControlTune") is None
    assert params.get("OnroadScreenOffBrightnessMigrated") == "1.0"  # later migrations still ran

  def test_migration_failure_blocks_remaining_migrations(self, monkeypatch):
    """A failed archive/write raises out of run_migration before any later migration runs;
    manager_init therefore never reaches seeding or registration."""
    def boom(*args, **kwargs):
      raise RuntimeError("disk full")
    monkeypatch.setattr(params_migration, "_atomic_write_json", boom)
    params = Params()
    self._old_ti_install(params)

    with pytest.raises(RuntimeError, match="disk full"):
      run_migration(params)

    assert params.get("OnroadScreenOffBrightnessMigrated") is None
    assert params.get("CarParamsPersistent") is not None  # destructive phase never started

