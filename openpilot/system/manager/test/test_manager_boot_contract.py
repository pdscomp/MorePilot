#!/usr/bin/env python3
"""Boot contract for manager_init(): the real migration -> offroad seed -> default
materialization -> finalize_ti_enable -> enforce_backend_state -> registration sequence,
with only the external boundaries (build metadata, hardware identity, registration,
logging, shm path) mocked. Assertions execute inside the registration call site,
so no later repair in manager_init can fake the state they pin. manager.main() and
managed processes never run; no network or device calls happen."""
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.car.structs import car

from openpilot.common.api.backend import backend_config, is_konik_locked, lock_sentinel
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
import openpilot.system.manager.manager as manager
from openpilot.sunnypilot.system import params_migration

# manager_init exports these; the fixture restores them so tests stay isolated
BOOT_ENV_KEYS = ("DONGLE_ID", "GIT_ORIGIN", "GIT_BRANCH", "GIT_COMMIT", "CLEAN")


def _meta_dir(params) -> Path:
  return Path(params.get_param_path()).parent


def _archive_path(params) -> Path:
  return _meta_dir(params) / ".mazda_control_cache_archive.json"


def _contract_path(params) -> Path:
  return _meta_dir(params) / ".mazda_control_cache_contract.json"


@pytest.fixture
def boot(monkeypatch, tmp_path):
  env_snapshot = {key: os.environ.get(key) for key in BOOT_ENV_KEYS}
  with OpenpilotPrefix():
    params = Params()
    build = SimpleNamespace(
      channel="unit-test", development_channel=True, tested_channel=False,
      release_channel=False, release_sp_channel=False,
      openpilot=SimpleNamespace(version="test", git_commit="candidate-test",
                                git_commit_date="test", git_origin="test",
                                git_normalized_origin="test", is_dirty=True),
    )
    monkeypatch.setattr(manager, "PC", False)  # migration/seeding are gated on not-PC
    monkeypatch.setattr(manager, "Params", lambda: params)
    monkeypatch.setattr(manager, "save_bootlog", lambda: None)
    monkeypatch.setattr(manager, "get_build_metadata", lambda: build)
    monkeypatch.setattr(manager, "HARDWARE", SimpleNamespace(
      get_serial=lambda: "test-serial", get_device_type=lambda: "pc"))
    monkeypatch.setattr(manager, "Paths", SimpleNamespace(shm_path=lambda: str(tmp_path)))
    monkeypatch.setattr(manager.cloudlog, "bind_global", lambda **kwargs: None)
    monkeypatch.setattr(manager, "register", lambda **kwargs: pytest.fail("unexpected registration"))
    # an old-branch garage: quick boot off, konik not yet locked, TI enable staged,
    # an explicit user tune, and the commit the old branch last wrote
    params.put_bool("QuickBootToggle", False, block=True)
    params.put_bool("UseKonikServer", False, block=True)
    params.put_bool("TorqueInterceptorEnableRequest", True, block=True)
    params.put("TorqueControlTune", 1.0, block=True)
    params.put("GitCommit", "40785dfd67f048f439100fbfd6608114760aca89", block=True)
    try:
      yield params
    finally:
      for key, value in env_snapshot.items():
        if value is None:
          os.environ.pop(key, None)
        else:
          os.environ[key] = value


@pytest.mark.parametrize("old_hardware_bit", [4, 8])  # v1 TORQUE_INTERCEPTOR / v1 LEGACY_FW_EPS
def test_real_migration_and_interlock_precede_registration(boot, monkeypatch, old_hardware_bit):
  cp = car.CarParams.new_message()
  cp.brand = "mazda"
  cp.carFingerprint = "MAZDA_CX5"
  cp.flags = 1 | old_hardware_bit  # v1 GEN1 plus a v1-only bit; unreadable under the v2 layout
  old_bytes = cp.to_bytes()
  boot.put("CarParamsPersistent", old_bytes, block=True)
  boot.put("CarParams", old_bytes, block=True)       # old branch's transient live keys
  boot.put("CarParamsCache", old_bytes, block=True)
  boot.put("LiveTorqueParameters", b"old-torque-cache", block=True)
  seen = []
  real_seed = manager.seed_car_defaults_offroad

  def seed_after_migration(params):
    # transient CP keys cleared by manager, old-layout persistent CP quarantined by the
    # migration — both before seeding can read either one
    assert params.get("CarParams") is None
    assert params.get("CarParamsCache") is None
    assert params.get("CarParamsPersistent") is None
    assert _contract_path(params).is_file()
    assert _archive_path(params).is_file()
    seen.append("seed")
    return real_seed(params)

  def register_after_interlock(**kwargs):
    assert kwargs == {"show_spinner": True}
    assert seen == ["seed"]
    # the quarantine reached registration: no old-layout blob or cache survives
    assert boot.get("CarParamsPersistent") is None
    assert boot.get("CarParamsSPPersistent") is None
    assert boot.get("CarParams") is None
    assert boot.get("CarParamsCache") is None
    assert boot.get("LiveTorqueParameters") is None
    assert len(json.loads(_archive_path(boot).read_text())["generations"]) == 1
    assert _contract_path(boot).is_file()
    # finalize_ti_enable and enforce_backend_state ran for real, before registration
    assert boot.get_bool("TorqueInterceptorEnabled")
    assert not boot.get_bool("TorqueInterceptorEnableRequest")
    assert boot.get_bool("KonikLockout")
    assert boot.get_bool("KonikInterlock")
    assert boot.get_bool("UseKonikServer")
    assert lock_sentinel(boot).exists()
    assert is_konik_locked(boot)
    assert backend_config(boot).name == "konik"
    # explicit user choice preserved; default materialization really ran
    assert boot.get("TorqueControlTune") == 1.0
    assert boot.get("RouteCount") == 0
    seen.append("register")
    return "unit-test-dongle"

  monkeypatch.setattr(manager, "seed_car_defaults_offroad", seed_after_migration)
  monkeypatch.setattr(manager, "register", register_after_interlock)
  manager.manager_init()
  assert seen == ["seed", "register"]


def test_migration_failure_prevents_seeding_and_registration(boot, monkeypatch):
  def failed_migration(params):
    raise OSError("simulated migration persistence failure")

  monkeypatch.setattr(params_migration, "migrate_mazda_control_cache", failed_migration)
  monkeypatch.setattr(manager, "seed_car_defaults_offroad",
                      lambda params: pytest.fail("seed ran after failed migration"))
  # the fixture's register stub fails the test if any registration/network attempt happens
  with pytest.raises(OSError, match="migration persistence failure"):
    manager.manager_init()

  # nothing downstream of the migration ran
  assert boot.get("OnroadScreenOffBrightnessMigrated") is None
  assert not _contract_path(boot).exists()
  assert not _archive_path(boot).exists()
  assert not lock_sentinel(boot).exists()
