"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import base64
import datetime
import hashlib
import json
import os

from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.sync_sunnylink_params import CAR_LIST_JSON_OUT

ONROAD_BRIGHTNESS_MIGRATION_VERSION: str = "1.0"
ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION: str = "1.0"

# index → seconds mapping for OnroadScreenOffTimer (SSoT)
ONROAD_BRIGHTNESS_TIMER_VALUES = {0: 3, 1: 5, 2: 7, 3: 10, 4: 15, 5: 30, **{i: (i - 5) * 60 for i in range(6, 16)}}
VALID_TIMER_VALUES = set(ONROAD_BRIGHTNESS_TIMER_VALUES.values())


def _resolve_brand(_params) -> str:
  bundle = _params.get("CarPlatformBundle")
  if isinstance(bundle, dict) and bundle.get("brand"):
    return str(bundle["brand"])

  # Auto-fingerprinted cars have no bundle, fall back to the last known CarParams.
  CP_bytes = _params.get("CarParamsPersistent")
  if CP_bytes is None:
    return ""

  # Never raises: callers rely on "" to mean "brand unknown, skip the migration".
  try:
    from openpilot.cereal import messaging  # lazy: avoids heavy import at module level
    from opendbc.car.structs import car
    return str(messaging.log_from_bytes(CP_bytes, car.CarParams).brand)
  except Exception as e:
    cloudlog.exception(f"params_migration: failed to resolve brand from CarParamsPersistent: {e}")
    return ""


def _migrate_car_platform_bundle(_params):
  bundle = _params.get("CarPlatformBundle")
  if bundle is None:
    return

  old_platform = bundle.get("platform")
  if not old_platform:
    return

  from opendbc.car.fingerprints import MIGRATION  # lazy: avoids heavy import at module level
  if old_platform not in MIGRATION:
    return

  new_platform = str(MIGRATION[old_platform])

  with open(CAR_LIST_JSON_OUT) as f:
    car_list = json.load(f)

  candidates = [(k, v) for k, v in car_list.items() if v.get("platform") == new_platform]
  if candidates:
    old_model = bundle.get("model")
    key, data = next(((k, v) for k, v in candidates if v.get("model") == old_model), candidates[0])
    bundle = {**data, "name": key}
  else:
    bundle["platform"] = new_platform

  _params.put("CarPlatformBundle", bundle, block=True)
  cloudlog.info(f"params_migration: CarPlatformBundle migrated {old_platform!r} -> {new_platform!r}")


def _migrate_tesla_mads_screen_button(_params):
  # TeslaMadsScreenButton defaults to Off for fresh installs, but the screen button was previously always
  # active on Teslas with a vehicle bus. Seed existing Tesla installs with 3-finger to preserve that.
  try:
    if _params.get("TeslaMadsScreenButton") is not None:
      return

    if _resolve_brand(_params) != "tesla":
      return

    from opendbc.sunnypilot.car.tesla.values import MadsScreenButtonType  # lazy: avoids heavy import at module level
    _params.put("TeslaMadsScreenButton", MadsScreenButtonType.THREE_FINGER, block=True)
    cloudlog.info("params_migration: seeded TeslaMadsScreenButton with 3-finger to preserve existing behavior")
  except Exception as e:
    cloudlog.exception(f"Error migrating TeslaMadsScreenButton: {e}")


def _migrate_model_bundle_slots(_params):
  # Pre-split, a chestnut user's big-model selection lived in the single
  # ActiveBundle. Seed both slots; validation drops whichever does not match
  # its own manifest.
  try:
    if _params.get("ModelManager_ActiveBundleChestnut") is not None:
      return
    if (chestnut_bundle := _params.get("ModelManager_ActiveBundleUSBGPU")) is not None:
      _params.put("ModelManager_ActiveBundleChestnut", chestnut_bundle, block=True)
      cloudlog.info("params_migration: seeded ModelManager_ActiveBundleChestnut from ModelManager_ActiveBundleUSBGPU")
      return
    if (bundle := _params.get("ModelManager_ActiveBundle")) is None:
      return
    _params.put("ModelManager_ActiveBundleChestnut", bundle, block=True)
    cloudlog.info("params_migration: seeded ModelManager_ActiveBundleChestnut from ModelManager_ActiveBundle")
  except Exception as e:
    cloudlog.exception(f"Error migrating model bundle slots: {e}")


# --- Mazda control-cache migration -------------------------------------------
#
# The Mazda flags renumbering (v1: TORQUE_INTERCEPTOR=4, LEGACY_FW_EPS=8; v2: LEGACY_FW_EPS=4,
# G46L_RADAR=8, MOVING_TAKEOVER=16, TORQUE_INTERCEPTOR=32) makes every CarParams blob written
# by an older branch unreadable: old TI bit 4 parses as v2 legacy-EPS, old legacy bit 8 as a
# radar dialect. State lives in two JSON files beside the params dir (not under it, so the old
# branch's Params::clearAll cannot delete them on a downgrade): an append-only archive and the
# mutable contract. The contract records the SHA-256 of each persistent blob; only an exact
# digest+layout match is trusted. Anything else is archived, then persistent CP/SP are removed
# and torque caches quarantined before offroad seeding or safety can read them.

MAZDA_CACHE_LAYOUT_V1 = "mazda-flags-v1"
MAZDA_CACHE_LAYOUT_V2 = "mazda-flags-v2"
MAZDA_CACHE_PROFILE_NATIVE = "native-v2"
MAZDA_CACHE_PROFILE_TI_LEGACY = "ti-legacy-v1"

_MAZDA_CONTRACT_FILE = ".mazda_control_cache_contract.json"
_MAZDA_ARCHIVE_FILE = ".mazda_control_cache_archive.json"

# Persistent blobs an old-layout cache can poison, archived (present or absent) before any
# removal. CarParams/CarParamsCache/CarParamsSP/CarParamsSPCache are transient: manager clears
# them before migrations run, so they are never part of the archive.
_MAZDA_CACHE_ARCHIVE_KEYS = ("CarParamsPersistent", "CarParamsSPPersistent", "CarParamsPrevRoute",
                             "LiveTorqueParameters", "LiveTorqueParametersSP", "LiveParametersV2", "LiveDelay")
_MAZDA_PERSISTENT_CP_KEYS = ("CarParamsPersistent", "CarParamsSPPersistent")
_MAZDA_TORQUE_CACHE_KEYS = ("LiveTorqueParameters", "LiveTorqueParametersSP")

_MAZDA_V1_TI_BIT = 0x4      # v1 TORQUE_INTERCEPTOR; the same bit is LEGACY_FW_EPS under v2
_MAZDA_V2_ONLY_BITS = 0x30  # MOVING_TAKEOVER | TORQUE_INTERCEPTOR, both unallocated under v1


def _mazda_meta_dir(params) -> Path:
  return Path(params.get_param_path()).parent


def _atomic_write_json(path: Path, obj) -> None:
  """Same-dir temp write, fsync of file and directory, replace, then read-back verify."""
  data = json.dumps(obj, sort_keys=True).encode()
  tmp = path.with_name(path.name + ".tmp")
  with open(tmp, "wb") as f:
    f.write(data)
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, path)
  dir_fd = os.open(path.parent, os.O_RDONLY)
  try:
    os.fsync(dir_fd)
  finally:
    os.close(dir_fd)
  if path.read_bytes() != data:
    raise RuntimeError(f"params_migration: read-back verification failed for {path}")


def _read_json_file(path: Path):
  try:
    with open(path) as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


def _sha256(data: bytes) -> str:
  return hashlib.sha256(data).hexdigest()


def _blob_record(data: bytes | None, layout: str | None) -> dict:
  return {"sha256": None if data is None else _sha256(data), "layout": None if data is None else layout}


def _write_mazda_cache_contract(meta_dir: Path, params, profile: str | None, prev_layout: str | None) -> None:
  blobs = {key: _blob_record(params.get(key), MAZDA_CACHE_LAYOUT_V2) for key in _MAZDA_PERSISTENT_CP_KEYS}
  blobs["CarParamsPrevRoute"] = _blob_record(params.get("CarParamsPrevRoute"), prev_layout)
  _atomic_write_json(meta_dir / _MAZDA_CONTRACT_FILE, {"version": 2, "profile": profile, "blobs": blobs})


def _mazda_contract_matches(contract, params) -> bool:
  """Trusted only when every recorded digest matches the current bytes and each present blob
  carries a known layout. A missing or stale contract detects interrupted card writes and
  downgrade/re-upgrade without trusting GitCommit, which manager rewrites independently."""
  if not isinstance(contract, dict) or contract.get("version") != 2:
    return False
  profile = contract.get("profile")
  if profile not in (MAZDA_CACHE_PROFILE_NATIVE, MAZDA_CACHE_PROFILE_TI_LEGACY, None):
    return False
  blobs = contract.get("blobs")
  if not isinstance(blobs, dict):
    return False
  live_digests = 0
  for key in ("CarParamsPersistent", "CarParamsSPPersistent", "CarParamsPrevRoute"):
    record = blobs.get(key)
    if not isinstance(record, dict):
      return False
    data = params.get(key)
    if data is None:
      if record.get("sha256") is not None or record.get("layout") is not None:
        return False
      continue
    if record.get("sha256") != _sha256(data):
      return False
    live_digests += 1
    layout = record.get("layout")
    if key == "CarParamsPrevRoute":
      if layout not in (MAZDA_CACHE_LAYOUT_V1, MAZDA_CACHE_LAYOUT_V2):
        return False
    elif layout != MAZDA_CACHE_LAYOUT_V2:
      return False
  # a null profile is only the recorded state of a fully quarantined unknown set
  return profile is not None or live_digests == 0


def _parse_car_params(data: bytes | None):
  """The stable CarParams fields as a dict, None when absent, False when malformed. A message
  without a brand never came from get_params and counts as malformed. Flags are never read here."""
  if data is None:
    return None
  try:
    from openpilot.cereal import messaging  # lazy: keeps manager_init's import cost down
    from opendbc.car.structs import car
    CP = messaging.log_from_bytes(data, car.CarParams)
    if not CP.brand:
      return False
    tuning = CP.lateralTuning.which()
    return {
      "brand": str(CP.brand), "fingerprint": str(CP.carFingerprint), "flags": int(CP.flags),
      "min_steer_speed": float(CP.minSteerSpeed), "tuning": tuning,
      "friction": float(CP.lateralTuning.torque.friction) if tuning == "torque" else None,
      "lat_accel_factor": float(CP.lateralTuning.torque.latAccelFactor) if tuning == "torque" else None,
    }
  except Exception:
    return False


def _is_mazda_cp(cp: dict) -> bool:
  # schema-stable identity only, never flags
  return cp["brand"] == "mazda" or cp["fingerprint"].startswith("MAZDA")


def _ti_legacy_seed_layout(cp: dict) -> tuple[int, list]:
  """(seed_version, bin centers) expected for this car from the ported ti_legacy seed table,
  with the same steering-floor bin drop as get_speed_dep_config_for_car."""
  from opendbc.sunnypilot.car.interfaces import get_ti_legacy_speed_dep_config  # lazy
  from openpilot.sunnypilot.selfdrive.locationd.speed_bin_learner import DEFAULT_SPEED_BIN_CENTERS  # lazy
  cfg = get_ti_legacy_speed_dep_config().get(cp["fingerprint"], {})
  centers = list(cfg.get("speed_bp", []))
  if centers and cp["min_steer_speed"] > 0:
    centers = [c for c in centers if c >= cp["min_steer_speed"]]
  if not centers:
    centers = list(DEFAULT_SPEED_BIN_CENTERS)
  return int(cfg.get("seed_version", 0)), centers


def _ti_legacy_identity(params, cp: dict) -> bool:
  """TI install on a v1-layout CP, established independently of the blob: the interlock param
  (only ever set via enable_ti), the v1 TI bit with no v2-only bits, and torque tune units."""
  if not params.get_bool("TorqueInterceptorEnabled"):
    return False
  if cp["flags"] & _MAZDA_V2_ONLY_BITS or not cp["flags"] & _MAZDA_V1_TI_BIT:
    return False
  return cp["tuning"] == "torque"


def _ti_legacy_learning_verified(params, cp: dict, prev_cp: dict | None) -> bool:
  """Whether the v1 torque caches may stay. Every leg of the old production restore contract is
  checked independently: TI identity, restore-key continuity between the two persisted CP blobs
  (torqued restores against CarParamsPrevRoute), and current-version parseable caches whose
  seed/centers match the ported ti_legacy table. Any failure quarantines the caches instead."""
  if not _ti_legacy_identity(params, cp):
    return False
  if prev_cp is None or prev_cp["fingerprint"] != cp["fingerprint"] or prev_cp["tuning"] != "torque" or \
     (prev_cp["friction"], prev_cp["lat_accel_factor"]) != (cp["friction"], cp["lat_accel_factor"]):
    return False

  from openpilot.cereal import log  # lazy
  from openpilot.selfdrive.locationd.torqued import VERSION  # lazy
  ltp = params.get("LiveTorqueParameters")
  if ltp is None:
    return False  # no learning to retain
  try:
    with log.Event.from_bytes(ltp) as evt:
      if evt.lateralTorqueParameters.version != VERSION:
        return False
  except Exception:
    return False

  sp_cache = params.get("LiveTorqueParametersSP")
  if sp_cache is not None:
    try:
      from openpilot.sunnypilot.selfdrive.locationd.speed_bin_learner import LIVE_TORQUE_PARAMETERS_SP_SERVICE  # lazy
      with log.Event.from_bytes(sp_cache) as evt:
        cache_sp = getattr(evt, LIVE_TORQUE_PARAMETERS_SP_SERVICE)
        seed_version, centers = _ti_legacy_seed_layout(cp)
        cached_centers = list(cache_sp.speedBinCenters)
        if cache_sp.version != VERSION or int(cache_sp.seedVersion) != seed_version or \
           len(cached_centers) != len(centers) or any(abs(a - b) > 0.01 for a, b in zip(cached_centers, centers, strict=False)) or \
           len(cache_sp.speedBinLatAccelFactors) != len(centers) or len(cache_sp.speedBinFrictions) != len(centers):
          return False
    except Exception:
      return False
  return True


def _archive_mazda_control_cache(meta_dir: Path, params, reason: str) -> None:
  """Append one immutable generation recording every persistent original, present or absent.
  An unreadable archive aborts the migration rather than risking the sole recovery copy."""
  path = meta_dir / _MAZDA_ARCHIVE_FILE
  archive = _read_json_file(path)
  if archive is None:
    if path.exists():
      raise RuntimeError(f"params_migration: unreadable Mazda cache archive at {path}")
    generations = []
  else:
    if not isinstance(archive, dict):
      raise RuntimeError(f"params_migration: malformed Mazda cache archive at {path}")
    generations = archive.get("generations")
    if not isinstance(generations, list):
      raise RuntimeError(f"params_migration: malformed Mazda cache archive at {path}")
  keys = {}
  for key in _MAZDA_CACHE_ARCHIVE_KEYS:
    data = params.get(key)
    keys[key] = None if data is None else {"sha256": _sha256(data), "data": base64.b64encode(data).decode()}
  generations.append({"reason": reason, "time": datetime.datetime.now(datetime.UTC).isoformat(), "keys": keys})
  _atomic_write_json(path, {"generations": generations})


def migrate_mazda_control_cache(params) -> None:
  """Quarantine Mazda CarParams/torque caches written under the historical flags layout before
  offroad seeding or safety can read them. CarPlatformBundle, TI/interlock/backend identity,
  calibration and user settings are preserved. Raises on any failure so manager_init never
  reaches seeding or registration with untrusted cache state."""
  meta_dir = _mazda_meta_dir(params)
  contract = _read_json_file(meta_dir / _MAZDA_CONTRACT_FILE)
  if _mazda_contract_matches(contract, params):
    return  # trusted: unchanged since the last stamp

  cp = _parse_car_params(params.get("CarParamsPersistent"))
  prev_cp = _parse_car_params(params.get("CarParamsPrevRoute"))
  bundle = params.get("CarPlatformBundle")
  bundle_mazda = None
  if isinstance(bundle, dict):
    bundle_mazda = str(bundle.get("brand") or "") == "mazda" or str(bundle.get("platform") or "").startswith("MAZDA")

  if cp is None and prev_cp is None and bundle is None and \
     all(params.get(key) is None for key in _MAZDA_CACHE_ARCHIVE_KEYS):
    return  # empty install: await the normal fresh-card path

  votes = [v for v in (_is_mazda_cp(cp) if isinstance(cp, dict) else None,
                       _is_mazda_cp(prev_cp) if isinstance(prev_cp, dict) else None,
                       bundle_mazda) if v is not None]
  if cp is not False and prev_cp is not False and len(set(votes)) <= 1 and votes and not any(votes):
    return  # consistently parseable non-Mazda: untouched

  # untrusted Mazda or unknown: archive first, then drop what cannot be trusted
  retain_ti = isinstance(cp, dict) and _is_mazda_cp(cp) and \
              _ti_legacy_learning_verified(params, cp, prev_cp if isinstance(prev_cp, dict) else None)
  profile = MAZDA_CACHE_PROFILE_TI_LEGACY if retain_ti else (MAZDA_CACHE_PROFILE_NATIVE if any(votes) else None)
  _archive_mazda_control_cache(meta_dir, params,
                               "retain ti-legacy-v1 learning" if retain_ti else "untrusted control cache")

  for key in _MAZDA_PERSISTENT_CP_KEYS:
    params.remove(key)
    if params.get(key) is not None:
      raise RuntimeError(f"params_migration: failed to remove {key}")
  if not retain_ti:
    for key in _MAZDA_TORQUE_CACHE_KEYS:
      params.remove(key)
      if params.get(key) is not None:
        raise RuntimeError(f"params_migration: failed to remove {key}")
    cloudlog.warning("params_migration: quarantined untrusted Mazda torque caches; learning restarts from seeds")
  else:
    cloudlog.info("params_migration: retained verified ti-legacy-v1 torque learning")

  # CarParamsPrevRoute survives only when parseable; paramsd/lagd/torqued read its fingerprint
  # and tuning, never its flags. It is recorded v1: migration only keeps historical blobs here.
  if params.get("CarParamsPrevRoute") is not None and not isinstance(prev_cp, dict):
    params.remove("CarParamsPrevRoute")
    if params.get("CarParamsPrevRoute") is not None:
      raise RuntimeError("params_migration: failed to remove malformed CarParamsPrevRoute")
  # Forensics only: an interrupted card write can leave a v2 PrevRoute here without a stamp.
  # No consumer trusts this label for flags; the matcher accepts either historical layout.
  prev_layout = MAZDA_CACHE_LAYOUT_V1 if params.get("CarParamsPrevRoute") is not None else None
  _write_mazda_cache_contract(meta_dir, params, profile, prev_layout)


def stamp_mazda_control_cache_contract(params, CP) -> None:
  """card seam: stamp the contract after the persistent CP/SP writes and the PrevRoute advance,
  last before other consumers. A crash or an old-branch card before this stamp leaves the
  digests mismatched, so the next boot re-quarantines."""
  if CP.brand != "mazda":
    return
  from opendbc.car.mazda.values import MazdaFlags  # lazy
  meta_dir = _mazda_meta_dir(params)
  prev = params.get("CarParamsPrevRoute")
  prev_layout = None
  if prev is not None:
    # card advanced PrevRoute from a persistent CP it wrote itself (v2), unless the digest still
    # matches the v1 blob a migration deliberately retained
    prev_layout = MAZDA_CACHE_LAYOUT_V2
    prior = _read_json_file(meta_dir / _MAZDA_CONTRACT_FILE)
    record = {}
    if isinstance(prior, dict):
      record = (prior.get("blobs") or {}).get("CarParamsPrevRoute") or {}
    if record.get("layout") == MAZDA_CACHE_LAYOUT_V1 and record.get("sha256") == _sha256(prev):
      prev_layout = MAZDA_CACHE_LAYOUT_V1
  profile = MAZDA_CACHE_PROFILE_TI_LEGACY if CP.flags & MazdaFlags.TORQUE_INTERCEPTOR else MAZDA_CACHE_PROFILE_NATIVE
  _write_mazda_cache_contract(meta_dir, params, profile, prev_layout)


def run_migration(_params):
  # Mazda control-cache migration first: old-layout persistent CP/SP must never reach offroad
  # seeding or safety, and a failure here aborts manager_init before registration
  migrate_mazda_control_cache(_params)
  # migrate OnroadScreenOffBrightness
  if _params.get("OnroadScreenOffBrightnessMigrated") != ONROAD_BRIGHTNESS_MIGRATION_VERSION:
    try:
      val = _params.get("OnroadScreenOffBrightness", return_default=True)
      if val >= 2:  # old: 5%, new: Screen Off
        new_val = val + 1
        _params.put("OnroadScreenOffBrightness", new_val, block=True)
        log_str = f"Successfully migrated OnroadScreenOffBrightness from {val} to {new_val}."
      else:
        log_str = "Migration not required for OnroadScreenOffBrightness."

      _params.put("OnroadScreenOffBrightnessMigrated", ONROAD_BRIGHTNESS_MIGRATION_VERSION, block=True)
      cloudlog.info(log_str + f" Setting OnroadScreenOffBrightnessMigrated to {ONROAD_BRIGHTNESS_MIGRATION_VERSION}")
    except Exception as e:
      cloudlog.exception(f"Error migrating OnroadScreenOffBrightness: {e}")

  # migrate OnroadScreenOffTimer
  if _params.get("OnroadScreenOffTimerMigrated") != ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION:
    try:
      val = _params.get("OnroadScreenOffTimer", return_default=True)
      if val not in VALID_TIMER_VALUES:
        _params.put("OnroadScreenOffTimer", 15, block=True)
        log_str = f"Successfully migrated OnroadScreenOffTimer from {val} to 15 (default)."
      else:
        log_str = "Migration not required for OnroadScreenOffTimer."

      _params.put("OnroadScreenOffTimerMigrated", ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION, block=True)
      cloudlog.info(log_str + f" Setting OnroadScreenOffTimerMigrated to {ONROAD_BRIGHTNESS_TIMER_MIGRATION_VERSION}")
    except Exception as e:
      cloudlog.exception(f"Error migrating OnroadScreenOffTimer: {e}")

  _migrate_car_platform_bundle(_params)

  # seed TeslaMadsScreenButton for existing Tesla installs
  _migrate_tesla_mads_screen_button(_params)

  # seed the chestnut model slot from the pre-split single slot
  _migrate_model_bundle_slots(_params)
