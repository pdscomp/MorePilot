from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.mazda.values import MazdaFlags
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.system import params_migration
import openpilot.selfdrive.car.card as card_module


def _owner_cp():
  CP = car.CarParams.new_message(brand="mazda", carFingerprint="MAZDA_CX5")
  CP.flags = int(MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS | MazdaFlags.TORQUE_INTERCEPTOR)
  CP.lateralTuning.init("torque")
  return CP


def _boundaries(monkeypatch, params):
  monkeypatch.setattr(card_module, "Params", lambda: params)
  monkeypatch.setattr(card_module.messaging, "sub_sock", lambda *args, **kwargs: object())
  monkeypatch.setattr(card_module.messaging, "PubMaster", lambda services: SimpleNamespace(sock={"sendcan": object()}))
  monkeypatch.setattr(card_module, "can_comm_callbacks", lambda *args: (lambda *a: None, lambda *a: None))
  monkeypatch.setattr(card_module, "set_alternative_experience", lambda *args: None)
  monkeypatch.setattr(card_module, "set_car_specific_params", lambda *args: None)
  monkeypatch.setattr(card_module, "convert_to_capnp", lambda value: custom.CarParamsSP.new_message())
  monkeypatch.setattr(card_module, "VCruiseHelper", lambda *args: SimpleNamespace())
  monkeypatch.setattr(card_module, "CardExt", lambda *args: SimpleNamespace())
  monkeypatch.setattr(card_module, "AlphaLongToggleMonitor", lambda *args: SimpleNamespace())
  monkeypatch.setattr(card_module, "Ratekeeper", lambda *args, **kwargs: SimpleNamespace())


def _interface(CP):
  return SimpleNamespace(CP=CP, CP_SP=structs.CarParamsSP(), CC=None, CS=SimpleNamespace(secoc_key=b""))


def test_cache_archive_failure_blocks_carparams_publication(monkeypatch):
  with OpenpilotPrefix():
    params = Params()
    CP = _owner_cp()
    params.put("CarParamsPersistent", CP.to_bytes(), block=True)
    params.put("LiveTorqueParameters", b"old-learning", block=True)
    params.put("LiveTorqueParametersSP", b"old-bin-learning", block=True)
    _boundaries(monkeypatch, params)
    monkeypatch.setattr(params_migration, "_archive_mazda_control_cache",
                        lambda *args: (_ for _ in ()).throw(RuntimeError("archive failed")))
    with pytest.raises(RuntimeError, match="archive failed"):
      card_module.Car(CI=_interface(CP))
    assert params.get("CarParams") is None
    assert params.get("CarParamsCache") is None
    assert params.get("LiveTorqueParameters") == b"old-learning"
    assert params.get("LiveTorqueParametersSP") == b"old-bin-learning"


def test_cache_preparation_precedes_carparams_and_new_epoch(monkeypatch):
  with OpenpilotPrefix():
    params = Params()
    CP = _owner_cp()
    params.put("CarParamsPersistent", CP.to_bytes(), block=True)
    params.put("LiveTorqueParameters", b"old-learning", block=True)
    params.put("LiveTorqueParametersSP", b"old-bin-learning", block=True)
    _boundaries(monkeypatch, params)
    card_module.Car(CI=_interface(CP))
    assert params.get("CarParams") == CP.to_bytes()
    assert params.get("CarParamsPersistent") == CP.to_bytes()
    assert params.get("LiveTorqueParameters") is None
    assert params.get("LiveTorqueParametersSP") is None
    contract = params_migration._read_json_file(params_migration._mazda_meta_dir(params) / params_migration._MAZDA_CONTRACT_FILE)
    assert contract["torque_report"] == "donor-cx5-ti-0x249-v1"
    archive = params_migration._read_json_file(params_migration._mazda_meta_dir(params) / params_migration._MAZDA_ARCHIVE_FILE)
    assert len(archive["generations"]) == 1
    assert archive["generations"][0]["keys"]["LiveTorqueParameters"]["data"] is not None
    assert archive["generations"][0]["keys"]["LiveTorqueParametersSP"]["data"] is not None
