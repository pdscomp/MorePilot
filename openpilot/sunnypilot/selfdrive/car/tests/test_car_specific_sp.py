from openpilot.cereal import custom
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP

EventNameSP = custom.OnroadEventSP.EventName


def _mazda_cp() -> structs.CarParams:
  return structs.CarParams(carFingerprint="MAZDA_CX5_2022", brand="mazda")


def _cs(vEgo: float) -> structs.CarState:
  return structs.CarState(vEgo=vEgo)


def _sp(pending: bool):
  return custom.CarStateSP.new_message(alphaLongTakeoverPending=pending)


def test_alpha_long_takeover_pending_toasts_once_per_drive():
  cse = CarSpecificEventsSP(_mazda_cp(), structs.CarParamsSP())
  for _ in range(100):
    assert not cse.update(_cs(15.0), _sp(True), Events()).has(EventNameSP.alphaLongTakeoverPending)
  assert cse.update(_cs(15.0), _sp(True), Events()).has(EventNameSP.alphaLongTakeoverPending)
  # one-shot: never fires again while still pending
  for _ in range(300):
    assert not cse.update(_cs(15.0), _sp(True), Events()).has(EventNameSP.alphaLongTakeoverPending)


def test_alpha_long_takeover_pending_stays_silent_when_clear():
  cse = CarSpecificEventsSP(_mazda_cp(), structs.CarParamsSP())
  for _ in range(300):
    assert not cse.update(_cs(15.0), _sp(False), Events()).has(EventNameSP.alphaLongTakeoverPending)
