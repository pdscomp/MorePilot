from openpilot.cereal import custom
from opendbc.car import DT_CTRL, structs
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET, Priority

EventNameSP = custom.OnroadEventSP.EventName


def _mazda_cp() -> structs.CarParams:
  return structs.CarParams(carFingerprint="MAZDA_CX5_2022", brand="mazda")


def _cs(vEgo: float) -> structs.CarState:
  return structs.CarState(vEgo=vEgo)


def _sp(pending: bool = False, initializing: bool = False):
  return custom.CarStateSP.new_message(
    alphaLongTakeoverPending=pending,
    alphaLongTakeoverInitializing=initializing,
  )


def test_mazda_alpha_long_states_survive_card_conversion():
  state = structs.CarStateSP(
    alphaLongTakeoverPending=True,
    alphaLongTakeoverInitializing=True,
  )
  message = convert_to_capnp(state)
  assert message.alphaLongTakeoverPending
  assert message.alphaLongTakeoverInitializing


def test_alpha_long_takeover_pending_toasts_once_per_drive():
  cse = CarSpecificEventsSP(_mazda_cp(), structs.CarParamsSP())
  for _ in range(100):
    assert not cse.update(_cs(15.0), _sp(pending=True), Events()).has(EventNameSP.alphaLongTakeoverPending)
  assert cse.update(_cs(15.0), _sp(pending=True), Events()).has(EventNameSP.alphaLongTakeoverPending)
  # one-shot: never fires again while still pending
  for _ in range(300):
    assert not cse.update(_cs(15.0), _sp(pending=True), Events()).has(EventNameSP.alphaLongTakeoverPending)


def test_alpha_long_takeover_pending_stays_silent_when_clear():
  cse = CarSpecificEventsSP(_mazda_cp(), structs.CarParamsSP())
  for _ in range(300):
    assert not cse.update(_cs(15.0), _sp(), Events()).has(EventNameSP.alphaLongTakeoverPending)


def test_alpha_long_initializing_toasts_once_within_250_ms():
  cse = CarSpecificEventsSP(_mazda_cp(), structs.CarParamsSP())
  for _ in range(20):
    assert not cse.update(_cs(0.0), _sp(initializing=True), Events()).has(EventNameSP.alphaLongTakeoverInitializing)
  assert cse.update(_cs(0.0), _sp(initializing=True), Events()).has(EventNameSP.alphaLongTakeoverInitializing)
  for _ in range(300):
    assert not cse.update(_cs(0.0), _sp(initializing=True), Events()).has(EventNameSP.alphaLongTakeoverInitializing)


def test_alpha_long_initializing_can_transition_to_pending_prompt():
  cse = CarSpecificEventsSP(_mazda_cp(), structs.CarParamsSP())
  for _ in range(21):
    initializing = cse.update(_cs(0.0), _sp(initializing=True), Events())
  assert initializing.has(EventNameSP.alphaLongTakeoverInitializing)

  for _ in range(101):
    pending = cse.update(_cs(15.0), _sp(pending=True), Events())
  assert pending.has(EventNameSP.alphaLongTakeoverPending)


def test_alpha_long_initializing_alert_contract():
  alert = EVENTS_SP[EventNameSP.alphaLongTakeoverInitializing][ET.PERMANENT]
  assert alert.alert_text_1 == "Alpha Long Initializing"
  assert alert.alert_text_2 == "Keep Stopped With Cruise Off"
  assert alert.audible_alert == structs.CarControl.HUDControl.AudibleAlert.none
  assert alert.priority == Priority.LOW
  assert alert.duration == int(5.0 / DT_CTRL)
