from opendbc.car.structs import car
from openpilot.common.parameterized import parameterized_class
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper, DISTANCE_LONG_PRESS


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return self.values.get(key, False)

  def put_bool(self, key, value):
    self.values[key] = value

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


@parameterized_class(('openpilot_longitudinal',), [(True,)])
class TestCruiseHelper(OpenpilotTestCase):
  def setup_method(self):
    self.CP = car.CarParams(openpilotLongitudinalControl=self.openpilot_longitudinal)
    self.cruise_helper = CruiseHelper(self.CP)
    self.cruise_helper.experimental_mode_switched = False
    self.events = Events()

  def reset(self):
    for _ in range(2):
      CS = car.CarState(cruiseState={"available": False})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=False)]
      self.cruise_helper._experimental_mode = False
      self.cruise_helper.experimental_mode_switched = False
      self.cruise_helper.update(CS, self.events, False)


  def test_gap_adjust_cruise_long_press_toggle_mode(self) -> None:
    for pressed in (True, False):
      for experimental_mode in (True, False):
        self.reset()
        self.cruise_helper._experimental_mode = experimental_mode
        toggled_mode = not experimental_mode if pressed else experimental_mode

        for i in range(DISTANCE_LONG_PRESS):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)] if i == 0 else []
          self.cruise_helper.update(CS, self.events, experimental_mode)

        # mode should be toggled
        assert self.cruise_helper._experimental_mode == toggled_mode
        assert self.cruise_helper.experimental_mode_switched is pressed

        # keep holding button after switching mode
        for _ in range(DISTANCE_LONG_PRESS):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)]
          self.cruise_helper.update(CS, self.events, toggled_mode)

        # mode should not be toggled
        assert self.cruise_helper._experimental_mode == toggled_mode
        assert self.cruise_helper.experimental_mode_switched is pressed

  def test_gap_adjust_cruise_short_press_toggle_mode(self) -> None:
    for pressed in (True, False):
      for experimental_mode in (True, False):
        self.reset()
        self.cruise_helper._experimental_mode = experimental_mode

        for i in range(DISTANCE_LONG_PRESS - 1):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)] if i == 0 else []
          self.cruise_helper.update(CS, self.events, experimental_mode)

        # mode should not be toggled
        assert self.cruise_helper._experimental_mode == experimental_mode
        assert self.cruise_helper.experimental_mode_switched is False


class TestMazdaAlphaLongButton(OpenpilotTestCase):
  def setup_method(self):
    self.CP = car.CarParams(brand="mazda", alphaLongitudinalAvailable=True)
    self.helper = CruiseHelper(self.CP)
    self.helper.params = FakeParams(AlphaLongitudinalEnabled=False)
    self.events = Events()

  def hold_up(self, frames=DISTANCE_LONG_PRESS, enabled=False):
    for i in range(frames):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.altButton2, pressed=True)] if i == 0 else []
      self.helper.update(CS, self.events, False, enabled)

  def release_up(self, available=True):
    CS = car.CarState(cruiseState={"available": available})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.altButton2, pressed=False)]
    self.helper.update(CS, self.events, False)

  def test_long_up_toggles_alpha_long_once_per_press(self):
    self.hold_up()
    assert self.helper.params.get_bool("AlphaLongitudinalEnabled")
    self.hold_up()
    assert self.helper.params.get_bool("AlphaLongitudinalEnabled")
    self.release_up(available=False)
    self.hold_up()
    assert not self.helper.params.get_bool("AlphaLongitudinalEnabled")

  def test_short_up_and_engaged_up_do_nothing(self):
    self.hold_up(DISTANCE_LONG_PRESS - 1)
    assert not self.helper.params.get_bool("AlphaLongitudinalEnabled")
    self.release_up()
    self.hold_up(enabled=True)
    assert not self.helper.params.get_bool("AlphaLongitudinalEnabled")
    self.helper.update(car.CarState(cruiseState={"available": True}), self.events, False, enabled=False)
    assert not self.helper.params.get_bool("AlphaLongitudinalEnabled")
    self.release_up()
    self.hold_up()
    assert self.helper.params.get_bool("AlphaLongitudinalEnabled")

  def test_non_mazda_or_unavailable_does_nothing(self):
    for brand, available in (("toyota", True), ("mazda", False)):
      self.CP.brand = brand
      self.CP.alphaLongitudinalAvailable = available
      self.hold_up()
      assert not self.helper.params.get_bool("AlphaLongitudinalEnabled")
