"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName

DISTANCE_LONG_PRESS = 50


class CruiseHelper:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()

    self.button_frame_counts = {ButtonType.gapAdjustCruise: 0, ButtonType.altButton2: 0}
    self._experimental_mode = False
    self.experimental_mode_switched = False
    self.alpha_long_switched = False
    self.alpha_long_blocked = False

  def update(self, CS, events, experimental_mode, enabled=False) -> None:
    if any(button.type.raw == ButtonType.altButton2 and not button.pressed for button in CS.buttonEvents):
      self.alpha_long_switched = False
      self.alpha_long_blocked = False
    if CS.cruiseState.available:
      self.update_button_frame_counts(CS)
      if self.CP.openpilotLongitudinalControl:
        # toggle experimental mode once on distance-down hold
        self.update_experimental_mode(events, experimental_mode)
      self.update_alpha_long(enabled)

  def update_button_frame_counts(self, CS) -> None:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] > 0:
        self.button_frame_counts[button] += 1

    for button_event in CS.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts:
        self.button_frame_counts[button] = int(button_event.pressed)

  def update_experimental_mode(self, events, experimental_mode) -> None:
    if self.button_frame_counts[ButtonType.gapAdjustCruise] >= DISTANCE_LONG_PRESS and not self.experimental_mode_switched:
      self._experimental_mode = not experimental_mode
      self.params.put_bool("ExperimentalMode", self._experimental_mode)
      events.add(EventNameSP.experimentalModeSwitched)
      self.experimental_mode_switched = True

  def update_alpha_long(self, enabled: bool) -> None:
    # Follow-distance-up is Mazda-specific; the existing toggle monitor owns the safe restart/hand-back.
    if self.CP.brand != "mazda" or not self.CP.alphaLongitudinalAvailable:
      return
    if enabled:
      self.alpha_long_blocked |= self.button_frame_counts[ButtonType.altButton2] > 0
      return
    if self.alpha_long_blocked:
      return
    if self.button_frame_counts[ButtonType.altButton2] == 0:
      self.alpha_long_switched = False
    elif self.button_frame_counts[ButtonType.altButton2] >= DISTANCE_LONG_PRESS and not self.alpha_long_switched:
      self.params.put_bool("AlphaLongitudinalEnabled", not self.params.get_bool("AlphaLongitudinalEnabled"))
      self.alpha_long_switched = True
