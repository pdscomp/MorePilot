"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import log, custom
from opendbc.car import structs

from opendbc.car.chrysler.values import RAM_DT
from opendbc.car.mazda.values import MazdaFlags
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter


class CarSpecificEventsSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.low_speed_alert = False
    self.alpha_long_pending_frames = 0
    self.alpha_long_toast_shown = False

  def update(self, CS: structs.CarState, CS_SP: custom.CarStateSP, events: Events):
    events_sp = EventsSP()

    if self.CP.brand == 'chrysler':
      if self.CP.carFingerprint in RAM_DT:
        # remove belowSteerSpeed event from CarSpecificEvents as RAM_DT uses a different logic
        if events.has(EventName.belowSteerSpeed):
          events.remove(EventName.belowSteerSpeed)

        # TODO-SP: use if/elif to have the gear shifter condition takes precedence over the speed condition
        # TODO-SP: add 1 m/s hysteresis
        if CS.vEgo >= self.CP.minEnableSpeed:
          self.low_speed_alert = False
        if self.CP.minEnableSpeed >= 14.5 and CS.gearShifter != GearShifter.drive:
          self.low_speed_alert = True
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

    elif self.CP.brand == 'toyota':
      if self.CP.openpilotLongitudinalControl:
        if CS.cruiseState.standstill and not CS.brakePressed and self.CP_SP.enableGasInterceptor:
          if events.has(EventName.resumeRequired):
            events.remove(EventName.resumeRequired)

    elif self.CP.brand == 'mazda':
      if self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS and events.has(EventName.steerTempUnavailable):
        # steerFaultTemporary on this EPS is the non-delivery latch reporting a sustained
        # road-speed block. The latch has already zeroed the command, so there is nothing
        # for a soft disable to protect; it only costs MADS its lateral after 3 s and
        # shouts at the driver. Keep the banner, drop the escalation.
        events.remove(EventName.steerTempUnavailable)
        events.add(EventName.steerTempUnavailableSilent)

      # The alpha-long radar teardown needs a stop with stock cruise off. While it is
      # still pending the car drives with cruise/lateral locked out and no hint why;
      # say it once per drive instead of leaving the driver guessing (route
      # 00000026--5ed2c94d05: enabled never became possible on a drive-off boot).
      if CS_SP.alphaLongTakeoverPending and not self.alpha_long_toast_shown:
        self.alpha_long_pending_frames += 1
      else:
        self.alpha_long_pending_frames = 0
      if self.alpha_long_pending_frames > 100:  # ~1 s
        self.alpha_long_toast_shown = True
        events_sp.add(EventNameSP.alphaLongTakeoverPending)

    return events_sp
