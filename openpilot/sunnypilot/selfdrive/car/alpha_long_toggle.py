"""Onroad orchestration for the AlphaLongitudinalEnabled toggle.

The param is read once at fingerprint, so applying a change requires an onroad cycle.
The UI only writes the param; card owns the cycle request so brands that silence a
stock ECU can hand it back first. pandad blocks TX within ~100 ms of `started`
dropping, which makes any shutdown-time deinit impossible: the hand-back must finish
before the cycle is requested (docs/mazda-alpha-long-setup-teardown.md).

For Mazda op-long the sequence on toggle-off is: assert CarControlSP.stockEcuHandBack
-> carcontroller stops tester present and requests the radar's default session while
keeping synthetic frames flowing -> the stock radar's CRZ_INFO returns (carstate
raises accFaulted, its "stock radar heard" two-master guard) -> request the cycle.
If the session request is lost the radar still recovers via its ~5 s S3 timeout,
which the timeout below outwaits.
"""

from opendbc.car import DT_CTRL, structs
from openpilot.common.params import Params

HANDBACK_TIMEOUT_T = 8.0  # seconds, past the radar's ~5 s S3 self-recovery
HANDBACK_TIMEOUT_FRAMES = int(HANDBACK_TIMEOUT_T / DT_CTRL)


class AlphaLongToggleMonitor:
  def __init__(self, CP: structs.CarParams, params: Params):
    self.CP = CP
    self.params = params
    self.toggle_enabled = CP.openpilotLongitudinalControl
    self.handback_frames = 0
    self.cycle_requested = False

  def update_params(self) -> None:
    # called from card's 10 Hz params thread
    self.toggle_enabled = self.params.get_bool("AlphaLongitudinalEnabled")

  def request_cycle(self) -> None:
    self.params.put_bool("OnroadCycleRequested", True)
    self.cycle_requested = True

  def update(self, CS: structs.CarState, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    """Runs at 100 Hz from controls_update, before CI.apply."""
    if not self.CP.alphaLongitudinalAvailable or self.cycle_requested:
      return
    if self.toggle_enabled == self.CP.openpilotLongitudinalControl:
      self.handback_frames = 0
      return

    if self.CP.brand != "mazda" or not self.CP.openpilotLongitudinalControl:
      # nothing to tear down (enable direction, or a brand without a silenced ECU):
      # restart immediately and refingerprint with the new toggle value
      self.request_cycle()
      return

    # wait out an active engagement before starting the hand-back; the UI blocks the
    # toggle while engaged, but the param can flip from anywhere
    if CC.enabled and self.handback_frames == 0:
      return

    CC_SP.stockEcuHandBack = True
    self.handback_frames += 1
    # accFaulted doubles as "stock radar heard" while op-long is active; once the
    # radar is broadcasting again the restart gap leaves the car fully stock
    if CS.accFaulted or self.handback_frames >= HANDBACK_TIMEOUT_FRAMES:
      self.request_cycle()
