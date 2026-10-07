"""Onroad-cycle intent for the AlphaLongitudinalEnabled toggle.

The alpha-long param is read once at fingerprint, so applying a change requires an onroad
cycle. The UI only writes the param; this monitor notices the mismatch and requests the
cycle. That is all it does: hardwared's StockEcuHandBackGate and card's hand-back server
own the stock ECU through the cycle, pandad blocks TX within ~100 ms of `started` dropping,
and the hand-back finishes before the cycle is taken (docs/mazda-alpha-long-setup-teardown.md).

Force-offroad ("Always Offroad") is also hardwared's alone: a pending request merely defers
the toggle, and withdrawing it lets a standing mismatch issue its cycle. A requested cycle
is irrevocable once hardwared consumes it, so the monitor latches done on issue.
"""

from opendbc.car import structs
from openpilot.common.params import Params


class AlphaLongToggleMonitor:
  def __init__(self, CP: structs.CarParams, params: Params):
    self.CP = CP
    self.params = params
    self.toggle_enabled = CP.openpilotLongitudinalControl
    self.offroad_requested = False
    self.done = False

  def update_params(self) -> None:
    # called from card's 10 Hz params thread
    self.toggle_enabled = self.params.get_bool("AlphaLongitudinalEnabled")
    self.offroad_requested = self.params.get_bool("OffroadModeRequested")

  def update(self, CS: structs.CarState, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    """Runs at 100 Hz from controls_update. Writes only OnroadCycleRequested; it never
    asserts the hand-back itself, so card's server stays the single restoration owner."""
    if self.done:
      return
    toggle_mismatch = self.CP.alphaLongitudinalAvailable and self.toggle_enabled != self.CP.openpilotLongitudinalControl
    if not toggle_mismatch:
      return
    if self.offroad_requested or CC.enabled:
      # forced offroad ends the session first (hardwared owns it), and a cycle is never
      # requested under the driver; either deferral re-arms on a later frame
      return
    self.params.put_bool("OnroadCycleRequested", True)
    self.done = True
