"""Onroad orchestration for the AlphaLongitudinalEnabled toggle and force-offroad.

The alpha-long param is read once at fingerprint, so applying a change requires an
onroad cycle. The UI only writes the param; card owns the cycle request so brands that
silence a stock ECU can hand it back first. pandad blocks TX within ~100 ms of
`started` dropping, which makes any shutdown-time deinit impossible: the hand-back
must finish before the cycle is requested (docs/mazda-alpha-long-setup-teardown.md).

Force-offroad ("Always Offroad") is owned by hardwared: HardwaredExt holds the
OffroadModeRequested preference open, waits out the same hand-back through
StockEcuHandBackGate, and is the only writer of OffroadMode. The monitor just
stands down when it is requested - a second writer granting OffroadMode itself
clears the request, which HardwaredExt reads as a withdrawal and exits the
forced offroad one loop later, flapping ignition through pandad.

For Mazda op-long the hand-back sequence is: assert CarControlSP.stockEcuHandBack
-> carcontroller stops tester present and requests the radar's default session while
keeping synthetic frames flowing -> the stock radar's CRZ_INFO returns (carstate
raises accFaulted, its "stock radar heard" two-master guard) -> take the action.
If the session request is lost the radar still recovers via its ~5 s S3 timeout,
which the timeout below outwaits.
"""

from opendbc.car import DT_CTRL, structs
from openpilot.common.params import Params

# Seconds before the monitor stops waiting on the radar's return and takes the action anyway;
# past the radar's ~5 s S3 self-recovery. The session manager keeps its own 10 s budget on the
# HANDBACK state, and the hand-back stays asserted after done (below), so firing this does not
# abandon the default-session request -- it only decides when the cycle/offroad grant happens.
HANDBACK_TIMEOUT_T = 8.0
HANDBACK_TIMEOUT_FRAMES = int(HANDBACK_TIMEOUT_T / DT_CTRL)


class AlphaLongToggleMonitor:
  def __init__(self, CP: structs.CarParams, params: Params):
    self.CP = CP
    self.params = params
    self.toggle_enabled = CP.openpilotLongitudinalControl
    self.offroad_requested = False
    self.handback_frames = 0
    self.done = False

  def update_params(self) -> None:
    # called from card's 10 Hz params thread
    self.toggle_enabled = self.params.get_bool("AlphaLongitudinalEnabled")
    self.offroad_requested = self.params.get_bool("OffroadModeRequested")

  def request_cycle(self) -> None:
    self.params.put_bool("OnroadCycleRequested", True)
    self.done = True

  def update(self, CS: structs.CarState, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    """Runs at 100 Hz from controls_update, before CI.apply."""
    if self.done:
      # CC_SP is rebuilt every frame, so a hand-back that ran must stay asserted until the
      # process exits: the session manager reads a dropped assert as a withdrawal and would
      # re-silence the radar it just handed back. This is the producer's side of the contract
      # and it is load-bearing -- the timeout above sits inside the manager's 10 s session
      # budget, so only a held assert lets a slow hand-back finish instead of reading as
      # withdrawn. (The manager also latches a completed hand-back as a backstop for a
      # producer that stops asserting.)
      if self.handback_frames > 0:
        CC_SP.stockEcuHandBack = True
      return
    toggle_mismatch = self.CP.alphaLongitudinalAvailable and self.toggle_enabled != self.CP.openpilotLongitudinalControl
    if not toggle_mismatch:
      self.handback_frames = 0
      return
    if self.offroad_requested:
      # forced offroad wins over a pending toggle cycle, and hardwared owns it: the
      # session is ending either way and the next onroad start fingerprints with the
      # current toggle value. Standing down also keeps its hand-back gate in charge.
      self.done = True
      return

    if self.CP.brand != "mazda" or not self.CP.openpilotLongitudinalControl:
      # nothing to tear down (enable direction, or a brand without a silenced ECU)
      self.request_cycle()
      return

    # wait out an active engagement before starting the hand-back; the UIs block both
    # actions while engaged, but the params can flip from anywhere
    if CC.enabled and self.handback_frames == 0:
      return

    CC_SP.stockEcuHandBack = True
    self.handback_frames += 1
    # accFaulted doubles as "stock radar heard" while op-long is active; once the
    # radar is broadcasting again the car is back to fully stock
    if CS.accFaulted or self.handback_frames >= HANDBACK_TIMEOUT_FRAMES:
      self.request_cycle()
