"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time
from collections.abc import Callable

from openpilot.common.params import Params, ParamKeyFlag
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.selfdrive.car.stock_ecu_handback import StockEcuHandBackGate
from opendbc.sunnypilot.car.stock_ecu import StockEcuState
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.stock_ecu_handback import RESULT_KEY, SETTLED, StockEcuHandBackGate, read_record

# how long a shutdown waits on an accelerator that has not taken the request to power off
ACCELERATOR_OFF_WAIT_T = 25.0


class HardwaredExt:
  """zoompilot's hooks into hardwared's hardware_thread, one object so hardwared.py carries one-line call sites.

  Ordering contract with hardware_thread: update runs at the top of every loop, before the
  onroad conditions are evaluated. It returns True on the loop that starts an onroad cycle.

  hardwared owns the requested started/offroad transitions. Two intents reach it:
  OnroadCycleRequested (upstream's) and OffroadModeRequested, the forced-offroad preference
  that the UI and remote settings write. OffroadMode itself is written here alone, because
  pandad reads it and drops the panda's ignition within 100 ms, before any hand-back could
  run. Entering waits on the stock ECU hand-back while onroad (stock_ecu_handback.py); a
  failed hand-back keeps the forced-offroad request open (the exit button withdraws it)
  rather than stopping on a timer. Withdrawing after card already accepted the hand-back
  cannot cancel the in-flight restoration: the record stands until the ECU is restored,
  and update then requests exactly one onroad cycle for a coherent fresh session. A failed
  restoration stays onroad under neutral replacement, visible as failed, with late recovery
  still completing the drain.
  Exiting clears the previous session's CarParams and readiness first, so pandad sequences
  the fresh session like a boot (ELM327 until the new CarParams is ready) instead of
  applying the old safety and opening the relay seconds before controls come up.

  An accelerator on its own supply outlives the comma, so a shutdown asks it once to power
  off and DoShutdown waits until it took the request or ACCELERATOR_OFF_WAIT_T has passed.
  hardware_thread keeps publishing deviceState through that wait, and no drive may start
  under the power-off that follows (powering_off is a startup condition).
  """

  def __init__(self, params: Params) -> None:
    self.params = params
    self.handback = StockEcuHandBackGate(params, owner="hardwared")
    self.draining: int | None = None
    self.drain_failed = False
    self.now = time.monotonic
    self.accelerator_off_ts: float | None = None

  @property
  def powering_off(self) -> bool:
    return self.accelerator_off_ts is not None

  def alerts(self, set_offroad_alert: Callable[..., None]) -> None:
    # an enabled accelerator that cannot come up is otherwise silently absent
    accelerator_error = jetlink_adapter.reason()
    set_offroad_alert("Offroad_AcceleratorUnavailable", accelerator_error is not None, extra_text=accelerator_error)

  def shutdown_ready(self, should_shutdown: bool, off_ts: float | None) -> bool:
    """True once hardwared may put DoShutdown. Once asked, the power monitor is not consulted again."""
    if self.accelerator_off_ts is None:
      if not should_shutdown:
        return False
      cloudlog.warning(f"shutting device down, offroad since {off_ts}")
      jetlink_adapter.request_shutdown(f"comma shutting down, offroad since {off_ts}")
      self.accelerator_off_ts = self.now()
    return not jetlink_adapter.shutdown_pending() or self.now() - self.accelerator_off_ts >= ACCELERATOR_OFF_WAIT_T

  def update(self, started: bool) -> bool:
    cycle = self.params.get_bool("OnroadCycleRequested")
    offroad_wanted = self.params.get_bool("OffroadModeRequested")
    offroad = self.params.get_bool("OffroadMode")
    if offroad and not offroad_wanted:
      self.on_offroad_exit()
    enter = offroad_wanted and not offroad
    if not (cycle or enter):
      drained = self.handback.reset()
      if drained is not None:
        self.draining = drained
      self._drain()
      return False
    # a live waiter owns the episode again; no drain
    self.draining = None
    self.drain_failed = False
    # forced offroad has an exit button to withdraw it, so it may hold on a failed hand-back;
    # a cycle has no cancel and proceeds on any answer, like a reboot
    if not self.handback.ready(started, hold_on_failure=enter):
      return False
    if enter:
      cloudlog.warning("entering forced offroad")
      self.params.put_bool("OffroadMode", True, block=True)
    if cycle:
      self.params.put_bool("OnroadCycleRequested", False, block=True)
      self.prepare_onroad_entry()
    return cycle

  def _drain(self) -> None:
    """Watch a withdrawn-but-accepted hand-back to its end. The intent is gone, the physical
    hand-back is not: never force offroad here. On restore, request one normal onroad cycle;
    on failure stay onroad under neutral replacement and let a late recovery still finish it."""
    if self.draining is None:
      return
    result = read_record(self.params, RESULT_KEY)
    if result is None or result["id"] != self.draining:
      return
    outcome = result.get("outcome")
    if outcome in SETTLED:
      cloudlog.warning(f"stock ECU hand-back {self.draining} restored after withdrawal; cycling onroad once")
      self.draining = None
      self.drain_failed = False
      self.params.put_bool("OnroadCycleRequested", True, block=True)
    elif outcome == StockEcuState.FAILED and not self.drain_failed:
      self.drain_failed = True
      cloudlog.error(f"stock ECU hand-back {self.draining} failed after withdrawal; staying onroad")

  def on_offroad_exit(self) -> None:
    cloudlog.warning("leaving forced offroad")
    self.prepare_onroad_entry()
    self.params.put_bool("OffroadMode", False, block=True)

  def prepare_onroad_entry(self) -> None:
    # pandad races manager's onroad-transition param clearing on every software-initiated
    # entry (an onroad cycle, a forced-offroad exit). If it wins, it applies the previous
    # session's CarParams safety immediately and opens the relay seconds before
    # controls come up, cutting the camera off from the car long enough to fault it. Run the
    # same clear early so the new session sequences like a normal boot: ELM327 (relay closed)
    # until the fresh CarParams is ready.
    self.params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
