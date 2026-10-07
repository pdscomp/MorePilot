"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The stock ECU hand-back: one correlated request/result pair between whoever ends the onroad
state and card, which owns the vehicle's diagnostic session.

Any software-initiated end of the onroad state (onroad cycle for a calibration reset or a
restart-needed toggle, forced offroad, reboot, shutdown, uninstall) kills TX within about
100 ms of `started` dropping. A brand that silences a stock ECU for the drive needs those
processes alive long enough to hand it back first, or the ECU recovers unattended through its
S3 timeout in a degraded state that blocks cruise until the next ignition.

Consumer (`StockEcuHandBackGate`, in hardwared and manager): writes StockEcuHandBackRequest
{id, waiters, accepted, stopCommitted} and holds its action until StockEcuHandBackResult
carries the same id. A second consumer arriving while one request is open joins it, so
exactly one hand-back is ever in flight. Each caller registers a fresh token under a stable
owner name; withdrawal releases only that token, and only a last, never-accepted,
uncommitted request is removed outright. Producer (`StockEcuHandBackServer`, in card):
claims the request (accepted) on its slow params thread, asserts the brand's hand-back off
the control loop while the request stands, and writes the result with the vehicle's
StockEcuState once it is one of restored, failed or notNeeded. Ids are monotonic past every
record on file and both records clear on the offroad transition, so a stale answer never
satisfies a new request.

The acceptance claim is the cancellation cutoff: a withdrawal that wins the lock first
cancels before any assertion, a claim that wins commits the episode to completion. A last
waiter withdrawing after acceptance leaves the record standing (intent withdrawn, hand-back
not complete); the assertion holds until the session manager restores the ECU, and
hardwared then drains it with exactly one onroad cycle. A stop that commits keeps its
generation and assertion until the offroad-transition clearing, so the vehicle cannot
re-take over while the shutdown is still pending.

A stop that can be withdrawn (forced offroad, through its exit button) proceeds only on
restored or notNeeded and stays open on failed: the vehicle keeps neutral replacement traffic
going and a late recovery still completes it. A stop with no cancel (cycle, reboot, shutdown,
uninstall) proceeds on any answer. Either proceeds after HANDBACK_WAIT_T with no answer at all,
which means no card is alive to give one. Ignition off and power loss cannot be held.

All request-record read/modify/write operations run under one flock on a file beside the
Params directory: this is a multi-process protocol, not an in-memory reference count. The
lock and all blocking writes stay off the 100 Hz control path.
"""
import contextlib
import fcntl
import time
import uuid
from pathlib import Path

from opendbc.sunnypilot.car.stock_ecu import StockEcuState
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

REQUEST_KEY = "StockEcuHandBackRequest"
RESULT_KEY = "StockEcuHandBackResult"

# The producer's own budget is one UDS attempt (10 s) plus the restore window; past this with no
# answer at all the processes are assumed dead, and the stop goes ahead regardless.
HANDBACK_WAIT_T = 15.0

SETTLED = (StockEcuState.RESTORED, StockEcuState.NOT_NEEDED)
ANSWERS = (*SETTLED, StockEcuState.FAILED)


@contextlib.contextmanager
def _request_lock(params: Params):
  """The one lock serializing every request-record read/modify/write across processes."""
  path = Path(params.get_param_path()).parent / ".stock_ecu_handback.lock"
  with open(path, "a") as f:
    fcntl.flock(f, fcntl.LOCK_EX)
    try:
      yield
    finally:
      fcntl.flock(f, fcntl.LOCK_UN)


def read_record(params: Params, key: str) -> dict | None:
  rec = params.get(key)
  return rec if isinstance(rec, dict) and isinstance(rec.get("id"), int) else None


class StockEcuHandBackGate:
  """Consumer side. Call ready() every loop while the stop is wanted, reset() if it is withdrawn.

  owner is the caller's stable name; a fresh token per request tells callers apart even when
  the owner name repeats (a restarted process asking again).
  """

  def __init__(self, params: Params, now=time.monotonic, owner: str = "unknown") -> None:
    self.params = params
    self.now = now
    self.owner = owner
    self.request_id: int | None = None
    self.token: str | None = None
    self.requested_ts: float | None = None
    self.failed = False

  @property
  def pending(self) -> bool:
    return self.request_id is not None

  def _open(self) -> None:
    with _request_lock(self.params):
      existing = read_record(self.params, REQUEST_KEY)
      result = read_record(self.params, RESULT_KEY)
      settled = result is not None and existing is not None and result["id"] == existing["id"] and result.get("outcome") in SETTLED
      self.token = uuid.uuid4().hex
      if existing is not None and (not settled or existing.get("waiters") or existing.get("stopCommitted")):
        # one hand-back in flight: join it rather than queue another behind it. A settled
        # generation with waiters still registered is joined, never replaced out from under
        # a waiter that has not observed it yet.
        self.request_id = existing["id"]
        existing.setdefault("waiters", {})[self.token] = self.owner
        self.params.put(REQUEST_KEY, existing, block=True)
      else:
        self.request_id = max([rec["id"] for rec in (existing, result) if rec is not None] + [0]) + 1
        self.params.put(REQUEST_KEY, {"id": self.request_id, "waiters": {self.token: self.owner},
                                      "accepted": False, "stopCommitted": False}, block=True)
    self.requested_ts = self.now()
    cloudlog.warning(f"stock ECU hand-back {self.request_id} requested by {self.owner} before ending onroad")

  def _close(self) -> None:
    self.request_id = None
    self.token = None
    self.requested_ts = None
    self.failed = False

  def _commit(self) -> None:
    # the caller is being released to stop: drop only this waiter and latch the committed
    # stop, retaining the record even with no waiters left so the assertion outlives the
    # wait -- the vehicle must not re-take over while the shutdown is still pending
    with _request_lock(self.params):
      rec = read_record(self.params, REQUEST_KEY)
      if rec is not None and rec["id"] == self.request_id:
        waiters = rec.get("waiters")
        if isinstance(waiters, dict):
          waiters.pop(self.token, None)
        rec["stopCommitted"] = True
        self.params.put(REQUEST_KEY, rec, block=True)
    self._close()

  def ready(self, started: bool, hold_on_failure: bool = False) -> bool:
    """True when the caller may end the onroad state now; the wait is cleared with it.
    hold_on_failure: the stop can be withdrawn, so a failed hand-back holds it open."""
    if not started:
      # the car went offroad on its own; the offroad-transition clearing wipes the record
      self._close()
      return True
    if self.request_id is None:
      self._open()
      return False
    result = read_record(self.params, RESULT_KEY)
    if result is not None and result["id"] == self.request_id:
      outcome = result.get("outcome")
      if outcome in SETTLED:
        self._commit()
        return True
      if outcome == StockEcuState.FAILED:
        if not self.failed:
          cloudlog.error(f"stock ECU hand-back {self.request_id} failed")
        self.failed = True
        if not hold_on_failure:
          self._commit()
          return True
        return False  # held open; a late recovery or a withdrawal ends it
    if self.now() - self.requested_ts > HANDBACK_WAIT_T:
      cloudlog.error(f"stock ECU hand-back {self.request_id} unanswered, ending onroad anyway")
      self._commit()
      return True
    return False

  def reset(self) -> int | None:
    """Withdraw this caller's token. Returns the request id left draining: accepted by card,
    last waiter gone, stop not committed, so the episode completes physically without us."""
    drained = None
    if self.request_id is not None:
      with _request_lock(self.params):
        rec = read_record(self.params, REQUEST_KEY)
        if rec is not None and rec["id"] == self.request_id and self.token in rec.get("waiters", {}):
          del rec["waiters"][self.token]
          if not rec["waiters"] and not rec.get("accepted") and not rec.get("stopCommitted"):
            # never reached card: cancel outright, card sees the record vanish
            self.params.remove(REQUEST_KEY)
          else:
            self.params.put(REQUEST_KEY, rec, block=True)
            if not rec["waiters"] and rec.get("accepted") and not rec.get("stopCommitted"):
              drained = self.request_id
    self._close()
    return drained


class StockEcuHandBackServer:
  """Producer side, on card's control loop."""

  def __init__(self, params: Params) -> None:
    self.params = params
    self.request: dict | None = None
    self.started = False       # asserted once: held for as long as the request stands
    self.started_id: int | None = None
    self.answered: tuple[int, StockEcuState] | None = None

  def update_params(self) -> None:
    # rides card's 10 Hz params thread. Claiming acceptance under the lock, before the
    # request is cached for the control loop, is the cancellation cutoff: a withdrawal that
    # wins the lock first removes the record unseen; a claim that wins commits the episode.
    with _request_lock(self.params):
      request = read_record(self.params, REQUEST_KEY)
      if request is not None and not request.get("accepted", False):
        request["accepted"] = True
        self.params.put(REQUEST_KEY, request, block=True)
      self.request = request
      if request is None or request["id"] != self.started_id:
        self.started = False
        self.started_id = None

  def _answer(self, request_id: int, outcome: StockEcuState) -> None:
    if self.answered == (request_id, outcome):
      return
    with _request_lock(self.params):
      rec = read_record(self.params, REQUEST_KEY)
      if rec is None or rec["id"] != request_id:
        # the episode ended without us (transition clearing): never resurrect an absent id,
        # and drop the stale cache so the next frame stops asserting
        self.request = None
        return
      self.params.put(RESULT_KEY, {"id": request_id, "outcome": str(outcome)}, block=True)
    self.answered = (request_id, outcome)
    (cloudlog.error if outcome == StockEcuState.FAILED else cloudlog.warning)(f"stock ECU hand-back {request_id} answered {outcome}")

  def update(self, enabled: bool, state: StockEcuState, CC_SP) -> None:
    """Runs at 100 Hz before CI.apply with the controller's stock ECU state. CC_SP is rebuilt
    every frame, so the assert is re-applied here; the session manager treats a dropped assert
    as a withdrawal."""
    if self.request is None:
      self.started = False
      self.started_id = None
      return
    request_id = self.request["id"]
    if state == StockEcuState.NOT_NEEDED:
      self._answer(request_id, state)
      return
    # Never start on an engaged car: the hand-back revokes availability under the driver.
    # Once started it runs to its end.
    if not self.started and enabled:
      return
    self.started = True
    self.started_id = request_id
    CC_SP.stockEcuHandBack = True
    if state in ANSWERS:
      self._answer(request_id, state)
