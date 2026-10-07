"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import multiprocessing
from contextlib import nullcontext

import pytest

from opendbc.car import structs
from opendbc.sunnypilot.car.stock_ecu import StockEcuState
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.common.prefix import OpenpilotPrefix
import openpilot.sunnypilot.selfdrive.car.stock_ecu_handback as handback_protocol
from openpilot.sunnypilot.selfdrive.car.stock_ecu_handback import (HANDBACK_WAIT_T, REQUEST_KEY, RESULT_KEY,
                                                                   StockEcuHandBackGate, StockEcuHandBackServer)
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeClock, FakeParams, answer
from openpilot.sunnypilot.system.hardware.hardwared_ext import HardwaredExt

RESTORED, FAILED, NOT_NEEDED = StockEcuState.RESTORED, StockEcuState.FAILED, StockEcuState.NOT_NEEDED
RESTORING = StockEcuState.RESTORING


@pytest.fixture
def fake_lock(monkeypatch):
  # dictionary-fake tests are pure state; the real flock is exercised by the real-Params tests
  monkeypatch.setattr(handback_protocol, "_request_lock", lambda params: nullcontext())


def _gate(**values):
  params = FakeParams(**values)
  clock = FakeClock()
  gate = StockEcuHandBackGate(params, now=clock)
  return gate, params, clock


def _owners(params):
  request = params.get(REQUEST_KEY)
  return sorted(request["waiters"].values()) if request else None


class TestStockEcuHandBackGate:
  @pytest.fixture(autouse=True)
  def _lock(self, fake_lock):
    pass

  def test_offroad_is_always_ready_and_asks_nothing(self):
    gate, params, _ = _gate()
    assert gate.ready(started=False)
    assert params.get(REQUEST_KEY) is None

  def test_onroad_asks_once_then_waits_for_its_own_answer(self):
    gate, params, _ = _gate()
    assert not gate.ready(started=True)
    assert params.get(REQUEST_KEY)["id"] == 1
    assert _owners(params) == ["unknown"]
    for _ in range(5):
      assert not gate.ready(started=True)
    assert params.get(REQUEST_KEY)["id"] == 1  # asked once
    answer(params, RESTORED, 7)  # somebody else's answer
    assert not gate.ready(started=True)
    answer(params, RESTORED)
    assert gate.ready(started=True)
    assert not gate.pending

  def test_stale_answer_from_the_previous_request_cannot_satisfy_a_new_one(self):
    gate, params, _ = _gate()
    params.put(REQUEST_KEY, {"id": 1, "waiters": {}, "accepted": True, "stopCommitted": False})
    answer(params, RESTORED)
    # the same consumer asks again after observing an uncommitted settled generation:
    # a fresh id, the old answer is not it
    assert not gate.ready(started=True)
    assert params.get(REQUEST_KEY)["id"] == 2
    assert not gate.ready(started=True)
    answer(params, NOT_NEEDED, 2)
    assert gate.ready(started=True)

  def test_second_consumer_joins_the_open_request(self):
    # a reboot pressed while forced offroad is already waiting: one hand-back, both proceed on it
    hw, params, _ = _gate()
    hw.ready(started=True)
    mgr = StockEcuHandBackGate(params, now=FakeClock(), owner="manager")
    assert not mgr.ready(started=True)
    assert params.get(REQUEST_KEY)["id"] == 1
    assert _owners(params) == ["manager", "unknown"]
    answer(params, RESTORED)
    assert hw.ready(started=True) and mgr.ready(started=True)

  def test_a_withdrawable_stop_holds_on_failure_until_late_recovery(self):
    gate, params, clock = _gate()
    gate.ready(started=True, hold_on_failure=True)
    answer(params, FAILED)
    assert not gate.ready(started=True, hold_on_failure=True)
    assert gate.failed and gate.pending
    clock.t = HANDBACK_WAIT_T * 3  # the no-answer bound does not apply: card answered
    assert not gate.ready(started=True, hold_on_failure=True)
    answer(params, RESTORED)
    assert gate.ready(started=True, hold_on_failure=True)

  def test_a_stop_with_no_cancel_proceeds_on_failure(self):
    gate, params, _ = _gate()
    gate.ready(started=True)
    answer(params, FAILED)
    assert gate.ready(started=True)

  def test_no_answer_at_all_is_bounded(self):
    # no card alive to answer: nothing holds the ECU, so nothing is gained by waiting
    for hold in (True, False):
      gate, params, clock = _gate()
      assert not gate.ready(started=True, hold_on_failure=hold)
      clock.t = HANDBACK_WAIT_T - 0.1
      assert not gate.ready(started=True, hold_on_failure=hold)
      clock.t = HANDBACK_WAIT_T + 0.1
      assert gate.ready(started=True, hold_on_failure=hold)

  def test_going_offroad_mid_wait_releases(self):
    gate, params, _ = _gate()
    assert not gate.ready(started=True)
    assert gate.ready(started=False)
    assert not gate.pending

  def test_withdrawal_removes_only_its_own_request(self):
    gate, params, _ = _gate()
    gate.ready(started=True)
    gate.reset()
    assert params.get(REQUEST_KEY) is None and not gate.pending and not gate.failed
    # a request opened by someone else stays
    gate.ready(started=True)
    params.put(REQUEST_KEY, {"id": 5})
    gate.reset()
    assert params.get(REQUEST_KEY)["id"] == 5

  def test_withdrawal_releases_only_its_own_token(self):
    # a matching id alone is not cancellation rights: the caller's token must be registered
    gate, params, _ = _gate()
    gate.ready(started=True)
    other = StockEcuHandBackGate(params, now=FakeClock(), owner="manager")
    other.ready(started=True)
    gate.reset()
    request = params.get(REQUEST_KEY)
    assert request is not None and list(request["waiters"].values()) == ["manager"]
    other.reset()
    assert params.get(REQUEST_KEY) is None  # last, never-accepted, uncommitted: removed

  def test_re_ask_after_withdrawal_gets_a_new_id(self):
    gate, params, _ = _gate()
    gate.ready(started=True, hold_on_failure=True)
    answer(params, FAILED)
    gate.ready(started=True, hold_on_failure=True)
    gate.reset()
    assert not gate.ready(started=True)
    assert params.get(REQUEST_KEY)["id"] == 2
    assert not gate.ready(started=True)  # the failed answer for id 1 is not an answer for id 2

  def test_accepted_request_survives_withdrawal(self):
    # card already claimed it: the physical hand-back completes even with no waiter left
    gate, params, _ = _gate()
    gate.ready(started=True)
    server = StockEcuHandBackServer(params)
    server.update_params()
    assert params.get(REQUEST_KEY)["accepted"]
    assert gate.reset() == 1  # left draining
    request = params.get(REQUEST_KEY)
    assert request["waiters"] == {} and request["accepted"] and not request["stopCommitted"]

  def test_committed_stop_keeps_its_record(self):
    gate, params, _ = _gate()
    other = StockEcuHandBackGate(params, now=FakeClock(), owner="manager")
    gate.ready(started=True)
    other.ready(started=True)  # joined: two registered waiters on one generation
    answer(params, RESTORED)
    assert gate.ready(started=True)
    request = params.get(REQUEST_KEY)
    assert request is not None and request["stopCommitted"]
    assert list(request["waiters"].values()) == ["manager"]
    # a later withdrawal of the joined caller cannot remove a committed record
    other.reset()
    request = params.get(REQUEST_KEY)
    assert request is not None and request["stopCommitted"] and request["waiters"] == {}
    # a committed stop keeps the settled generation for the transition clear: the next
    # caller joins it rather than opening a second hand-back while shutdown is pending
    third = StockEcuHandBackGate(params, now=FakeClock())
    assert not third.ready(started=True)
    assert params.get(REQUEST_KEY)["id"] == 1
    assert third.ready(started=True)
    assert params.get(REQUEST_KEY)["waiters"] == {} and params.get(REQUEST_KEY)["stopCommitted"]


def _server(request_id=1):
  params = FakeParams()
  if request_id is not None:
    params.put(REQUEST_KEY, {"id": request_id})
  server = StockEcuHandBackServer(params)
  server.update_params()
  return server, params


def _step(server, state=StockEcuState.STARTING, enabled=False):
  cc_sp = structs.CarControlSP()
  server.update(enabled, state, cc_sp)
  return cc_sp


class TestStockEcuHandBackServer:
  @pytest.fixture(autouse=True)
  def _lock(self, fake_lock):
    pass

  def test_nothing_to_hand_back_answers_not_needed_at_once(self):
    server, params = _server()
    cc_sp = _step(server, NOT_NEEDED)
    assert not cc_sp.stockEcuHandBack
    assert params.get(RESULT_KEY) == {"id": 1, "outcome": "notNeeded"}

  def test_nothing_requested_nothing_asserted(self):
    server, params = _server(request_id=None)
    for _ in range(10):
      assert not _step(server).stockEcuHandBack
    assert params.get(RESULT_KEY) is None

  def test_poll_claims_acceptance_once(self):
    server, params = _server()
    assert params.get(REQUEST_KEY)["accepted"]
    params.put(REQUEST_KEY, dict(params.get(REQUEST_KEY), accepted=False))
    server.update_params()
    assert params.get(REQUEST_KEY)["accepted"]  # reclaimed; the record was never removed

  def test_asserts_until_the_session_restores_then_answers(self):
    server, params = _server()
    for state in (StockEcuState.READY, RESTORING, RESTORING):
      assert _step(server, state).stockEcuHandBack
      assert params.get(RESULT_KEY) is None
    assert _step(server, RESTORED).stockEcuHandBack
    assert params.get(RESULT_KEY)["outcome"] == "restored"
    # the assert holds while the request stands
    for _ in range(10):
      assert _step(server, RESTORED).stockEcuHandBack

  def test_waits_for_disengagement_to_start_then_runs_to_the_end(self):
    server, params = _server()
    for _ in range(20):
      assert not _step(server, StockEcuState.READY, enabled=True).stockEcuHandBack
    assert _step(server, StockEcuState.READY, enabled=False).stockEcuHandBack
    assert _step(server, RESTORED, enabled=True).stockEcuHandBack
    assert params.get(RESULT_KEY)["outcome"] == "restored"

  def test_failure_then_late_recovery_are_two_answers(self):
    server, params = _server()
    _step(server, RESTORING)
    _step(server, FAILED)
    assert params.get(RESULT_KEY) == {"id": 1, "outcome": "failed"}
    params.put(RESULT_KEY, None)  # a consumer would not clear it; make the re-answer observable
    for _ in range(5):
      _step(server, FAILED)
    assert params.get(RESULT_KEY) is None  # answered once per outcome
    _step(server, RESTORED)
    assert params.get(RESULT_KEY)["outcome"] == "restored"

  def test_new_request_id_reevaluates_the_engaged_guard(self):
    server, params = _server()
    assert _step(server, StockEcuState.READY, enabled=False).stockEcuHandBack
    assert server.started
    params.put(REQUEST_KEY, {"id": 2})
    server.update_params()

    assert not _step(server, StockEcuState.READY, enabled=True).stockEcuHandBack
    assert params.get(RESULT_KEY) is None

  def test_new_request_id_is_answered_again(self):
    server, params = _server()
    _step(server, RESTORED)
    assert params.get(RESULT_KEY)["id"] == 1
    params.put(REQUEST_KEY, {"id": 2})
    server.update_params()
    _step(server, RESTORED)
    assert params.get(RESULT_KEY)["id"] == 2

  def test_withdrawn_request_drops_the_assert(self):
    # the record gone is the withdrawal: the vehicle's session manager finishes any in-flight
    # restoration on its own and treats the next takeover as a first one
    server, params = _server()
    assert _step(server, StockEcuState.READY).stockEcuHandBack
    params.remove(REQUEST_KEY)
    server.update_params()
    assert not _step(server, RESTORING).stockEcuHandBack
    assert not server.started


class TestJoinedOwnership:
  """The cross-process request record: real isolated Params, the real flock."""

  @pytest.mark.parametrize('manager_first', [False, True])
  def test_joined_request_survives_one_owner_withdrawal(self, manager_first):
    with OpenpilotPrefix():
      params = Params()
      clock = FakeClock()
      hardware = StockEcuHandBackGate(params, now=clock, owner='hardwared')
      manager_gate = StockEcuHandBackGate(params, now=clock, owner='manager')
      gates = (manager_gate, hardware) if manager_first else (hardware, manager_gate)
      for gate in gates:
        assert not gate.ready(started=True)
      request_id = params.get(REQUEST_KEY)['id']
      assert hardware.request_id == manager_gate.request_id == request_id
      hardware.reset()
      request = params.get(REQUEST_KEY)
      assert request['id'] == request_id
      assert list(request['waiters'].values()) == ['manager']
      server = StockEcuHandBackServer(params)
      server.update_params()
      command = structs.CarControlSP()
      server.update(False, RESTORING, command)
      assert command.stockEcuHandBack
      params.put(RESULT_KEY, {'id': request_id, 'outcome': 'restored'}, block=True)
      assert manager_gate.ready(started=True)
      assert params.get(REQUEST_KEY)['stopCommitted']

  def test_reset_before_server_claim_cancels_without_assertion(self):
    with OpenpilotPrefix():
      params = Params()
      gate = StockEcuHandBackGate(params, now=FakeClock(), owner='hardwared')
      assert not gate.ready(started=True)
      assert gate.reset() is None  # never accepted: no drain, cancelled outright
      assert params.get(REQUEST_KEY) is None
      # the slow poll never sees it: no acceptance, no assertion, no result
      server = StockEcuHandBackServer(params)
      for _ in range(3):
        server.update_params()
        command = structs.CarControlSP()
        server.update(False, RESTORING, command)
        assert not command.stockEcuHandBack
        assert params.get(REQUEST_KEY) is None
        assert params.get(RESULT_KEY) is None
      # a later request is a fresh generation and is served normally
      gate2 = StockEcuHandBackGate(params, now=FakeClock(), owner='manager')
      assert not gate2.ready(started=True)
      server.update_params()
      command = structs.CarControlSP()
      server.update(False, RESTORED, command)
      assert command.stockEcuHandBack
      assert params.get(RESULT_KEY)['id'] == gate2.request_id

  def test_reset_after_server_claim_drains_before_one_cycle(self):
    with OpenpilotPrefix():
      params = Params()
      clock = FakeClock()
      ext = HardwaredExt(params)
      ext.handback.now = clock
      server = StockEcuHandBackServer(params)
      params.put_bool('OffroadModeRequested', True, block=True)
      assert not ext.update(started=True)  # hardwared's gate opens the request
      request_id = params.get(REQUEST_KEY)['id']
      server.update_params()  # card claims it: the cancellation cutoff has passed
      assert params.get(REQUEST_KEY)['accepted']
      command = structs.CarControlSP()
      server.update(False, RESTORING, command)
      assert command.stockEcuHandBack
      # the user withdraws forced offroad mid-restore: intent is gone, the hand-back is not
      params.put_bool('OffroadModeRequested', False, block=True)
      for _ in range(3):
        assert not ext.update(started=True)
        assert not params.get_bool('OffroadMode')          # no offroad grant without intent
        assert not params.get_bool('OnroadCycleRequested') # no cycle before the drain ends
        server.update_params()
        command = structs.CarControlSP()
        server.update(False, RESTORING, command)
        assert command.stockEcuHandBack  # the record stands, so card keeps restoring
      request = params.get(REQUEST_KEY)
      assert request['id'] == request_id and request['waiters'] == {}
      # the ECU comes back: hardwared drains it with exactly one normal onroad cycle
      command = structs.CarControlSP()
      server.update(False, RESTORED, command)
      assert not ext.update(started=True)
      assert params.get_bool('OnroadCycleRequested')
      # the drained cycle runs the normal gated path: a fresh generation, served, consumed
      assert not ext.update(started=True)
      request = params.get(REQUEST_KEY)
      assert request['id'] != request_id
      assert list(request['waiters'].values()) == ['hardwared']
      server.update_params()
      command = structs.CarControlSP()
      server.update(False, RESTORED, command)
      assert ext.update(started=True)
      assert not params.get_bool('OnroadCycleRequested')
      for _ in range(3):  # and it stays exactly one cycle
        assert not ext.update(started=True)
        assert not params.get_bool('OnroadCycleRequested')

  def test_completed_stop_holds_assertion_until_transition_clear(self):
    with OpenpilotPrefix():
      params = Params()
      gate = StockEcuHandBackGate(params, now=FakeClock(), owner='hardwared')
      assert not gate.ready(started=True)
      request_id = params.get(REQUEST_KEY)['id']
      server = StockEcuHandBackServer(params)
      server.update_params()
      command = structs.CarControlSP()
      server.update(False, RESTORING, command)
      assert command.stockEcuHandBack
      command = structs.CarControlSP()
      server.update(False, RESTORED, command)
      assert params.get(RESULT_KEY)['outcome'] == 'restored'
      assert gate.ready(started=True)  # the stop commits
      request = params.get(REQUEST_KEY)
      assert request['id'] == request_id
      assert request['waiters'] == {} and request['stopCommitted']
      # the assertion holds while the record stands: shutdown pending is not restored-to-drive
      server.update_params()
      for _ in range(3):
        command = structs.CarControlSP()
        server.update(False, RESTORED, command)
        assert command.stockEcuHandBack
      # the existing offroad-transition clearing ends the generation
      params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
      assert params.get(REQUEST_KEY) is None and params.get(RESULT_KEY) is None
      server.update_params()
      command = structs.CarControlSP()
      server.update(False, RESTORED, command)
      assert not command.stockEcuHandBack

  def test_stale_server_poll_cannot_resurrect_request(self):
    with OpenpilotPrefix():
      params = Params()
      gate = StockEcuHandBackGate(params, now=FakeClock(), owner='hardwared')
      assert not gate.ready(started=True)
      server = StockEcuHandBackServer(params)
      server.update_params()
      command = structs.CarControlSP()
      server.update(False, RESTORING, command)
      assert command.stockEcuHandBack  # cached and asserted for id 1
      # the episode ends with the session while the server still holds its cached copy
      params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
      server.update_params()
      assert params.get(REQUEST_KEY) is None  # the poll itself recreates nothing
      # a delayed frame against the stale cache must not resurrect the absent id
      command = structs.CarControlSP()
      server.update(False, RESTORED, command)
      assert params.get(RESULT_KEY) is None
      assert server.request is None
      command = structs.CarControlSP()
      server.update(False, RESTORING, command)
      assert not command.stockEcuHandBack
      # the next request is a fresh generation and is answered against the current record
      gate2 = StockEcuHandBackGate(params, now=FakeClock(), owner='manager')
      assert not gate2.ready(started=True)
      server.update_params()
      command = structs.CarControlSP()
      server.update(False, RESTORED, command)
      assert params.get(RESULT_KEY)['id'] == gate2.request_id

  def test_manager_stop_wins_over_withdrawn_offroad(self):
    with OpenpilotPrefix():
      params = Params()
      clock = FakeClock()
      ext = HardwaredExt(params)
      ext.handback.now = clock
      manager_gate = StockEcuHandBackGate(params, now=clock, owner='manager')
      params.put_bool('OffroadModeRequested', True, block=True)
      assert not ext.update(started=True)      # forced offroad waits on the hand-back
      assert not manager_gate.ready(started=True)  # a shutdown pressed joins the same request
      request_id = params.get(REQUEST_KEY)['id']
      server = StockEcuHandBackServer(params)
      server.update_params()
      # the offroad intent is withdrawn while the manager stop stands
      params.put_bool('OffroadModeRequested', False, block=True)
      assert not ext.update(started=True)
      request = params.get(REQUEST_KEY)
      assert request['id'] == request_id
      assert list(request['waiters'].values()) == ['manager']
      # the hand-back completes: the manager stop proceeds and commits, no drain, no cycle
      command = structs.CarControlSP()
      server.update(False, RESTORED, command)
      assert manager_gate.ready(started=True)
      request = params.get(REQUEST_KEY)
      assert request['stopCommitted'] and request['waiters'] == {}
      for _ in range(3):
        assert not ext.update(started=True)
        assert not params.get_bool('OnroadCycleRequested')
        assert not params.get_bool('OffroadMode')

  def test_two_processes_share_one_request_record(self, tmp_path):
    d = str(tmp_path / 'params')
    parent_opened = multiprocessing.Event()
    child_opened = multiprocessing.Event()
    parent_reset = multiprocessing.Event()
    queue = multiprocessing.Queue()

    def child(d, parent_opened, child_opened, parent_reset, queue):
      params = Params(d)
      gate = StockEcuHandBackGate(params, owner='manager')
      assert parent_opened.wait(10)
      assert not gate.ready(started=True)  # joins under the real flock
      child_opened.set()
      assert parent_reset.wait(10)
      request = params.get(REQUEST_KEY)
      queue.put((gate.request_id, request['id'], sorted(request['waiters'].values())))

    params = Params(d)
    gate = StockEcuHandBackGate(params, owner='hardwared')
    assert not gate.ready(started=True)
    parent_opened.set()
    proc = multiprocessing.Process(target=child, args=(d, parent_opened, child_opened, parent_reset, queue))
    proc.start()
    try:
      assert child_opened.wait(10)
      request = params.get(REQUEST_KEY)
      assert sorted(request['waiters'].values()) == ['hardwared', 'manager']
      gate.reset()
      parent_reset.set()
      proc.join(10)
      assert proc.exitcode == 0
      child_rid, record_id, owners = queue.get(timeout=10)
      assert child_rid == record_id == 1
      assert owners == ['manager']  # neither process lost the other's token
      assert list(params.get(REQUEST_KEY)['waiters'].values()) == ['manager']
    finally:
      parent_reset.set()
      proc.join(10)
      if proc.is_alive():
        proc.terminate()
