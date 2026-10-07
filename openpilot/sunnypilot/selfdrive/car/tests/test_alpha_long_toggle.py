from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.car.alpha_long_toggle import AlphaLongToggleMonitor


class FakeParams:
  def __init__(self, **bools):
    self.bools = dict(bools)

  def get_bool(self, key):
    return self.bools.get(key, False)

  def put_bool(self, key, value, **kwargs):
    self.bools[key] = value


def _monitor(toggle: bool, op_long=True, alpha_avail=True, offroad_requested=False):
  cp = structs.CarParams()
  cp.openpilotLongitudinalControl = op_long
  cp.alphaLongitudinalAvailable = alpha_avail
  params = FakeParams(AlphaLongitudinalEnabled=toggle, OffroadModeRequested=offroad_requested)
  m = AlphaLongToggleMonitor(cp, params)
  m.update_params()
  return m, params


def _step(monitor, enabled=False):
  cs = structs.CarState()
  cc = structs.CarControl()
  cc.enabled = enabled
  cc_sp = structs.CarControlSP()
  monitor.update(cs, cc, cc_sp)
  return cc_sp


class TestAlphaLongToggleMonitor:
  """The monitor is intent-only: it writes OnroadCycleRequested for a genuine disengaged
  mismatch. The hand-back itself is card's server plus hardwared's gate, never the monitor."""

  def test_no_mismatch_no_action(self):
    m, params = _monitor(toggle=True, op_long=True)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert not params.get_bool("OnroadCycleRequested")
    assert not m.done

  def test_disable_direction_requests_a_cycle(self):
    m, params = _monitor(toggle=False, op_long=True)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack  # the server asserts, not the monitor
    assert params.get_bool("OnroadCycleRequested")
    assert m.done

  def test_enable_direction_requests_a_cycle(self):
    m, params = _monitor(toggle=True, op_long=False)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_monitor_never_asserts_the_handback(self):
    # accFaulted is the carstate guard, not the monitor's signal; restoration has one owner
    m, params = _monitor(toggle=False, op_long=True)
    cs = structs.CarState()
    cs.accFaulted = True
    cc = structs.CarControl()
    cc_sp = structs.CarControlSP()
    m.update(cs, cc, cc_sp)
    assert not cc_sp.stockEcuHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_waits_for_disengagement(self):
    m, params = _monitor(toggle=False, op_long=True)
    for _ in range(10):
      cc_sp = _step(m, enabled=True)
      assert not cc_sp.stockEcuHandBack
      assert not params.get_bool("OnroadCycleRequested")
      assert not m.done
    _step(m, enabled=False)
    assert params.get_bool("OnroadCycleRequested")

  def test_unavailable_never_acts(self):
    m, params = _monitor(toggle=True, op_long=False, alpha_avail=False)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert not params.get_bool("OnroadCycleRequested")

  def test_cycle_requested_only_once(self):
    m, params = _monitor(toggle=True, op_long=False)
    _step(m)
    params.put_bool("OnroadCycleRequested", False)  # hardwared consumed it
    _step(m)
    assert not params.get_bool("OnroadCycleRequested")
    assert m.done  # the issued cycle is irrevocable, so done latches

  def test_done_does_not_block_a_new_monitor(self):
    # the cycle restarts card; the next process fingerprints with the new toggle value
    m, params = _monitor(toggle=True, op_long=False)
    _step(m)
    fresh, _ = _monitor(toggle=True, op_long=True)  # fingerprint now matches the toggle
    _step(fresh)
    assert not fresh.done


class TestOffroadRequest:
  """Force-offroad is hardwared's alone (HardwaredExt + StockEcuHandBackGate own the grant and
  its hand-back). A pending request merely defers the toggle; withdrawing it re-arms the
  mismatch, so a cancelled forced offroad can no longer strand the toggle."""

  def test_pending_offroad_defers_without_latching(self):
    m, params = _monitor(toggle=False, op_long=True, offroad_requested=True)
    for _ in range(100):
      cc_sp = _step(m)
      assert not cc_sp.stockEcuHandBack
      assert not params.get_bool("OnroadCycleRequested")
    assert not m.done  # deferred, not consumed
    assert not params.get_bool("OffroadMode")
    assert params.get_bool("OffroadModeRequested")  # left open for hardwared

  def test_withdrawn_offroad_does_not_abandon_pending_toggle(self):
    m, params = _monitor(toggle=False, op_long=True, offroad_requested=True)
    _step(m)
    assert not params.get_bool("OnroadCycleRequested")
    params.put_bool("OffroadModeRequested", False)  # the exit button
    m.update_params()
    _step(m)
    assert params.get_bool("OnroadCycleRequested")
    assert m.done
    assert not params.get_bool("OffroadMode")

  def test_enable_direction_also_defers_to_offroad(self):
    m, params = _monitor(toggle=True, op_long=False, offroad_requested=True)
    _step(m)
    assert not params.get_bool("OnroadCycleRequested")
    assert not m.done
