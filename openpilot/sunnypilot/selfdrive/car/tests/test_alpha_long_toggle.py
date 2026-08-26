from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.car.alpha_long_toggle import AlphaLongToggleMonitor, HANDBACK_TIMEOUT_FRAMES


class FakeParams:
  def __init__(self, **bools):
    self.bools = dict(bools)

  def get_bool(self, key):
    return self.bools.get(key, False)

  def put_bool(self, key, value, **kwargs):
    self.bools[key] = value


def _monitor(toggle: bool, brand="mazda", op_long=True, alpha_avail=True, offroad_requested=False):
  cp = structs.CarParams()
  cp.brand = brand
  cp.openpilotLongitudinalControl = op_long
  cp.alphaLongitudinalAvailable = alpha_avail
  params = FakeParams(AlphaLongitudinalEnabled=toggle, OffroadModeRequested=offroad_requested)
  m = AlphaLongToggleMonitor(cp, params)
  m.update_params()
  return m, params


def _step(monitor, acc_faulted=False, enabled=False):
  cs = structs.CarState()
  cs.accFaulted = acc_faulted
  cc = structs.CarControl()
  cc.enabled = enabled
  cc_sp = structs.CarControlSP()
  monitor.update(cs, cc, cc_sp)
  return cc_sp


class TestAlphaLongToggleMonitor:
  def test_no_mismatch_no_action(self):
    m, params = _monitor(toggle=True, op_long=True)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert not params.get_bool("OnroadCycleRequested")

  def test_enable_direction_cycles_immediately(self):
    m, params = _monitor(toggle=True, op_long=False)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_disable_runs_handback_until_radar_returns(self):
    m, params = _monitor(toggle=False, op_long=True)
    # radar still silent: hand-back asserted, no cycle yet
    for _ in range(50):
      cc_sp = _step(m, acc_faulted=False)
      assert cc_sp.stockEcuHandBack
      assert not params.get_bool("OnroadCycleRequested")
    # stock radar heard again: cycle requested
    cc_sp = _step(m, acc_faulted=True)
    assert cc_sp.stockEcuHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_disable_times_out_to_cycle(self):
    m, params = _monitor(toggle=False, op_long=True)
    for _ in range(HANDBACK_TIMEOUT_FRAMES):
      _step(m, acc_faulted=False)
    assert params.get_bool("OnroadCycleRequested")

  def test_waits_for_disengagement(self):
    m, params = _monitor(toggle=False, op_long=True)
    cc_sp = _step(m, enabled=True)
    assert not cc_sp.stockEcuHandBack
    # once started, engagement no longer pauses the sequence
    _step(m, enabled=False)
    cc_sp = _step(m, enabled=True)
    assert cc_sp.stockEcuHandBack

  def test_handback_stays_asserted_after_done(self):
    # CC_SP is rebuilt each frame; dropping the assert once done latched made the session
    # manager read a withdrawal and re-silence the radar it had just handed back, right
    # before shutdown
    m, params = _monitor(toggle=False, op_long=True)
    _step(m)
    _step(m, acc_faulted=True)
    assert params.get_bool("OnroadCycleRequested")
    for _ in range(10):
      cc_sp = _step(m, acc_faulted=True)
      assert cc_sp.stockEcuHandBack

  def test_no_assert_after_done_when_nothing_was_handed_back(self):
    # the enable direction never starts a hand-back, so there is nothing to keep asserting
    m, params = _monitor(toggle=True, op_long=False)
    _step(m)
    assert params.get_bool("OnroadCycleRequested")
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack

  def test_non_mazda_disable_cycles_immediately(self):
    m, params = _monitor(toggle=False, brand="toyota", op_long=True)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
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


class TestOffroadRequest:
  def test_mazda_op_long_hands_back_before_granting(self):
    m, params = _monitor(toggle=True, op_long=True, offroad_requested=True)
    for _ in range(50):
      cc_sp = _step(m)
      assert cc_sp.stockEcuHandBack
      assert not params.get_bool("OffroadMode")
    _step(m, acc_faulted=True)
    assert params.get_bool("OffroadMode")
    assert not params.get_bool("OffroadModeRequested")
    assert not params.get_bool("OnroadCycleRequested")

  def test_handback_timeout_still_grants(self):
    m, params = _monitor(toggle=True, op_long=True, offroad_requested=True)
    for _ in range(HANDBACK_TIMEOUT_FRAMES):
      _step(m)
    assert params.get_bool("OffroadMode")

  def test_non_mazda_grants_immediately(self):
    m, params = _monitor(toggle=True, brand="toyota", op_long=True, offroad_requested=True)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert params.get_bool("OffroadMode")

  def test_stock_long_grants_immediately(self):
    m, params = _monitor(toggle=False, op_long=False, offroad_requested=True)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert params.get_bool("OffroadMode")

  def test_offroad_wins_over_pending_toggle_cycle(self):
    # toggle-off and force-offroad both pending: go offroad, skip the cycle
    m, params = _monitor(toggle=False, op_long=True, offroad_requested=True)
    _step(m, acc_faulted=True)
    assert params.get_bool("OffroadMode")
    assert not params.get_bool("OnroadCycleRequested")

  def test_grant_works_when_alpha_unavailable(self):
    m, params = _monitor(toggle=False, op_long=False, alpha_avail=False, offroad_requested=True)
    _step(m)
    assert params.get_bool("OffroadMode")
