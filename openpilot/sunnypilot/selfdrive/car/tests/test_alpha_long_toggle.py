from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.car.alpha_long_toggle import AlphaLongToggleMonitor, HANDBACK_TIMEOUT_FRAMES


class FakeParams:
  def __init__(self, **bools):
    self.bools = dict(bools)

  def get_bool(self, key):
    return self.bools.get(key, False)

  def put_bool(self, key, value, **kwargs):
    self.bools[key] = value


def _cp(brand="mazda", op_long=True, alpha_avail=True):
  cp = structs.CarParams()
  cp.brand = brand
  cp.openpilotLongitudinalControl = op_long
  cp.alphaLongitudinalAvailable = alpha_avail
  return cp


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
    params = FakeParams(AlphaLongitudinalEnabled=True)
    m = AlphaLongToggleMonitor(_cp(op_long=True), params)
    m.update_params()
    cc_sp = _step(m)
    assert not cc_sp.radarHandBack
    assert not params.get_bool("OnroadCycleRequested")

  def test_enable_direction_cycles_immediately(self):
    params = FakeParams(AlphaLongitudinalEnabled=True)
    m = AlphaLongToggleMonitor(_cp(op_long=False), params)
    m.update_params()
    cc_sp = _step(m)
    assert not cc_sp.radarHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_disable_runs_handback_until_radar_returns(self):
    params = FakeParams(AlphaLongitudinalEnabled=False)
    m = AlphaLongToggleMonitor(_cp(op_long=True), params)
    m.update_params()
    # radar still silent: hand-back asserted, no cycle yet
    for _ in range(50):
      cc_sp = _step(m, acc_faulted=False)
      assert cc_sp.radarHandBack
      assert not params.get_bool("OnroadCycleRequested")
    # stock radar heard again: cycle requested
    cc_sp = _step(m, acc_faulted=True)
    assert cc_sp.radarHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_disable_times_out_to_cycle(self):
    params = FakeParams(AlphaLongitudinalEnabled=False)
    m = AlphaLongToggleMonitor(_cp(op_long=True), params)
    m.update_params()
    for _ in range(HANDBACK_TIMEOUT_FRAMES):
      _step(m, acc_faulted=False)
    assert params.get_bool("OnroadCycleRequested")

  def test_waits_for_disengagement(self):
    params = FakeParams(AlphaLongitudinalEnabled=False)
    m = AlphaLongToggleMonitor(_cp(op_long=True), params)
    m.update_params()
    cc_sp = _step(m, enabled=True)
    assert not cc_sp.radarHandBack
    # once started, engagement no longer pauses the sequence
    _step(m, enabled=False)
    cc_sp = _step(m, enabled=True)
    assert cc_sp.radarHandBack

  def test_non_mazda_disable_cycles_immediately(self):
    params = FakeParams(AlphaLongitudinalEnabled=False)
    m = AlphaLongToggleMonitor(_cp(brand="toyota", op_long=True), params)
    m.update_params()
    cc_sp = _step(m)
    assert not cc_sp.radarHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_unavailable_never_acts(self):
    params = FakeParams(AlphaLongitudinalEnabled=True)
    m = AlphaLongToggleMonitor(_cp(op_long=False, alpha_avail=False), params)
    m.update_params()
    cc_sp = _step(m)
    assert not cc_sp.radarHandBack
    assert not params.get_bool("OnroadCycleRequested")

  def test_cycle_requested_only_once(self):
    params = FakeParams(AlphaLongitudinalEnabled=True)
    m = AlphaLongToggleMonitor(_cp(op_long=False), params)
    m.update_params()
    _step(m)
    params.put_bool("OnroadCycleRequested", False)  # hardwared consumed it
    _step(m)
    assert not params.get_bool("OnroadCycleRequested")
