"""Rebase boundary contracts: the CarControlSP schema/converter split, and the committed
contract-C lifecycle classes (alpha monitor, HardwaredExt, hand-back server) sequenced
together. The schema test pins which fork/target fields are wire telemetry only; the
lifecycle tests pin the withdrawn-offroad toggle and forced-offroad late-recovery paths
through the real classes, not isolated methods."""
from contextlib import nullcontext
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.sunnypilot.car.stock_ecu import StockEcuState
from openpilot.cereal import custom
from openpilot.selfdrive.car.helpers import convert_carControlSP
from openpilot.sunnypilot.selfdrive.car.alpha_long_toggle import AlphaLongToggleMonitor
from openpilot.sunnypilot.selfdrive.car.stock_ecu_handback import StockEcuHandBackServer
import openpilot.sunnypilot.selfdrive.car.stock_ecu_handback as handback_protocol
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeClock, FakeParams
from openpilot.sunnypilot.system.hardware.hardwared_ext import HardwaredExt


def test_target_schema_and_fork_telemetry_convert_together():
  msg = custom.CarControlSP.new_message()
  msg.mads.enabled = True
  msg.mads.lateralHeld = True
  msg.leadOne.dRel = 12.5
  msg.turnAssist.holdCurvature = 0.01
  msg.torqueV2Mode = 'modeB'
  msg.zoompilot.lateralExtension = 'jerkAware'
  msg.intelligentCruiseButtonManagement.sendButton = 'increaseHold'
  msg.intelligentCruiseButtonManagement.vTarget = 18.0
  msg.leadTwo.dRel = 30.0
  msg.init('params', 1)
  msg.params[0].key = 'unit-test'
  msg.params[0].type = 'string'
  msg.params[0].value = b'test-value'
  out = convert_carControlSP(msg.as_reader())
  assert out.mads.enabled
  assert out.leadOne.dRel == pytest.approx(12.5)
  assert out.intelligentCruiseButtonManagement.sendButton == 'increaseHold'
  assert out.intelligentCruiseButtonManagement.vTarget == pytest.approx(18.0)
  assert out.leadTwo.dRel == pytest.approx(30.0)
  assert out.params[0]['key'] == 'unit-test'
  assert out.params[0]['type'] == 'string'
  assert out.params[0]['value'] == b'test-value'
  # Selected boundary: these are wire telemetry, not opendbc actuator inputs.
  assert not hasattr(out, 'torqueV2Mode')
  assert not hasattr(out, 'turnAssist')
  assert not hasattr(out.mads, 'lateralHeld')
  with custom.CarControlSP.from_bytes(msg.to_bytes()) as decoded:
    assert decoded.torqueV2Mode == custom.CarControlSP.TorqueV2Mode.modeB
    assert decoded.turnAssist.holdCurvature == pytest.approx(0.01)
    assert decoded.zoompilot.lateralExtension == 'jerkAware'
    assert decoded.mads.lateralHeld


def test_withdrawn_offroad_does_not_abandon_pending_alpha_toggle():
  params = FakeParams(AlphaLongitudinalEnabled=True, OffroadModeRequested=True)
  cp = SimpleNamespace(brand='mazda', alphaLongitudinalAvailable=True,
                       openpilotLongitudinalControl=False)
  monitor = AlphaLongToggleMonitor(cp, params)
  cs = structs.CarState()
  cc = structs.CarControl()
  monitor.update_params()
  monitor.update(cs, cc, structs.CarControlSP())
  assert not params.get_bool('OnroadCycleRequested')
  params.put_bool('OffroadModeRequested', False)
  monitor.update_params()
  monitor.update(cs, cc, structs.CarControlSP())
  assert params.get_bool('OnroadCycleRequested')
  assert not params.get_bool('OffroadMode')


@pytest.mark.parametrize('monitor_first', [False, True])
def test_forced_offroad_late_recovery_sticks_with_card_monitor(monitor_first, monkeypatch):
  # Pure-state ordering test; the real shared lock is covered with real Params in
  # test_stock_ecu_handback.py's two-process case.
  monkeypatch.setattr(handback_protocol, '_request_lock', lambda params: nullcontext())
  params = FakeParams(AlphaLongitudinalEnabled=True, OffroadModeRequested=True)
  cp = SimpleNamespace(brand='mazda', alphaLongitudinalAvailable=True,
                       openpilotLongitudinalControl=True)
  monitor = AlphaLongToggleMonitor(cp, params)
  hardware = HardwaredExt(params)
  clock = FakeClock()
  hardware.handback.now = clock
  server = StockEcuHandBackServer(params)
  cs, cc = structs.CarState(), structs.CarControl()

  def tick(state):
    monitor.update_params()
    if not monitor_first:
      hardware.update(started=True)
    command = structs.CarControlSP()
    monitor.update(cs, cc, command)
    if monitor_first:
      hardware.update(started=True)
    server.update_params()
    server.update(False, state, command)
    return command

  tick(StockEcuState.RESTORING)
  tick(StockEcuState.FAILED)
  clock.t = 30.0
  for _ in range(3):
    tick(StockEcuState.FAILED)
    assert not params.get_bool('OffroadMode')
  for _ in range(3):
    tick(StockEcuState.RESTORED)
  assert params.get_bool('OffroadMode')
  assert params.get_bool('OffroadModeRequested')
  for _ in range(5):
    tick(StockEcuState.RESTORED)
    assert params.get_bool('OffroadMode')
  params.put_bool('OffroadModeRequested', False)
  hardware.update(started=False)
  assert not params.get_bool('OffroadMode')
