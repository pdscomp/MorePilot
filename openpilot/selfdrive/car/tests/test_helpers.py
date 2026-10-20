"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from openpilot.cereal import custom
from openpilot.selfdrive.car.helpers import convert_carControlSP


class TestConvertCarControlSP:
  def test_real_message_keeps_consumed_fields_and_drops_wire_telemetry(self):
    """A real CarControlSP reader: the fields card consumes survive into the opendbc
    dataclass; the fork/target wire telemetry (turnAssist, torqueV2Mode,
    zoompilot) does not, and the wire bytes still carry it."""
    msg = custom.CarControlSP.new_message()
    msg.mads.enabled = True
    msg.leadOne.dRel = 12.5
    msg.leadTwo.dRel = 30.0
    msg.intelligentCruiseButtonManagement.sendButton = 'increase'
    msg.intelligentCruiseButtonManagement.vTarget = 18.0
    msg.turnAssist.holdCurvature = 0.01
    msg.torqueV2Mode = 'modeB'
    msg.zoompilot.lateralExtension = 'jerkAware'
    msg.init('params', 1)
    msg.params[0].key = 'unit-test'
    msg.params[0].type = 'string'
    msg.params[0].value = b'test-value'

    out = convert_carControlSP(msg.as_reader())
    assert out.mads.enabled is True
    assert out.leadOne.dRel == pytest.approx(12.5)
    assert out.leadTwo.dRel == pytest.approx(30.0)
    assert out.intelligentCruiseButtonManagement.sendButton == 'increase'
    assert out.intelligentCruiseButtonManagement.vTarget == pytest.approx(18.0)
    assert out.params[0]['key'] == 'unit-test'
    assert out.params[0]['value'] == b'test-value'
    # wire telemetry, not opendbc actuator inputs; the zoompilot nest IS carried for icbmHold,
    # which opendbc's ICBM interface reads (lateralExtension inside it is wire-only)
    assert not hasattr(out, 'torqueV2Mode')
    assert not hasattr(out, 'turnAssist')
    assert out.zoompilot.icbmHold is False

    with custom.CarControlSP.from_bytes(msg.to_bytes()) as decoded:
      assert decoded.torqueV2Mode == custom.CarControlSP.TorqueV2Mode.modeB
      assert decoded.turnAssist.holdCurvature == pytest.approx(0.01)
      assert decoded.zoompilot.lateralExtension == 'jerkAware'

  def test_capnp_only_fields_do_not_crash(self):
    """capnp CarControlSP can carry substructs and fields that have no field on the opendbc
    dataclass; the converter must drop them instead of passing them as kwargs. 2026-10-04:
    mads.lateralHeld was one, and card crashed on its first control step of every drive.
    zoompilot.laneChangeSmoothing is one now, beside the icbmHold opendbc does carry."""
    msg = custom.CarControlSP.new_message()
    msg.mads.enabled = True
    msg.zoompilot.laneChangeSmoothing.jerkFactor = 0.5
    msg.zoompilot.icbmHold = True
    msg.leadOne.dRel = 12.5
    struct_dict = {**msg.as_reader().to_dict(), 'capnpOnlySubstruct': {'gain': 0.5}}

    out = convert_carControlSP(SimpleNamespace(to_dict=lambda: struct_dict))
    assert out.mads.enabled is True
    assert out.zoompilot.icbmHold is True
    assert abs(out.leadOne.dRel - 12.5) < 1e-6
