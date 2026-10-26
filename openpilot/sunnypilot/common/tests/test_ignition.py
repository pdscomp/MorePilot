"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.common import ignition
from openpilot.sunnypilot.common.ignition import get_ignition_state

PandaType = log.PandaState.PandaType


def ps(can=False, line=False, panda_type=PandaType.tres):
  return SimpleNamespace(pandaType=panda_type, ignitionCan=can, ignitionLine=line)


@pytest.fixture(autouse=True)
def _fresh_state():
  ignition._ignition_can_seen = False
  yield
  ignition._ignition_can_seen = False


class TestIgnition:
  def test_can_on(self):
    assert get_ignition_state([ps(can=True)])
    assert get_ignition_state([ps(can=True, line=True)])

  def test_can_drop_with_line_high_is_off_immediately(self):
    assert get_ignition_state([ps(can=True, line=True)])
    assert not get_ignition_state([ps(can=False, line=True)])
    # Mazda's slow line must not resurrect ignition later either.
    assert not get_ignition_state([ps(can=False, line=True)])

  def test_can_drop_with_line_low_is_off_immediately(self):
    assert get_ignition_state([ps(can=True, line=True)])
    assert not get_ignition_state([ps(can=False, line=False)])

  def test_line_only_car_follows_the_line(self):
    assert get_ignition_state([ps(line=True)])
    assert not get_ignition_state([ps(line=False)])
    assert get_ignition_state([ps(line=True)])
    assert not get_ignition_state([ps(line=True, panda_type=PandaType.unknown)])
    assert get_ignition_state([ps(line=True)])

  def test_no_valid_panda_preserves_can_priority(self):
    assert get_ignition_state([ps(can=True, line=True)])
    assert not get_ignition_state([ps(can=True, line=True, panda_type=PandaType.unknown)])
    assert not get_ignition_state([])
    # Do not reset to line-only semantics after CAN ownership was established.
    assert not get_ignition_state([ps(line=True)])

  def test_can_recovery_reasserts_ignition(self):
    assert get_ignition_state([ps(can=True, line=True)])
    assert not get_ignition_state([ps(can=False, line=True)])
    assert get_ignition_state([ps(can=True, line=True)])

  def test_any_panda_counts(self):
    states = [ps(can=False, line=False), ps(can=True, line=False)]
    assert get_ignition_state(states)
    states = [ps(can=False, line=True), ps(can=False, line=False)]
    assert not get_ignition_state(states)
