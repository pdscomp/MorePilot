"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.common import ignition
from openpilot.sunnypilot.common.ignition import CAN_IGNITION_DROP_DEBOUNCE_S
from openpilot.sunnypilot.common.ignition import get_ignition_state

PandaType = log.PandaState.PandaType

T0 = 1000.0


def ps(can=False, line=False, panda_type=PandaType.tres):
  return SimpleNamespace(pandaType=panda_type, ignitionCan=can, ignitionLine=line)


@pytest.fixture(autouse=True)
def _fresh_state():
  ignition._ignition_can_seen = False
  ignition._ignition_can_drop_since = None
  yield
  ignition._ignition_can_seen = False
  ignition._ignition_can_drop_since = None


class TestIgnition:
  def test_can_on(self):
    assert get_ignition_state([ps(can=True)], now=T0)
    assert get_ignition_state([ps(can=True, line=True)], now=T0 + 0.1)

  def test_can_drop_with_line_high_goes_off_after_debounce(self):
    assert get_ignition_state([ps(can=True, line=True)], now=T0)
    # the dropout is held through the debounce window...
    assert get_ignition_state([ps(can=False, line=True)], now=T0)
    assert get_ignition_state([ps(can=False, line=True)], now=T0 + CAN_IGNITION_DROP_DEBOUNCE_S - 0.1)
    assert not get_ignition_state([ps(can=False, line=True)], now=T0 + CAN_IGNITION_DROP_DEBOUNCE_S + 0.1)
    # Mazda's slow line must not resurrect ignition later either.
    assert not get_ignition_state([ps(can=False, line=True)], now=T0 + 30.0)

  def test_can_drop_with_line_low_goes_off_after_debounce(self):
    assert get_ignition_state([ps(can=True, line=True)], now=T0)
    assert get_ignition_state([ps(can=False, line=False)], now=T0)
    assert not get_ignition_state([ps(can=False, line=False)], now=T0 + CAN_IGNITION_DROP_DEBOUNCE_S + 0.1)

  def test_short_can_dropout_is_ignored(self):
    assert get_ignition_state([ps(can=True, line=True)], now=T0)
    for dt in (0.4, 0.8, 1.2):
      assert get_ignition_state([ps(can=False, line=True)], now=T0 + dt)
    # recovery inside the window resets the drop timer
    assert get_ignition_state([ps(can=True, line=True)], now=T0 + 1.5)
    assert get_ignition_state([ps(can=False, line=True)], now=T0 + 1.6)
    assert get_ignition_state([ps(can=False, line=True)], now=T0 + 1.6 + CAN_IGNITION_DROP_DEBOUNCE_S - 0.2)
    assert not get_ignition_state([ps(can=False, line=True)], now=T0 + 1.6 + CAN_IGNITION_DROP_DEBOUNCE_S + 0.2)

  def test_line_only_car_follows_the_line(self):
    assert get_ignition_state([ps(line=True)], now=T0)
    assert not get_ignition_state([ps(line=False)], now=T0 + 0.1)
    assert get_ignition_state([ps(line=True)], now=T0 + 0.2)
    assert not get_ignition_state([ps(line=True, panda_type=PandaType.unknown)], now=T0 + 0.3)
    assert get_ignition_state([ps(line=True)], now=T0 + 0.4)

  def test_no_valid_panda_preserves_can_priority(self):
    assert get_ignition_state([ps(can=True, line=True)], now=T0)
    # panda gone entirely -> offroad immediately (no debounce for link loss)
    assert not get_ignition_state([ps(can=True, line=True, panda_type=PandaType.unknown)], now=T0 + 0.1)
    assert not get_ignition_state([], now=T0 + 0.2)
    # line-only must not resurrect ignition once CAN owns it, before or after the debounce
    assert get_ignition_state([ps(line=True)], now=T0 + 0.3)
    assert not get_ignition_state([ps(line=True)], now=T0 + CAN_IGNITION_DROP_DEBOUNCE_S + 0.4)

  def test_can_recovery_reasserts_ignition(self):
    assert get_ignition_state([ps(can=True, line=True)], now=T0)
    assert get_ignition_state([ps(can=False, line=True)], now=T0 + 1.0)
    assert get_ignition_state([ps(can=True, line=True)], now=T0 + 2.0)
    # a later real drop still goes offroad after the debounce
    assert get_ignition_state([ps(can=False, line=True)], now=T0 + 3.0)
    assert not get_ignition_state([ps(can=False, line=True)], now=T0 + 3.0 + CAN_IGNITION_DROP_DEBOUNCE_S + 0.1)

  def test_any_panda_counts(self):
    states = [ps(can=False, line=False), ps(can=True, line=False)]
    assert get_ignition_state(states, now=T0)
    states = [ps(can=False, line=True), ps(can=False, line=False)]
    assert get_ignition_state(states, now=T0 + 1.0)
    assert not get_ignition_state(states, now=T0 + 1.0 + CAN_IGNITION_DROP_DEBOUNCE_S + 0.1)
