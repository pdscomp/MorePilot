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
  ignition._ignition_can_drop_expired = False
  yield
  ignition._ignition_can_seen = False
  ignition._ignition_can_drop_since = None
  ignition._ignition_can_drop_expired = False


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
    # panda gone entirely -> offroad immediately (no debounce for link loss), but the
    # dropout clock starts, so the episode keeps running through the outage
    assert not get_ignition_state([ps(can=True, line=True, panda_type=PandaType.unknown)], now=T0 + 0.1)
    assert not get_ignition_state([], now=T0 + 0.2)
    # inside the debounce window a line-only frame still holds ignition (the episode is
    # young); past it, the slow line can no longer resurrect
    assert get_ignition_state([ps(line=True)], now=T0 + 0.3)
    assert not get_ignition_state([ps(line=True)], now=T0 + CAN_IGNITION_DROP_DEBOUNCE_S + 0.4)

  def test_reconnect_after_long_outage_cannot_manufacture_ignition(self):
    assert get_ignition_state([ps(can=True, line=True)], now=T0)
    # the panda link dies for longer than the debounce
    assert not get_ignition_state([ps(can=True, line=True, panda_type=PandaType.unknown)], now=T0 + 0.1)
    assert not get_ignition_state([], now=T0 + 2.6)
    # reconnect with every signal low: no fresh 2 s window, off immediately
    assert not get_ignition_state([ps(can=False, line=False)], now=T0 + 2.7)
    # Mazda's slow line is high at reconnect: still no resurrection, CAN owns ignition
    assert not get_ignition_state([ps(can=False, line=True)], now=T0 + 2.8)
    # actual CAN recovery reasserts
    assert get_ignition_state([ps(can=True, line=True)], now=T0 + 3.0)
    # and the next real drop debounces normally from its own start
    assert get_ignition_state([ps(can=False, line=True)], now=T0 + 3.1)
    assert not get_ignition_state([ps(can=False, line=True)], now=T0 + 3.1 + CAN_IGNITION_DROP_DEBOUNCE_S + 0.1)

  def test_reconnect_inside_window_completes_the_episode(self):
    assert get_ignition_state([ps(can=True, line=True)], now=T0)
    assert not get_ignition_state([ps(can=True, line=True, panda_type=PandaType.unknown)], now=T0 + 0.1)
    # total episode stays under the debounce, so the low-CAN reconnect holds ignition
    assert get_ignition_state([ps(can=False, line=False)], now=T0 + 0.6)
    # the clock did not restart at reconnect: the episode still expires 2 s after it began
    assert not get_ignition_state([ps(can=False, line=False)], now=T0 + 0.1 + CAN_IGNITION_DROP_DEBOUNCE_S + 0.1)

  def test_debounce_boundary_is_strict(self):
    assert get_ignition_state([ps(can=True, line=True)], now=T0)
    # the drop clock starts at the first observed low frame
    assert get_ignition_state([ps(can=False, line=False)], now=T0 + 0.1)
    assert get_ignition_state([ps(can=False, line=False)], now=T0 + 0.1 + CAN_IGNITION_DROP_DEBOUNCE_S - 0.001)
    # exactly at the boundary the drop has persisted long enough
    assert not get_ignition_state([ps(can=False, line=False)], now=T0 + 0.1 + CAN_IGNITION_DROP_DEBOUNCE_S)

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
