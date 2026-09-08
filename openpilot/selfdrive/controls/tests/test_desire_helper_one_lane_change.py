from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode


class TestOneLaneChange(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.params.put("AutoLaneChangeTimer", AutoLaneChangeMode.NUDGE, block=True)
    self.params.put_bool("OneLaneChange", True, block=True)
    self.dh = DesireHelper()
    self.dh.alc.read_params()
    self.cs = SimpleNamespace(
      vEgo=25.0,
      leftBlinker=True,
      rightBlinker=False,
      steeringPressed=True,
      steeringTorque=1.0,
      leftBlindspot=False,
      rightBlindspot=False,
      brakePressed=False,
    )

  def update(self, probability=1.0):
    self.dh.update(self.cs, lateral_active=True, lane_change_prob=probability)

  def start_left_change(self):
    self.update()
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.laneChangeStarting

  def finish_change_with_signal_held(self):
    for _ in range(11):
      if self.dh.lane_change_state == log.LaneChangeState.preLaneChange:
        break
      self.update(probability=0.0)
    assert self.dh.lane_change_state == log.LaneChangeState.preLaneChange

  def test_enabled_allows_only_one_change_until_signal_released(self):
    self.start_left_change()
    self.finish_change_with_signal_held()
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.off

  def test_release_rearms_enabled_setting(self):
    self.start_left_change()
    self.finish_change_with_signal_held()
    self.cs.leftBlinker = False
    self.update()
    self.cs.leftBlinker = True
    self.update()
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.laneChangeStarting

  def test_both_blinkers_preserve_consumed_direction(self):
    self.start_left_change()
    self.finish_change_with_signal_held()
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.off

    self.cs.rightBlinker = True
    self.update()
    assert self.dh.lane_change_completed_direction == log.LaneChangeDirection.left

    self.cs.rightBlinker = False
    self.update()
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.off

  def test_direct_direction_switch_rearms_opposite_signal(self):
    self.start_left_change()
    self.finish_change_with_signal_held()
    self.update()

    self.cs.leftBlinker = False
    self.cs.rightBlinker = True
    self.cs.steeringTorque = -1.0
    self.update()
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.laneChangeStarting
    assert self.dh.lane_change_completed_direction == log.LaneChangeDirection.right

  def test_direct_direction_switch_disabled_keeps_legacy_rearm_rule(self):
    self.params.put_bool("OneLaneChange", False, block=True)
    self.dh.alc.read_params()
    self.start_left_change()
    # stall the maneuver past the timeout with the signal held: state off, no release edge
    for _ in range(210):
      self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.off

    self.cs.leftBlinker = False
    self.cs.rightBlinker = True
    self.cs.steeringTorque = -1.0
    for _ in range(5):
      self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.off

  def test_enabling_setting_honors_start_already_seen(self):
    self.params.put_bool("OneLaneChange", False, block=True)
    self.dh.alc.read_params()
    self.start_left_change()
    self.finish_change_with_signal_held()

    self.params.put_bool("OneLaneChange", True, block=True)
    self.dh.alc.read_params()
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.off

  def test_declared_default_is_disabled(self):
    self.params.remove("OneLaneChange")
    assert not self.params.get("OneLaneChange", return_default=True)
    self.dh.alc.read_params()
    assert not self.dh.alc.one_lane_change

  def test_automatic_start_is_limited_to_one(self):
    self.params.put("AutoLaneChangeTimer", AutoLaneChangeMode.NUDGELESS, block=True)
    self.dh.alc.read_params()
    self.cs.steeringPressed = False
    self.cs.steeringTorque = 0.0

    self.update()
    for _ in range(3):
      self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.laneChangeStarting

    self.finish_change_with_signal_held()
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.off

  def test_blocked_prestart_does_not_consume_signal(self):
    self.cs.leftBlindspot = True
    self.update()
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.preLaneChange
    assert self.dh.lane_change_completed_direction == log.LaneChangeDirection.none

    self.cs.leftBlindspot = False
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.laneChangeStarting
    assert self.dh.lane_change_completed_direction == log.LaneChangeDirection.left

  def test_disabling_setting_allows_repeat_without_signal_release(self):
    self.start_left_change()
    self.finish_change_with_signal_held()
    self.params.put_bool("OneLaneChange", False, block=True)
    self.dh.alc.read_params()
    self.update()
    assert self.dh.lane_change_state == log.LaneChangeState.laneChangeStarting
