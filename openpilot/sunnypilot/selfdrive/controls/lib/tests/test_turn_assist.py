"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.turn_assist import (
  TurnAssistController, HOLD_HARD_SPEED, HOLD_RELEASE_SPEED, HOLD_STANDSTILL_TIMEOUT,
  HOLD_SWEPT_EXIT, TURN_LEAD_MIN_SPEED, TURN_LEAD_FULL_SPEED, plan_dual_probe,
)


def make_cs(v_ego, left=False, right=False, torque=0.0, pressed=False, a_ego=0.0):
  return SimpleNamespace(vEgo=v_ego, aEgo=a_ego, leftBlinker=left, rightBlinker=right,
                         steeringPressed=pressed, steeringTorque=torque)


def make_model(kappa=0.0, straight_len=0.0, reach=15.0, lane_change=False):
  """A 33-point plan: straight for straight_len meters, then a constant-curvature arc.
  Positive kappa arcs right (positive y), matching the controlsd sign convention."""
  xs, ys = [], []
  for i in range(33):
    s = reach * i / 32
    if s <= straight_len or kappa == 0.0:
      xs.append(s)
      ys.append(0.0)
    else:
      th = (s - straight_len) * abs(kappa)
      r = 1.0 / abs(kappa)
      xs.append(straight_len + r * math.sin(th))
      ys.append(math.copysign(r * (1 - math.cos(th)), kappa))
  state = log.LaneChangeState.laneChangeStarting if lane_change else log.LaneChangeState.off
  return SimpleNamespace(meta=SimpleNamespace(laneChangeState=state),
                         position=SimpleNamespace(x=xs, y=ys))


class TestTurnAssist(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.params.put_bool("LowSpeedTurnAssist", True, block=True)
    self.ta = TurnAssistController(SimpleNamespace(steerControlType='torque'))
    self.ta.get_params()

  def run_frames(self, n, cs, model, cmd, measured, lat_active=True):
    out = cmd
    for _ in range(n):
      out = self.ta.update(cs, lat_active, model, cmd, measured)
    return out

  def test_disabled_is_passthrough(self):
    self.params.put_bool("LowSpeedTurnAssist", False, block=True)
    self.ta.get_params()
    out = self.run_frames(10, make_cs(1.0, right=True), make_model(kappa=0.1), 0.002, 0.0)
    assert out == 0.002 and self.ta.hold == 0.0

  def test_blinker_pause_wins(self):
    self.params.put_bool("BlinkerPauseLateralControl", True, block=True)
    self.ta.get_params()
    assert not self.ta.enabled

  def test_standstill_prewind_ratchets_from_plan(self):
    # stopped at a stop line, right blinker, the model action is blind but the plan
    # already bends right just ahead
    model = make_model(kappa=0.1, straight_len=0.5)
    out = self.run_frames(50, make_cs(0.0, right=True), model, 0.002, 0.0)
    assert self.ta.hold > 0.01
    assert out == pytest.approx(self.ta.hold)

  def test_prewind_gated_on_distant_corner(self):
    # same corner but starting 8 m out: the onset gate must block the plan pre-wind
    # (command held at zero so only the plan path could raise the hold)
    model = make_model(kappa=0.1, straight_len=8.0)
    self.run_frames(50, make_cs(0.0, right=True), model, 0.0, 0.0)
    assert self.ta.hold == 0.0

  def test_floor_survives_command_collapse_below_hard_speed(self):
    model = make_model(kappa=0.1, straight_len=0.5)
    self.run_frames(50, make_cs(0.0, right=True), model, 0.002, 0.0)
    hold = self.ta.hold
    assert hold > 0.01
    # pull away at creep speed with the command collapsed to zero: the floor holds
    out = self.run_frames(100, make_cs(HOLD_HARD_SPEED * 0.8, right=True), make_model(), 0.0, hold)
    assert self.ta.hold == pytest.approx(hold)
    assert out == pytest.approx(hold)

  def test_handoff_when_action_takes_over(self):
    model = make_model(kappa=0.1, straight_len=0.5)
    self.run_frames(50, make_cs(0.0, right=True), model, 0.002, 0.0)
    hold = self.ta.hold
    # model wakes to 90% of the hold and sustains it: complete handoff within 0.3 s + margin
    self.run_frames(40, make_cs(2.0, right=True), model, 0.9 * hold, hold)
    assert self.ta.hold == 0.0
    assert self.ta.done

  def test_opposite_command_releases(self):
    model = make_model(kappa=0.1, straight_len=0.5)
    self.run_frames(50, make_cs(0.0, right=True), model, 0.002, 0.0)
    assert self.ta.hold > 0.0
    self.ta.update(make_cs(1.5, right=True), True, make_model(), -0.02, 0.05)
    assert self.ta.hold == 0.0 and self.ta.done

  def test_exit_decay_after_swept_heading(self):
    model = make_model(kappa=0.1, straight_len=0.5)
    self.run_frames(50, make_cs(0.0, right=True), model, 0.002, 0.0)
    hold = self.ta.hold
    self.ta.swept = HOLD_SWEPT_EXIT + 0.1  # past the exit threshold
    # even below the hard speed, the fast decay drains a floor the model undercuts
    self.run_frames(100, make_cs(0.8, right=True), make_model(), 0.001, hold)
    assert self.ta.hold < 0.5 * hold

  def test_nudge_to_commit_captures_wound_wheel(self):
    # rolling left gap at 3.5 m/s: driver pushes left (positive torque) with the wheel
    # already wound left (negative curvature). Capture is immediate, not rate-limited.
    cs = make_cs(3.5, left=True, torque=250.0, pressed=True)
    out = self.ta.update(cs, True, make_model(), 0.0, -0.02)
    assert self.ta.hold == pytest.approx(-0.02)
    assert out == pytest.approx(-0.02)

  def test_bracing_against_machine_wind_not_captured(self):
    # opposite-direction driver torque (bracing) must not capture
    cs = make_cs(3.5, left=True, torque=-250.0, pressed=True)
    self.ta.update(cs, True, make_model(), 0.0, -0.02)
    assert self.ta.hold == 0.0

  def test_release_speed_clears_everything(self):
    self.ta.hold = 0.08
    self.ta.done = True
    self.ta.update(make_cs(HOLD_RELEASE_SPEED + 0.5, right=True), True, make_model(), 0.0, 0.0)
    assert self.ta.hold == 0.0 and not self.ta.done

  def test_blinker_flip_clears_hold(self):
    self.ta.hold = 0.08
    self.ta.update(make_cs(1.0, left=True), True, make_model(), 0.0, 0.0)
    assert self.ta.hold == 0.0

  def test_standstill_timeout_drops_hold(self):
    self.ta.hold = 0.08
    self.ta.standstill_t = HOLD_STANDSTILL_TIMEOUT - 0.05
    self.run_frames(10, make_cs(0.0, right=True), make_model(), 0.0, 0.0)
    assert self.ta.hold == 0.0

  def test_turn_lead_fires_rolling_toward_corner(self):
    # 4 m/s (full lead authority, below the hold release speed) toward a right corner
    # starting ~2 m ahead: the constant-time probe sees it, the meter-anchored action
    # does not yet
    model = make_model(kappa=0.08, straight_len=2.0, reach=20.0)
    cs = make_cs(4.0, right=True)
    out = self.ta.update(cs, True, model, 0.001, 0.0)
    assert out > 0.005
    assert self.ta.lead_applied > 0.0
    # and the applied lead is captured into the hold for deceleration continuity
    assert self.ta.hold > 0.0

  def test_turn_lead_still_fires_above_hold_release_speed(self):
    # the lead's 3-7 m/s range straddles the hold's 4.47 m/s release: clearing the
    # hold state at release speed must not silence the lead
    model = make_model(kappa=0.08, straight_len=2.0, reach=20.0)
    out = self.ta.update(make_cs(5.0, right=True), True, model, 0.001, 0.0)
    assert self.ta.lead_applied > 0.0
    assert out == pytest.approx(self.ta.lead_applied)
    assert self.ta.hold == 0.0  # no capture above release speed

  def test_turn_lead_vetoed_when_model_opposes(self):
    model = make_model(kappa=0.08, straight_len=2.0, reach=20.0)
    out = self.ta.update(make_cs(5.0, right=True), True, model, -0.01, 0.0)
    assert out == -0.01

  def test_turn_lead_vetoed_when_stopping_short(self):
    model = make_model(kappa=0.08, straight_len=2.0, reach=20.0)
    # braking hard enough to stop well before the probe distance
    out = self.ta.update(make_cs(5.0, right=True, a_ego=-2.5), True, model, 0.001, 0.0)
    assert out == 0.001

  def test_turn_lead_off_during_lane_change(self):
    model = make_model(kappa=0.08, straight_len=2.0, reach=20.0, lane_change=True)
    out = self.ta.update(make_cs(5.0, right=True), True, model, 0.001, 0.0)
    assert out == 0.001

  def test_turn_lead_off_below_min_speed(self):
    model = make_model(kappa=0.08, straight_len=2.0, reach=20.0)
    out = self.ta.update(make_cs(TURN_LEAD_MIN_SPEED - 0.5, right=True), True, model, 0.001, 0.0)
    assert self.ta.lead_applied == 0.0
    assert out == pytest.approx(max(0.001, self.ta.hold))

  def test_turn_lead_engagement_fade(self):
    # once the measured curvature reaches the lead, authority fades to zero
    model = make_model(kappa=0.08, straight_len=2.0, reach=20.0)
    cs = make_cs(TURN_LEAD_FULL_SPEED, right=True)
    self.ta.update(cs, True, model, 0.001, 0.0)
    lead_fresh = self.ta.lead_applied
    assert lead_fresh > 0.0
    self.ta.reset()
    self.ta.update(cs, True, model, 0.001, lead_fresh)  # wheel already at the lead value
    assert self.ta.lead_applied < lead_fresh * 0.6

  def test_plan_probe_sign_disagreement_reads_zero(self):
    # near probe straight, far probe arcing: the S-bend must contribute nothing
    model = make_model(kappa=0.1, straight_len=0.5)
    xs = list(model.position.x)
    ys = [-y for y in model.position.y[:16]] + list(model.position.y[16:])
    s_model = SimpleNamespace(meta=model.meta, position=SimpleNamespace(x=xs, y=ys))
    assert plan_dual_probe(s_model, 4.0, 7.0) == 0.0
