"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The torque tune swap on a model change, against the real controllers: the big model's v1
# steering and the small model's v2 idle with state from an earlier curve, as after a join.
# On the 2026-09-29 drives a hand-back while steering swapped to the idle v2 at once and the
# commanded torque stepped by 0.09 to 0.19 in one tick. The tune that steers now carries the
# small model to the first frame lateral is inactive.

from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque as LatControlTorqueV1
from openpilot.sunnypilot.selfdrive.controls.controls_lateral_zp import TUNE_SWAP_INACTIVE_FRAMES, ControlsLateralZP
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v2 import LatControlTorque as LatControlTorqueV2
from openpilot.sunnypilot.selfdrive.controls.lib.tests.torque_harness import FRICTION, LAT_DELAY, LP, VM, make_cs, make_lac, step

V_EGO = 10.0
LAT_ACCEL = 0.8           # a steady curve, and the car following it
HAND_BACK = 200           # the frame the small model's first modelV2 arrives, v1 settled
DRIVE_ARGS = (VM, LP, False, LAT_ACCEL / V_EGO ** 2, None, False, LAT_DELAY)


class FakeSubMaster:
  """Just the pieces of controlsd's sm that update_lateral_assist and the swap read."""

  def __init__(self, cs, big: bool, maneuver: bool = False):
    self.messages = {'carState': cs, 'modelV2': SimpleNamespace(big=big)}
    self.valid = {'lateralManeuverPlan': maneuver}

  def __getitem__(self, name):
    return self.messages[name]


def controls():
  """controlsd's two controllers, the big one steering, the small one idle with the state it
  had when it last steered, on another curve at another speed."""
  small, big = make_lac(LatControlTorqueV2, friction=FRICTION), make_lac(LatControlTorqueV1, friction=FRICTION)
  for _ in range(300):
    step(small, make_cs(v_ego=20.0, lat_accel=-1.5), -1.5 / 20.0 ** 2)
  ctl = SimpleNamespace(_lacs=(small, big), _lac_by_size={False: small, True: big}, LaC=big, _inactive_frames=0,
                        turn_assist=TurnAssistController(make_cp()), lane_change_smoothing=LaneChangeSmoothing())
  ctl.note_lat_active = lambda active: ControlsExt.note_lat_active(ctl, active)
  return ctl


def drive(ctl, hand_back: bool, inactive_at: int | None = None, frames: int = 300,
          inactive: set[int] | None = None, maneuvers: set[int] | None = None,
          counts: list[int] | None = None) -> list[float]:
  """Commanded torque every frame, as controlsd runs it: state_control's update_lateral_assist
  (which counts the inactive frames), the controller update, then the swap at the end of the
  frame. Lateral is inactive from `inactive_at` on, and on the frames in `inactive`; the frames
  in `maneuvers` run the lateral maneuver mode's scripted path."""
  out = []
  for i in range(frames):
    active = (inactive_at is None or i < inactive_at) and i not in (inactive or ())
    torque, _, _ = ctl.LaC.update(active, make_cs(V_EGO, LAT_ACCEL), *DRIVE_ARGS)
    ControlsLateralZP.note_lat_active(ctl, active)
    out.append(torque)
    ControlsLateralZP.select_lateral_control(ctl, {'modelV2': SimpleNamespace(big=not (hand_back and i >= HAND_BACK))})
  return out


class TestSwapWhileSteering:
  def test_a_hand_back_while_steering_leaves_the_torque_as_it_was(self, params):
    steady = drive(controls(), hand_back=False)
    handed_back = drive(controls(), hand_back=True)
    assert handed_back == pytest.approx(steady, abs=1e-9)

  def test_a_short_drop_keeps_the_tune_and_the_torque(self, params):
    # a steer fault flicker, and a 0.3 s gap: neither swaps, so the resume is the same
    # controller as without the hand-back
    gaps = {HAND_BACK + 20} | set(range(HAND_BACK + 40, HAND_BACK + 70))
    steady = drive(controls(), hand_back=False, inactive=gaps)
    ctl = controls()
    handed_back = drive(ctl, hand_back=True, inactive=gaps)
    assert handed_back == pytest.approx(steady, abs=1e-9)
    assert ctl.LaC is ctl._lac_by_size[True]

  def test_half_a_second_off_swaps(self, params):
    off = HAND_BACK + 40
    ctl = controls()
    drive(ctl, hand_back=True, inactive_at=off, frames=off + TUNE_SWAP_INACTIVE_FRAMES - 1)
    assert ctl.LaC is ctl._lac_by_size[True], "swapped before lateral had been off for half a second"
    ctl = controls()
    drive(ctl, hand_back=True, inactive_at=off, frames=off + TUNE_SWAP_INACTIVE_FRAMES)
    assert ctl.LaC is ctl._lac_by_size[False], "not swapped after half a second off"

  def test_the_counter_advances_once_per_inactive_tick_and_resets_on_active(self, params):
    # production wiring: update_lateral_assist runs once per control tick, so an inactive
    # frame moves the count by exactly one and an active frame zeroes it
    ctl = controls()
    counts = []
    gaps = {HAND_BACK + 10, HAND_BACK + 11, HAND_BACK + 20}
    drive(ctl, hand_back=False, frames=HAND_BACK + 30, inactive=gaps, counts=counts)
    assert counts[HAND_BACK + 10] == 1
    assert counts[HAND_BACK + 11] == 2
    assert counts[HAND_BACK + 12] == 0
    assert counts[HAND_BACK + 20] == 1
    assert counts[HAND_BACK + 21] == 0

  def test_the_counter_advances_in_maneuver_mode_and_the_swap_still_waits(self, params):
    # the maneuver mode's scripted commands bypass the assist layer, not the count
    off = HAND_BACK + 40
    counts = []
    ctl = controls()
    maneuvers = set(range(off, off + TUNE_SWAP_INACTIVE_FRAMES))
    drive(ctl, hand_back=True, inactive_at=off, frames=off + TUNE_SWAP_INACTIVE_FRAMES - 1,
          maneuvers=maneuvers, counts=counts)
    assert counts[off:] == list(range(1, TUNE_SWAP_INACTIVE_FRAMES))
    assert ctl.LaC is ctl._lac_by_size[True], "swapped before the half-second wait in maneuver mode"
    ctl = controls()
    drive(ctl, hand_back=True, inactive_at=off, frames=off + TUNE_SWAP_INACTIVE_FRAMES, maneuvers=maneuvers)
    assert ctl.LaC is ctl._lac_by_size[False], "not swapped after half a second off in maneuver mode"

  def test_the_first_re_engaged_frame_uses_a_primed_controller(self, params):
    # the swap reset the incoming controller and the remaining inactive frames primed its
    # setpoint machinery with the live command (the integrator carries by design), so
    # re-engagement must behave like the same controller with its exact history replayed,
    # not like the idle one that still held the 20 m/s curve's setpoint state
    off = HAND_BACK + 40
    priming = 60
    resume = off + TUNE_SWAP_INACTIVE_FRAMES + priming
    ctl = controls()
    out = drive(ctl, hand_back=True, inactive=set(range(off, resume)), frames=resume + 20)
    assert ctl.LaC is ctl._lac_by_size[False], "no swap: the re-engagement ran the old tune"
    # the entry the first engaged frame's setpoint lead reads (lat_delay / dt = 30 back)
    assert ctl.LaC.curvature_request_buffer[-30] == pytest.approx(CURVATURE)
    assert ctl.LaC.previous_measurement == pytest.approx(LAT_ACCEL)

    ref = make_lac(LatControlTorqueV2, friction=FRICTION)
    for _ in range(300):
      step(ref, make_cs(v_ego=20.0, lat_accel=-1.5), -1.5 / 20.0 ** 2)
    ref.reset()  # what select_lateral_control ran on it at the swap
    for _ in range(priming):
      step(ref, make_cs(V_EGO, LAT_ACCEL), CURVATURE, active=False)
    for i in range(20):
      ref_torque, _, _ = ref.update(True, make_cs(V_EGO, LAT_ACCEL), *DRIVE_ARGS)
      assert out[resume + i] == pytest.approx(ref_torque, abs=1e-9)
