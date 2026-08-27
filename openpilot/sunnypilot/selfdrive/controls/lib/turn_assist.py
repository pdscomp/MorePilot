"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Low-speed turn assist: keeps signaled intersection turns wound instead of going wide.

Approaching a turn with the blinker on, the model's time-based plan collapses as the
car slows: desiredCurvature decays to zero, the controller actively unwinds the wheel
at the intersection, and on pull-away it re-winds too late — the car goes wide. Two
mechanisms fix the two halves:

  - a curvature HOLD that ratchets up on the blinker-matching model command below the
    release speed and floors the command magnitude afterwards, with a plan-sourced
    pre-wind near standstill and driver nudge-to-commit capture;
  - a turn-initiation LEAD that probes the plan at a constant-TIME distance while
    still rolling (9-12 mph), where the model's meter-anchored horizon gives too
    little warning to wind the wheel.

Ported from StarPilot (github.com/firestar5683/StarPilot, controlsd.py); the tuning
constants and the failure evidence cited beside them are theirs, from rlog-driven
iteration on their fleet.
"""
import math

from opendbc.car import structs
from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL

# The hold ratchets up on the blinker-matching model command below the release speed
# and floors the command magnitude afterwards. Below the hard speed the floor is firm;
# between hard and release speed it decays toward the model's sustained demand, so a
# transient model dip barely sags it while a genuine end-of-turn unwind or an aborted
# turn still drains it in a few seconds. Retention deliberately does NOT depend on the
# blinker (the stalk auto-cancels during the stop), on latActive (lateral goes inactive
# at standstill on torque cars; the wheel parks on rack friction), or on
# steeringPressed (the driver's instinctive grip during the unwind is what let the
# collapse through, and the driver physically overpowers a torque command regardless).
HOLD_HARD_SPEED = 4.5 * CV.MPH_TO_MS
# Release must sit ABOVE the speed where the model's action wakes up mid-turn: left
# turns cross the intersection before arcing, reaching ~3.5-4 m/s with the action still
# ~0 (a 6 mph release dropped the floor mid-turn and visibly unwound the wheel 50 deg;
# rights wake at ~2.2 m/s). Hold authority above creep speed is bounded by the
# opposite-command release and the decay band, not by this ceiling.
HOLD_RELEASE_SPEED = 10.0 * CV.MPH_TO_MS
# Pre-wind is a NEAR-STANDSTILL device: winding the wheel is only free when the car
# isn't moving. On rolling slow turns a plan-sourced floor applies the turn's final
# curvature at the entry, starting the arc 4-7 m early. Above this speed only the
# model's own action can raise the hold, rate-limited so a single-frame action spike
# can't get captured and floored for seconds.
HOLD_PLAN_SOURCE_SPEED = 2.0 * CV.MPH_TO_MS
HOLD_RATCHET_RATE = 0.04  # 1/m per s, hold growth limit above the plan-source speed
# Once the model's action has sustainably taken over the turn, hand off COMPLETELY:
# clear the hold and don't re-engage until the blinker cycle ends. A floor that chases
# the awake action only distorts the model's entry spiral, mid-turn shape, and exit
# unwind — the bridge job is done the moment the action is awake.
HOLD_HANDOFF_FRAC = 0.75
HOLD_HANDOFF_TIME = 0.3  # s of sustained action >= frac*hold before handoff
HOLD_DECAY_TAU = 2.0     # s; hold tracks a sustained lower model demand with this time constant
# At a turn exit the model unwinds through small SAME-sign commands (curvature only
# flips negative for the final counter-steer), so the opposite-release fires late and
# the tau-2 decay melts the floor slower than the model's exit ramp. Turn progress
# discriminates a mid-turn dip (protect the floor; sags happen ~20 deg of swept
# heading) from an exit (drop it; sticks happen past ~80 deg): past the swept
# threshold the decay switches to the fast tau and runs at ANY speed, including below
# the hard-hold speed. Swept resets whenever the hold disengages.
HOLD_SWEPT_EXIT = 0.9      # rad of heading actually turned (~52 deg)
HOLD_EXIT_DECAY_TAU = 0.5  # s
HOLD_STANDSTILL_TIMEOUT = 30.0  # s stopped before the held turn intent is dropped

# The model's time-domain action.desiredCurvature is blind below ~2.5 m/s (0.3 s ahead
# at creep speed is centimeters of road), but the plan's spatial geometry already shows
# the turn at standstill (plan curvature 0.13-0.16 with the action at 0.005, matching
# the demand the action produced once rolling). Feeding it into the ratchet lets the
# wheel pre-wind toward the real turn before the car moves. Scaled and capped
# conservatively: a too-high floor turns in tighter than the path (mild at creep lat
# accel), while a too-low one just reduces the head start. The plan flickers straight
# for ~1-2 s right at the standstill->motion transition; the ratchet holds through it
# by design.
HOLD_PLAN_LOOKAHEAD_NEAR = 4.0  # m; reads whether the turn starts NOW
HOLD_PLAN_LOOKAHEAD_FAR = 7.0   # m; reads the turn's curvature
HOLD_PLAN_SCALE = 0.85
HOLD_PLAN_CAP = 0.12            # 1/m
# Proximity gate on the pre-wind: the 4/7 m probe reads the corner's full curvature the
# instant the plan bends toward it, which at a stop-line turn is several meters before
# the car reaches the line — winding the wheel while still rolling cuts the inside
# curb, and an early blinker turns in before the corner. The plan itself carries the
# distance: the onset probe finds where the path bends. Scale the pre-wind from full at
# ONSET_NEAR to zero at ONSET_FAR_GATE so it winds only as the corner closes. At a real
# stop the onset collapses to ~0 m once stopped, so the stop-line pre-wind still
# reaches full strength — just later, at the line instead of 3 m short of it.
HOLD_ONSET_HEADING = 10.0   # deg of plan heading change that marks the corner
HOLD_ONSET_NEAR = 1.5       # m; corner this close -> full pre-wind
HOLD_ONSET_FAR_GATE = 5.0   # m; corner past this -> no pre-wind yet
HOLD_ONSET_FAR = 100.0      # m; sentinel "no corner found"
HOLD_REACH_MIN = 7.0        # m of plan reach below which the pre-wind is untrusted
HOLD_REACH_FULL = 12.0
# The model counter-steers at every turn exit; an opposite-direction command is the
# "turn is over" signal at any speed. Without this the floor converts the exit unwind
# into a stuck same-sign command the driver has to fight. Deadband rejects the ~0.002
# pull-away flickers.
HOLD_OPPOSITE_RELEASE = 0.01  # 1/m
# Nudge-to-commit: creeping toward a rolling turn below the release speed, the model
# often does not commit until the geometry is entered, leaving the driver to wind the
# whole turn by hand. The driver's own torque separates a premature machine wind (they
# BRACE against it) from a gap that is theirs (they PUSH into it), so a matching push
# is the "go" signal: capture the curvature they have physically wound into the hold,
# un-rate-limited (the wheel is already there; holding adds no motion), so the car
# grabs the turn at a nudge. Allowed anywhere the hold exists (< release speed); the
# decay/handoff/opposite-release machinery already bounds the captured floor.
HOLD_CONFIRM_MIN = 0.003   # 1/m (~7 deg of wheel) of wound curvature before capture
HOLD_CONFIRM_SWEPT = 0.6   # rad swept this blinker cycle; past this a push is exit-shaping, not initiation

# Turn-initiation lead. The model's action and the fixed 4/7 m probes are anchored in
# METERS, so the seconds of warning they give shrinks with speed — at 12 mph a corner
# enters the 7 m window only ~1.3 s out, too late to wind the wheel. This lead probes
# the plan at a constant-TIME distance instead and max-mag blends into the command, so
# initiation can start while still rolling at 9-12 mph. The engagement fade (authority
# ramps to zero as measured curvature approaches the lead) limits it to the initiation
# phase: once the car is tracking the arc, the model owns the turn shape. A binary gate
# here limit-cycles (cutting drops demand to a still-small action, the wheel unwinds
# below the threshold, the lead re-fires — a 5 Hz demand sawtooth felt as wiggle); the
# fade instead settles at a stable ~2/3-of-lead equilibrium until the action takes over
# via the max-mag blend. Below TURN_LEAD_MIN_SPEED the lead must stay OFF: at creep
# speed the probe's 4 m distance floor chord-fits a turn's straight-then-arc entry as
# "arc now", demanding 3-5x the model's intent, and the model fights back — while its
# meter-anchored horizon already gives 2+ s of warning there, so the lead's reason to
# exist does not apply.
TURN_LEAD_T = 1.3           # s of travel the probe looks ahead (~wind-up time + lat delay)
TURN_LEAD_MIN_M = 4.0
TURN_LEAD_MAX_M = 14.0
TURN_LEAD_MIN_SPEED = 3.0   # m/s: authority 0 here, ramps to full at FULL_SPEED
TURN_LEAD_FULL_SPEED = 4.0  # m/s
TURN_LEAD_MAX_SPEED = 7.0   # m/s (~15.7 mph)
TURN_LEAD_SCALE = 0.85
TURN_LEAD_CAP = 0.12        # 1/m
TURN_LEAD_ENGAGED_FRAC = 0.5    # engagement fade starts here, zero authority at 1.0
TURN_LEAD_MODEL_OPPOSE = 0.003  # 1/m: model steering this hard against the blinker vetoes the lead
# Braking-to-a-stop veto: if sustaining the current decel parks the car within this
# factor of the probe distance, the driver intends to stop short of the arc — demanding
# that arc's curvature NOW winds the wheel at a stop approach the model didn't plan.
# Turn-approach braking releases before this trips; a held brake to standstill keeps it
# tripped, deferring the turn to the pre-wind.
TURN_LEAD_STOP_MARGIN = 1.5
TURN_LEAD_DECEL_GATE = -0.5  # m/s^2: only project a stop when genuinely braking


def _circle_curvature(x: float, y: float) -> float:
  # curvature of the circle through the origin, tangent to the car's heading, passing
  # through the plan point (x, y): kappa = 2y / (x^2 + y^2)
  d2 = x * x + y * y
  if d2 < 1.0:
    return 0.0
  return 2.0 * y / d2


def plan_dual_probe(model_v2, d_near: float, d_far: float) -> float:
  # Min-magnitude of a near and a far circle fit. The far probe alone assumes the turn
  # starts immediately, which over-winds wide turns whose arc begins several meters out
  # (wide multi-lane lefts): the near probe reads ~straight there and only grows as the
  # car approaches the arc, so the readout self-scales to the turn geometry. Sign
  # disagreement means no coherent turn ahead: contribute nothing.
  # One pass over the plan (this runs at 100 Hz): each probe takes the first point at
  # or past its lookahead, falling back to the last point.
  nx, ny = fx, fy = 0.0, 0.0
  near_found = False
  d_near_sq, d_far_sq = d_near * d_near, d_far * d_far
  for x, y in zip(model_v2.position.x, model_v2.position.y, strict=True):
    d2 = x * x + y * y
    if not near_found:
      nx, ny = x, y
      near_found = d2 >= d_near_sq
    fx, fy = x, y
    if d2 >= d_far_sq:
      break
  near = _circle_curvature(nx, ny)
  far = _circle_curvature(fx, fy)
  if near * far <= 0.0:
    return 0.0
  return near if abs(near) < abs(far) else far


def get_plan_turn_onset_dist(model_v2) -> float:
  # Distance along the plan at which the path first bends past ONSET_HEADING from the
  # car's current heading — i.e. how far ahead the corner actually starts. Returns a
  # large sentinel when no bend is found, so distant/straight plans read "far" and
  # don't wind.
  xs, ys = model_v2.position.x, model_v2.position.y
  n = min(len(xs), len(ys))
  for i in range(2, n):
    dx = xs[i] - xs[i - 1]
    dy = ys[i] - ys[i - 1]
    if abs(dx) < 1e-3 and abs(dy) < 1e-3:
      continue
    if abs(math.degrees(math.atan2(dy, dx))) > HOLD_ONSET_HEADING:
      return math.hypot(xs[i], ys[i])
  return HOLD_ONSET_FAR


def get_plan_reach(model_v2) -> float:
  xs = model_v2.position.x
  return xs[-1] if len(xs) else 0.0


class TurnAssistController:
  """Stateful low-speed turn hold + turn-initiation lead over the desired curvature.

  Curvature sign convention matches controlsd: positive is a RIGHT turn, so the
  blinker maps right=+1, left=-1, and a LEFT driver push is positive steeringTorque.
  """

  def __init__(self, CP: structs.CarParams):
    self.params = Params()
    self.enabled = False
    # Torque steering mechanically damps the lead/catch-up fade cycle; a direct
    # angle/curvature controller follows it literally, which can reverse the wheel
    # command several times during one turn initiation.
    self.lead_allowed = CP.steerControlType not in (structs.CarParams.SteerControlType.angle,
                                                    structs.CarParams.SteerControlType.curvature)
    self.hold = 0.0
    self.standstill_t = 0.0
    self.swept = 0.0
    self.handoff_t = 0.0
    self.done = False
    self.blinker_swept = 0.0
    self.lead_applied = 0.0
    self.get_params()

  def get_params(self) -> None:
    # BlinkerPauseLateralControl suppresses lateral entirely on the blinker below its
    # speed — exactly the regime this feature steers in. The pause wins: it is the
    # driver saying "don't steer on my blinker".
    self.enabled = self.params.get_bool("LowSpeedTurnAssist") and \
                   not self.params.get_bool("BlinkerPauseLateralControl")

  def reset(self) -> None:
    self.hold = 0.0
    self.standstill_t = 0.0
    self.swept = 0.0
    self.handoff_t = 0.0
    self.done = False
    self.lead_applied = 0.0

  def update(self, CS: structs.CarState, lat_active: bool, model_v2, new_desired_curvature: float,
             current_curvature: float) -> float:
    """Returns the (possibly floored/led) desired curvature. new_desired_curvature is
    the raw model command controlsd would otherwise clip; current_curvature is the
    measured curvature from the steering angle."""
    self.lead_applied = 0.0
    if not self.enabled:
      self.reset()
      self.blinker_swept = 0.0
      return new_desired_curvature

    v_ego = CS.vEgo
    blinker_dir = float(CS.rightBlinker) - float(CS.leftBlinker)
    # heading swept in the blinker's direction over the whole blinker cycle (any
    # speed): discriminates a turn not yet made from one being exited
    if blinker_dir == 0.0:
      self.blinker_swept = 0.0
    else:
      self.blinker_swept += max(v_ego * current_curvature * blinker_dir, 0.0) * DT_CTRL

    if v_ego >= HOLD_RELEASE_SPEED:
      # hold state clears, but the turn lead below still runs: its speed range
      # (3-7 m/s) deliberately straddles the hold's release speed
      self.reset()
      return self._apply_turn_lead(CS, lat_active, model_v2, new_desired_curvature, current_curvature, blinker_dir)

    if self.hold == 0.0:
      self.swept = 0.0
    else:
      # heading actually swept in the hold's direction: the measure of turn progress
      self.swept += max(v_ego * current_curvature * math.copysign(1.0, self.hold), 0.0) * DT_CTRL
    turn_exiting = self.swept > HOLD_SWEPT_EXIT

    if (v_ego > HOLD_HARD_SPEED or turn_exiting) and lat_active and self.hold != 0.0:
      # Decay toward the model's sustained same-direction demand instead of leaking on
      # wall-clock time: a wall-clock leak drains the floor while the model dips
      # transiently mid-turn, while sustained low demand (end of turn, abort) still
      # drains the hold within a couple of time constants.
      hold_dir = math.copysign(1.0, self.hold)
      model_mag = max(new_desired_curvature * hold_dir, 0.0)
      if model_mag < abs(self.hold):
        decay_tau = HOLD_EXIT_DECAY_TAU if turn_exiting else HOLD_DECAY_TAU
        decayed = abs(self.hold) + (model_mag - abs(self.hold)) * (DT_CTRL / decay_tau)
        self.hold = math.copysign(decayed, self.hold)

    if v_ego < 0.5:
      self.standstill_t += DT_CTRL
      if self.standstill_t > HOLD_STANDSTILL_TIMEOUT:
        self.hold = 0.0
      # a stop resets the turn cycle: the model goes blind again, so a prior handoff
      # must not block the standstill pre-wind
      self.done = False
    else:
      self.standstill_t = 0.0

    if lat_active and self.hold != 0.0 and \
       new_desired_curvature * math.copysign(1.0, self.hold) < -HOLD_OPPOSITE_RELEASE:
      # model is actively counter-steering: the turn is over, release at any speed
      self.hold = 0.0
      self.done = True

    if lat_active and self.hold != 0.0 and \
       new_desired_curvature * math.copysign(1.0, self.hold) >= HOLD_HANDOFF_FRAC * abs(self.hold):
      self.handoff_t += DT_CTRL
      if self.handoff_t > HOLD_HANDOFF_TIME:
        # action has sustainably taken over: hand off completely
        self.hold = 0.0
        self.done = True
    else:
      self.handoff_t = 0.0

    # Driver push into the signaled turn with the wheel already wound: positive
    # steeringTorque is a LEFT push (negative curvature), so agreement is a negative
    # product with blinker_dir. Feeds both the re-arm and the nudge capture below.
    driver_push = lat_active and CS.steeringPressed and CS.steeringTorque * blinker_dir < 0.0 and \
        current_curvature * blinker_dir > HOLD_CONFIRM_MIN

    if blinker_dir == 0.0:
      # blinker cycle over: a fresh turn may engage a fresh hold
      self.done = False
    elif driver_push and self.blinker_swept < HOLD_CONFIRM_SWEPT:
      # an active driver push into the signaled turn BEFORE the turn is made is fresh
      # turn intent: re-arm the cycle even after a prior handoff. A long blinker-on
      # approach can latch done on a trivial micro-handoff and lock out nudge-to-commit
      # at the real turn. The swept gate keeps a light same-direction touch during the
      # EXIT unwind from re-latching a large hold against the model's recentering.
      self.done = False

    if blinker_dir != 0.0 and not self.done:
      # Ratchet up on the raw model command, never on the floored/measured value, so
      # the hold can't feed itself and defeat the decay. Below the release speed the
      # plan's spatial curvature is the second, earlier-seeing source: it shows the
      # turn at standstill while the action is still blind, letting the pre-wind start
      # before the car moves.
      turn_candidate = new_desired_curvature if lat_active else 0.0
      if lat_active and v_ego < HOLD_PLAN_SOURCE_SPEED:
        # Cheapest gate first — a zero weight nulls the probe, and a nulled probe never
        # raises the candidate, so the plan scans can be skipped outright.
        reach = get_plan_reach(model_v2)
        reach_w = min(max((reach - HOLD_REACH_MIN) / (HOLD_REACH_FULL - HOLD_REACH_MIN), 0.0), 1.0)
        # Proximity gate: wind only as the corner closes, so a stop-line turn winds at
        # the line and an early blinker doesn't turn in early.
        onset_w = 0.0
        if reach_w > 0.0:
          onset = get_plan_turn_onset_dist(model_v2)
          onset_w = min(max((HOLD_ONSET_FAR_GATE - onset) / (HOLD_ONSET_FAR_GATE - HOLD_ONSET_NEAR), 0.0), 1.0)
        if reach_w > 0.0 and onset_w > 0.0:
          plan_curvature = plan_dual_probe(model_v2, HOLD_PLAN_LOOKAHEAD_NEAR, HOLD_PLAN_LOOKAHEAD_FAR) * HOLD_PLAN_SCALE
          plan_curvature = max(min(plan_curvature, HOLD_PLAN_CAP), -HOLD_PLAN_CAP)
          plan_curvature *= onset_w * reach_w
          if plan_curvature * blinker_dir > turn_candidate * blinker_dir:
            turn_candidate = plan_curvature
      # Nudge-to-commit: the driver actively pushing in the blinker direction at creep
      # speed captures what they have wound. Exempt from the ratchet rate limit:
      # latching the wheel's current position commands no motion, only keeps the
      # driver's progress.
      driver_confirmed = False
      if driver_push:
        wound_curvature = max(min(current_curvature, HOLD_PLAN_CAP), -HOLD_PLAN_CAP)
        if wound_curvature * blinker_dir > turn_candidate * blinker_dir:
          turn_candidate = wound_curvature
          driver_confirmed = True
      if turn_candidate * blinker_dir > abs(self.hold):
        new_mag = turn_candidate * blinker_dir
        if v_ego > HOLD_PLAN_SOURCE_SPEED and not driver_confirmed:
          new_mag = min(new_mag, abs(self.hold) + HOLD_RATCHET_RATE * DT_CTRL)
        self.hold = math.copysign(new_mag, turn_candidate)
      elif self.hold * blinker_dir < 0.0:
        # blinker flipped to the other side: turn intent changed
        self.hold = 0.0

    if lat_active and self.hold != 0.0:
      hold_dir = math.copysign(1.0, self.hold)
      if new_desired_curvature * hold_dir < abs(self.hold):
        new_desired_curvature = self.hold

    return self._apply_turn_lead(CS, lat_active, model_v2, new_desired_curvature, current_curvature, blinker_dir)

  def _apply_turn_lead(self, CS: structs.CarState, lat_active: bool, model_v2, new_desired_curvature: float,
                       current_curvature: float, blinker_dir: float) -> float:
    # Applied AFTER the hold block so the ratchet/handoff only ever see the raw model
    # action; pure max-magnitude, so it can never reduce or oppose the model. Lane
    # changes are excluded: that blinker's plan bend is not a turn. The model-oppose
    # veto is defense-in-depth for the fade-in edge: a model actively steering against
    # the blinker is correcting something the lead must not fight.
    if not (self.lead_allowed and lat_active and blinker_dir != 0.0 and
            TURN_LEAD_MIN_SPEED <= CS.vEgo < TURN_LEAD_MAX_SPEED and
            new_desired_curvature * blinker_dir > -TURN_LEAD_MODEL_OPPOSE and
            model_v2.meta.laneChangeState == log.LaneChangeState.off):
      return new_desired_curvature
    d_near = max(min(TURN_LEAD_T * CS.vEgo, TURN_LEAD_MAX_M), TURN_LEAD_MIN_M)
    stopping_short = CS.aEgo < TURN_LEAD_DECEL_GATE and \
        CS.vEgo ** 2 / (2.0 * -CS.aEgo) < TURN_LEAD_STOP_MARGIN * d_near
    lead_curvature = 0.0 if stopping_short else plan_dual_probe(model_v2, d_near, d_near + 3.0) * TURN_LEAD_SCALE
    lead_curvature = max(min(lead_curvature, TURN_LEAD_CAP), -TURN_LEAD_CAP)
    if lead_curvature * blinker_dir > 0.0:
      speed_w = min(max((CS.vEgo - TURN_LEAD_MIN_SPEED) / (TURN_LEAD_FULL_SPEED - TURN_LEAD_MIN_SPEED), 0.0), 1.0)
      engaged_ratio = abs(current_curvature) / abs(lead_curvature)
      engage_w = min(max((1.0 - engaged_ratio) / (1.0 - TURN_LEAD_ENGAGED_FRAC), 0.0), 1.0)
      lead_curvature *= speed_w * engage_w
      if lead_curvature * blinker_dir > max(new_desired_curvature * blinker_dir, 0.0):
        new_desired_curvature = lead_curvature
        self.lead_applied = lead_curvature
        # Capture the applied lead into the hold (rate-limited like any moving-speed
        # ratchet) so decelerating through the fade floor keeps the initiation
        # progress: without this, braking mid-wind dumps the lead's demand back to the
        # still-small action and visibly unwinds the wheel before the standstill
        # pre-wind has to redo the work.
        if CS.vEgo < HOLD_RELEASE_SPEED and not self.done and \
           lead_curvature * blinker_dir > abs(self.hold):
          held_mag = min(lead_curvature * blinker_dir, abs(self.hold) + HOLD_RATCHET_RATE * DT_CTRL)
          self.hold = math.copysign(held_mag, lead_curvature)
    return new_desired_curvature
