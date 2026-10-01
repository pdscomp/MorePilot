"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from opendbc.sunnypilot.car.icbm_actuation_profile import get_actuation_profile
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import minimum_set_speed_ms
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.icbm_latch import icbm_active
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import V_CRUISE_UNSET

ButtonType = car.CarState.ButtonEvent.Type
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

INACTIVE_TIMER = 0.4
# After a genuine driver press the servo yields in the opposing direction for this long:
# the driver just chose a speed and an immediate synthesized walk-back reads as a fight
# (route 126 t=341: a +5 was reverted within 1.4 s). SET+ parks down-moves, SET- parks
# up-moves (a refused re-anchor otherwise restores the baseline right over a fresh -5);
# a press in the opposing direction cancels the other grace. Accountability mirrors the
# gas override: the press is the driver's, so no urgency carve-out.
DRIVER_PRESS_GRACE_T = 3.0
DRIVER_PRESS_GRACE_FRAMES = int(DRIVER_PRESS_GRACE_T / DT_CTRL)
# Reaction deadband in display units, applied only while a limiter (SCC/SLA) drives the
# plan; those targets jitter 1-2 units frame to frame and an undamped servo ping-pongs
# SET+/SET- around the noise. A cruise-source target is the driver setpoint, a stable
# integer, so track it exactly: a dash residual from a dropped press self-heals instead
# of stranding the dash low.
REACT_DEADBAND = 2
# The error must persist this long before acting, so a single-frame target glitch
# (e.g. a bad map sample) or a momentary dip can't trigger a button burst.
REACT_TIMER = 0.3
# Down moves act after REACT_TIMER. Up moves on decel_needs_stable_setpoint cars wait for
# the target to hold still this long first: limiter dips arrive in trains, and restoring
# between them churns the dash and delays the next decel on an ECU that will not commit
# while the set speed is moving. The quiet window also gives card time to adopt the dash
# after a driver press before the servo could chase a stale target. Decel-overshoot
# release is exempt; its slow monotonic rise is measured as tolerated.
# Sized from an 11-route / 57k-frame sweep of the recorded target streams: the churn
# suppression is all bought in the first second (regret 67.7% -> 27.0% at 1.0 s; 3.0 s
# only reaches 26.2% while nearly doubling the speed lost to the wait).
RESTORE_QUIET_TIME = 1.0
RESTORE_QUIET_FRAMES = int(RESTORE_QUIET_TIME / DT_CTRL)

# Stock ACC deceleration follows the gap between dash set speed and actual speed. Hold the dash
# the gap that yields the limiter's own requested decel below vEgo (the actuation profile's
# decel_overshoot table) instead of walking it to the whole dip.
# Apply quickly and release slowly across the ECU's discrete deceleration stages.
DECEL_OVERSHOOT_RISE = 10.  # mph/s
DECEL_OVERSHOOT_RELEASE = 3.  # mph/s
# Under openpilot longitudinal the planner executes these targets directly; the servo only
# keeps the dash on the speed limit session (Mazda alpha long).
OP_LONG_PLANNER_SOURCES = (LongitudinalPlanSource.sccVision, LongitudinalPlanSource.sccMap)
DECEL_OVERSHOOT_SOURCES = (*OP_LONG_PLANNER_SOURCES, LongitudinalPlanSource.speedLimitAssist)

# Curve/limit sources the planner already brakes for under openpilot longitudinal: the
# servo leaves the dash alone for those (see update_calculations).
OP_LONG_PLANNER_SOURCES = (LongitudinalPlanSource.sccVision, LongitudinalPlanSource.sccMap)

# The sustained 10 Hz stream registers on the ECU as paced discrete presses, never as a
# held button (all-routes census: 149/149 stream-driven dash steps were 1 mph -- the
# wheel's own button-up frames interleave with the forged ones, so the ECU never sees an
# unbroken hold). At ~4 mph/s it is still the fastest walk available, so it takes any
# move with a real distance to cover; discrete taps take the small remainder, where the
# stream's in-flight frames would overshoot and ping-pong around the target.
FAST_MODE_MIN = 3  # display units of remaining error to run the stream
# If the dash never moves under the stream, this ECU is not registering it at all;
# taps are the proven fallback for the rest of the drive.
FAST_STALL_T = 1.5  # s

# sent as taps, or as the hold stream while fast mode runs (fast_active, SelfdriveStateZP.icbmHold)
SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.profile = get_actuation_profile(CP.brand)
    self.active = not CP_SP.pcmCruiseSpeed  # card's decision, kept current by update_activation
    self.overshoot_params = self.profile.decel_overshoot  # brands with a measured plant
    self.fast_faulted = False  # disable the stream for this drive if the dash never moves
    self.reset()

  def reset(self) -> None:
    """Servo state for one engagement's worth of moves; what the drive learned about the dash
    (fast_faulted) is kept."""
    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0
    self.restore_quiet_timer = 0
    self.v_target_raw = 0
    self.v_target_raw_prev = 0
    self.react_deadband = REACT_DEADBAND
    self.lookahead_valid = False
    self.dip_ahead = False
    self.down_grace_timer = 0
    self.up_grace_timer = 0
    self.press_target = 0  # plan target at the driver's SET+: the grace parks down moves to it, not to a lower one

    self.is_ready = False
    self.is_ready_prev = False
    self.is_metric = False
    # A pending SLA confirmation: card vetoes up moves while it is open (cruise_arbiter), so a
    # dash that does not move then is not a stalled stream.
    self.prompting = False
    self.decel_overshoot_enabled = False
    self.overshoot_mph = 0.0
    self.limiter_active = False

    # Fast-walk stream execution
    self.fast_active = False
    self.fast_stall_frames = 0
    self.fast_last_cluster = 0

    self.cruise_button_timers = dict(CRUISE_BUTTON_TIMER)

  def update_decel_overshoot(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> float:
    if self.overshoot_params is None:
      return 0.0

    p = self.overshoot_params
    want = 0.0
    # Never integrate a command the servo cannot emit: is_ready covers a driver press,
    # prompting a pending confirm. Both block emission, so winding up behind them
    # only banks a stale gap to dump the moment the block lifts. Nothing is lost by
    # waiting -- a limiter still asking for decel rebuilds at DECEL_OVERSHOOT_RISE, ~0.5 s
    # to a full gap, well inside the REACT_TIMER the servo owes before it acts anyway.
    if (self.decel_overshoot_enabled and self.is_ready and not self.prompting
        and not self.down_parked
        # under openpilot longitudinal the planner brakes for curves itself; the lever
        # would only bank a gap the dash walk dumps mid-curve
        and not self.CP.openpilotLongitudinalControl
        and LP_SP.longitudinalPlanSource in DECEL_OVERSHOOT_SOURCES
        and LP_SP.aTarget < -p['min_decel'] and CS.vEgo > LP_SP.vTarget):
      want = min(float(np.interp(-LP_SP.aTarget, p['decel_bp'], p['gap_v'])), p['max_gap'])

    if want > self.overshoot_mph:
      self.overshoot_mph = min(want, self.overshoot_mph + DECEL_OVERSHOOT_RISE * DT_CTRL)
    else:
      # release gently only while the limiter is live (aTarget flaps between the ECU's
      # decel stages mid-maneuver); once the plan is back on cruise the residual only
      # holds the dash down and stalls the restore, so drop it at the build rate
      release = DECEL_OVERSHOOT_RELEASE if self.limiter_active else DECEL_OVERSHOOT_RISE
      self.overshoot_mph = max(want, self.overshoot_mph - release * DT_CTRL)

    return self.overshoot_mph

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

    self.limiter_active = LP_SP.longitudinalPlanSource != LongitudinalPlanSource.cruise

    v_target_ms = LP_SP.vTarget
    if self.CP.openpilotLongitudinalControl and LP_SP.longitudinalPlanSource in OP_LONG_PLANNER_SOURCES:
      # openpilot brakes for curves itself: the dash keeps the driver's setpoint, or the
      # session's cap inside a zone (the mirror publishes it, V_CRUISE_UNSET when idle), and
      # never walks down to a curve target nor back up over the cap during one
      session_cap = LP_SP.speedLimit.assist.vTarget
      v_target_ms = min(CS.vCruise * CV.KPH_TO_MS, session_cap if session_cap > 0. else V_CRUISE_UNSET)
    overshoot_ms = self.update_decel_overshoot(CS, LP_SP) * CV.MPH_TO_MS
    if overshoot_ms > 0:
      # command relative to actual speed so the ECU sees the gap that produces the requested
      # decel; never above the planner target, and never more than the gap below it
      v_target_ms = min(v_target_ms, max(CS.vEgo, LP_SP.vTarget) - overshoot_ms)

    self.v_target = round(v_target_ms * speed_conv)
    # The plan's own target, before the overshoot lever: restore intent is judged against
    # this. The lever's decay is self-inflicted motion and must not look like a moving plan.
    self.v_target_raw_prev = self.v_target_raw
    self.v_target_raw = round(LP_SP.vTarget * speed_conv)
    self.v_cruise_min = round(minimum_set_speed_ms(self.CP_SP, self.is_metric) * speed_conv)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

    # Exact tracking against the (stable) driver setpoint; jitter band against limiters.
    # Overshoot keeps the limiter band: its command moves by design.
    self.react_deadband = REACT_DEADBAND if self.limiter_active or self.overshoot_mph > 0 else 1

    # A zero vision lookahead falls back to the stable-target restore gate.
    v_ahead_min = LP_SP.zoompilot.sccVAheadMin
    self.lookahead_valid = v_ahead_min > 0.
    self.dip_ahead = self.lookahead_valid and v_ahead_min * speed_conv < self.v_target_raw - self.react_deadband
    if self.dip_ahead and self.v_target > self.v_cruise_cluster:
      # A dip ahead caps a restore at its own level rather than freezing a dash that sits below it.
      v_ahead = int(v_ahead_min * speed_conv)
      if v_ahead - self.v_cruise_cluster >= self.react_deadband:
        self.v_target = min(self.v_target, v_ahead)
        self.dip_ahead = False

  def update_restore_quiet_timer(self) -> None:
    # how long an up-error has persisted against a still PLAN target; plan-target motion,
    # the error closing, or a pending confirm prompt resets it. Holding the timer at zero
    # through the prompt means a decline or timeout still waits out a FULL quiet window
    # before any restore: the prompt must not pre-pay the servo's patience.
    # Keyed on v_target_raw, not the overshoot-adjusted command: the lever's slow release
    # after a limiter ends moved v_target every few frames and pinned this timer at zero
    # until the decay finished (route 126: 4.1 s of extra post-curve braking).
    up_error = self.v_target_raw - self.v_cruise_cluster
    if up_error >= self.react_deadband and self.v_target_raw == self.v_target_raw_prev:
      self.restore_quiet_timer += 1
    else:
      self.restore_quiet_timer = 0

  def plan_fast_mode(self) -> None:
    # Run the stream while the remaining error is worth it; taps take the remainder.
    # Evaluated every frame against the live dash. No grid or metric assumptions: the
    # stream is just presses, so it is valid wherever taps are.
    remaining = abs(self.v_target - self.v_cruise_cluster)
    use_fast = not self.fast_faulted and remaining >= FAST_MODE_MIN

    if use_fast and not self.fast_active:
      self.fast_active = True
      self.fast_stall_frames = 0
      self.fast_last_cluster = self.v_cruise_cluster
    elif self.fast_active:
      if remaining < FAST_MODE_MIN:
        self.fast_active = False
      elif self.v_cruise_cluster != self.fast_last_cluster or self.prompting:
        self.fast_last_cluster = self.v_cruise_cluster
        self.fast_stall_frames = 0
      else:
        self.fast_stall_frames += 1
        if self.fast_stall_frames * DT_CTRL > FAST_STALL_T:
          self.fast_faulted = True
          self.fast_active = False
          cloudlog.event("icbm_fast_mode_fallback", brand=self.CP.brand)

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)
    self.update_restore_quiet_timer()


    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # Up-moves need the quiet window on decel_needs_stable_setpoint cars, on EVERY
        # entry path: the preActive route (taken after any driver press resets
        # readiness) used to bypass it straight into increasing, letting the servo
        # chase a stale target before card had settled the press's own effects.
        # The overshoot exemption only covers the overshoot command's own slow release
        # (still limiter-sourced); residual overshoot after a source flip back to cruise
        # must not bypass the quiet window into a full baseline restore.
        # With a valid vision lookahead the profile replaces the stillness heuristic
        # outright: restore immediately when nothing ahead binds below the target, and
        # hold while a dip is coming, however quiet the target is -- restoring between
        # bends accelerates the car into the next apex (route 126: 3 of 8 over-ceiling
        # apexes were restore-fed).
        if self.lookahead_valid:
          up_allowed = not self.dip_ahead
        else:
          up_allowed = ((self.overshoot_mph > 0 and self.limiter_active)
                        or not self.profile.decel_needs_stable_setpoint
                        or self.restore_quiet_timer >= RESTORE_QUIET_FRAMES)
        up_allowed = up_allowed and self.up_grace_timer <= 0

        # Live limiters may decrease immediately. Residual overshoot after returning to cruise
        # may only release, while ordinary setpoint corrections remain unconditional.
        down_allowed = (self.limiter_active or self.overshoot_mph <= 0) and not self.down_parked

        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_target - self.v_cruise_cluster >= self.react_deadband and up_allowed:
              self.state = State.increasing

            elif self.v_cruise_cluster - self.v_target >= self.react_deadband \
                 and self.v_cruise_cluster > self.v_cruise_min and down_allowed:
              self.state = State.decreasing

            else:
              self.state = State.holding

        elif self.state == State.holding:
          down_pending = self.v_cruise_cluster - self.v_target >= self.react_deadband and down_allowed
          up_pending = self.v_target - self.v_cruise_cluster >= self.react_deadband
          if down_pending or (up_pending and up_allowed):
            self.pre_active_timer = int(REACT_TIMER / DT_CTRL)
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          # a dip appearing mid-restore aborts it: the commit gate trails the profile,
          # and stepping up until the limiter takes the source feeds the next apex
          if self.v_target <= self.v_cruise_cluster or self.dip_ahead:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    if self.state in SEND_BUTTONS:
      self.plan_fast_mode()
      send_button = SEND_BUTTONS[self.state]
    else:
      self.fast_active = False
      send_button = SendButtonState.none

    return send_button

  @property
  def down_parked(self) -> bool:
    """A driver SET+ parks down moves to the target it overrode for the grace window; a lower
    target that appears afterwards (a curve) is new information and goes through."""
    return self.down_grace_timer > 0 and self.v_target_raw >= self.press_target - self.react_deadband

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    # buttonEvents carry only the wheel's own presses (forged frames echo on src 128+ and
    # never reach carState), so this cannot latch on the servo's own sends
    if self.cruise_button_timers[ButtonType.accelCruise] > 0:
      self.down_grace_timer = DRIVER_PRESS_GRACE_FRAMES
      self.press_target = self.v_target_raw
      self.up_grace_timer = 0
    elif self.cruise_button_timers[ButtonType.decelCruise] > 0:
      self.up_grace_timer = DRIVER_PRESS_GRACE_FRAMES
      self.down_grace_timer = 0
    else:
      self.down_grace_timer = max(0, self.down_grace_timer - 1)
      self.up_grace_timer = max(0, self.up_grace_timer - 1)

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool,
          decel_overshoot_enabled: bool = False, session_state=None) -> None:
    if not self.active:
      return


    self.is_metric = is_metric
    self.decel_overshoot_enabled = decel_overshoot_enabled
    # None means read the planner's mirror of the SLA session (selfdrived passes the live one)
    self.prompting = (session_state if session_state is not None else LP_SP.speedLimit.assist.state) == SessionState.preActive

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
