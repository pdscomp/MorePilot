# ICBM restore quiet window: measurement and recommendation

Status: measured 2026-08-29, not yet implemented. Concerns `RESTORE_QUIET_TIME` in
`openpilot/sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py`.

## What the constant does

Down-moves act after `REACT_TIMER` (0.3 s). Up-moves on cars whose profile declares
`decel_needs_stable_setpoint` (only `mazda`) additionally wait for the limiter target to hold
still for `RESTORE_QUIET_TIME`, currently 3.0 s. The timer resets on any target motion, on the
error closing, and on a pending confirm prompt.

The rule itself is well founded: MRCC will not commit to decelerating while the set speed is
still moving, so restoring the dash between two dips of a train both churns the dash and delays
the next deceleration. The *value* is not. It came from `docs/sla-icbm-redesign.md` §3 ("wait
~3-5 s of limiter quiet before restoring up"), a design-time estimate, unlike the constants
around it, which were fit to log corpora.

## Method

Simulated the servo against the recorded limiter target stream from every route in
`tools/mazda_long/device_data/` where a limiter drove the plan: 11 routes, 49 segments, 57,536
plan frames (of 1,802 segments swept). Down-moves unconditional, up-moves gated on stillness
`W`, dash capped at the driver's own setpoint, engaged frames only. Servo actuation is idealised
as instant, which is fair across `W` since every `W` pays the same actuation cost.

- **restores** - up-moves issued; a proxy for dash churn and button traffic.
- **regret** - up-moves followed by a drop within the horizon, i.e. restored into a dip.
- **lag** - mph-seconds the dash spends below the target it could have held; the speed the
  driver loses to the window.

Scripts are throwaway; the corpus sweep that finds limiter-bearing segments is the reusable
part (`longitudinalPlanSource != cruise`).

## Result

| W (s) | restores | regret (H=5 s) | lag (mph*s) |
|-------|---------:|---------------:|------------:|
| 0.0 | 885 | 67.7% | 388 |
| 0.5 | 139 | 30.2% | 2,473 |
| 1.0 | 100 | 27.0% | 3,619 |
| 1.5 | 94 | 26.6% | 4,498 |
| 2.0 | 92 | 27.2% | 5,171 |
| 3.0 (shipped) | 84 | 26.2% | 6,549 |
| 5.0 | 70 | 28.6% | 9,612 |

A window is clearly needed: at `W=0` two thirds of restores are immediately undone. But the
whole benefit is bought in the first second. Going 1.0 -> 3.0 s moves regret by 0.8 pp and saves
16 restores, while costing 81% more lag. Seconds two and three are close to pure delay.

## The signal, not the number

Stillness infers "no dip is coming" from the past. The plan already knows the future:
`liveMapDataSP.speedLimitAhead` and `speedLimitAheadDistance`,
`longitudinalPlanSP.speedLimit.resolver.distToSpeedLimit`, and SCC-V's own curvature preview.
An oracle version of that signal (restore only when the target genuinely holds for `L`) bounds
what any real predictor could buy:

| rule | restores | regret (H=2 s) | lag |
|------|---------:|---------------:|----:|
| stillness W=3.0 | 84 | 10.7% | 6,549 |
| oracle lookahead 2 s | 373 | 0.0% | 784 |
| stillness W=1.0 + oracle 3 s | 85 | 0.0% | 3,947 |

Lookahead alone is 4.4x the button traffic: it restores promptly and often, every move justified
but the dash busy. Combined with a small stillness floor it beats today on every axis: the same
restore count, no regret, 40% less lag.

## Recommendation

1. **Now:** `RESTORE_QUIET_TIME` 3.0 -> 1.0. One line. Keeps the jitter absorption that carries
   the benefit, halves the restore delay. `test_icbm_cruise.py:245` and `:262` compute from the
   constant and should still pass.
2. **Then:** gate the restore on lookahead - hold when a lower limit or curve target lies within
   ~3 s of travel - keeping the 1 s stillness floor as the backstop for target jitter and for
   cars with no usable preview.
3. Keep the per-car scoping as it is. `decel_needs_stable_setpoint` already limits this to Mazda;
   the flag is the right axis, the raw number never needed to be per-car.

## Caveats

- 11 routes is a thin corpus. It is every limiter-bearing route we have.
- Dips are dominated by SCC-V (2,553 of 2,616 next-dip sources measured); its curvature preview
  is less certain than a map speed limit, so a real lookahead gate will land between the
  stillness and oracle rows.
- The simulation assumes the dash tracks the target down immediately. Real actuation is 5 Hz taps
  and 5 mph holds, which adds latency uniformly across `W`.
