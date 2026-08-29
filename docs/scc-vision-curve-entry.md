# SCC-V curve entry: why the car arrives hot

Status: measured 2026-08-29, not yet implemented. Concerns
`openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/vision_controller.py`.

Reported symptom: the car often does not reach the desired speed early enough to be at that
speed *during* the curve. The logs agree, and the cause is not the deceleration ramp.

## Corpus

Every route in `tools/mazda_long/device_data/` where a limiter drove the plan: 11 routes, 49
segments (of 1,802 swept), all MAZDA_CX5_2022, a mix of openpilot longitudinal (5 routes) and
stock ACC driven through ICBM buttons (4 routes). SCC-V state, `aTarget`, `vTarget`,
`maxPredictedLateralAccel` are all logged, so the controller's own view is recoverable frame by
frame.

## What the logs show

**The car arrives hot.** Of curves whose apex reached the TURNING threshold, 54-59% exceeded
`_A_LAT_REG_MAX` (2.0 m/s^2) at the apex; p90 = 2.62, p99 = 3.13 m/s^2.

**The ramp is not the bottleneck.** Comparing the deceleration SCC-V commanded during ENTERING
against what the curve actually demanded over the distance available, the command was *stronger*
than required in 92% of cases (required p50 -0.51, commanded p50 -0.93 m/s^2).

**The command is not executed.**

| | openpilot long | stock ACC (ICBM) |
|---|---:|---:|
| ENTERING runs >= 1 s | 55 | 24 |
| commanded decel p50 | -0.59 | -0.75 |
| achieved decel p50 | -0.12 | -0.07 |
| achieved / commanded p50 | 0.19 | 0.06 |
| runs that did not slow at all | 42% | 46% |
| apex > 2.0 m/s^2 | 53% | 80% |

## Why: the published target sits above the current speed

`_update_calculations` derives the curve speed from the *current* speed:

```
max_curve = max_pred_lat_acc / v_ego**2
v_target  = (_A_LAT_REG_MAX / max_curve) ** 0.5
          = v_ego * sqrt(_A_LAT_REG_MAX / max_pred_lat_acc)
```

so `v_target > v_ego` for every predicted lateral acceleration below the 2.0 ceiling. The only
thing pulling the published value under the current speed is the `+ a_target *
_NO_OVERSHOOT_TIME_HORIZON` term, worth just -0.8 m/s at the entry threshold. A slowdown is
possible only when

```
v_ego * (sqrt(2 / L) - 1) < -4 * a(L)
```

At the ENTERING trigger (L = 1.3, a = -0.2) that means `v_ego < 3.3 m/s` - 7 mph. Above walking
pace the state machine enters the turn and commands nothing.

Measured onset, against the closed form:

| approach speed | measured first bite (p10) | closed form |
|---|---:|---:|
| 10-15 m/s | 1.61 | 1.57-1.66 |
| 15-20 m/s | 1.70 | 1.66-1.72 |
| 20-25 m/s | 1.76 | 1.72-1.76 |

43% of all ENTERING frames publish a target at or above the current speed: 100% of frames at
predicted lat acc 1.3, still 46% at 1.6, and effectively none past 1.8. **The first third of
every curve approach is a no-op**, and the faster the approach the higher the predicted lateral
acceleration has to climb before anything happens - exactly backwards.

## Why it cannot self-correct: the distance is thrown away

`max_pred_lat_acc = np.percentile(predicted_lat_accels, 97)` reduces the model's whole horizon to
one scalar. Nothing downstream knows *where* the curve is, so no part of the controller can scale
the deceleration to the runway remaining. The ENTERING ramp is a fixed comfort curve in
`_ENTERING_SMOOTH_DECEL_V` keyed only on severity: a curve 30 m ahead and one 150 m ahead get the
same request.

## Proposal: kinematic curve entry

Superseded by `docs/curve-and-limit-planning.md`, which is the plan of record. The
sketch below is kept for the reasoning; the plan replaces peak-picking with a backward
pass over the path and corrects the deceleration budget.


The model already carries everything needed; the percentile is what discards it.

1. **Locate the peak, not just its height.** Take the argmax (or a high percentile with its
   index) of `rate_plan * vel_plan`, and read `modelV2.position.x` at that index for the distance
   to the curve.
2. **Use the model's own curvature there**, not the current speed:
   `curvature = rate_plan[i] / vel_plan[i]`, `v_curve = sqrt(_A_LAT_REG_MAX / curvature)`. This is
   the physically correct curve speed and drops below `v_ego` exactly when the car is too fast for
   the curve ahead - which is the condition the current formula fails to express.
3. **Command the deceleration the geometry requires:**
   `a_req = (v_curve**2 - v_ego**2) / (2 * d)`, then
   `a_target = min(comfort_ramp, a_req)` clipped to a comfort floor. A distant curve keeps
   today's gentle ramp; a close one gets what it needs, and the onset moves earlier on fast
   approaches instead of later.
4. **Publish `v_curve` directly** rather than `v_target + a_target * 4`. The horizon term is a
   proxy for "where we will be in 4 s"; with a real required-decel calculation it is redundant and
   is currently the only thing that makes the target bite at all.

This also feeds the ICBM work in `docs/icbm-restore-quiet-window.md`: a controller that knows the
distance to the next curve can answer "is a dip coming?" directly, which is the lookahead signal
the restore gate wants.

## Caveats

- 11 routes, 79 ENTERING runs over 1 s. Thin, but the failure is structural and reproduces in the
  closed form, not just in the sample.
- The achieved/commanded ratio conflates two things under stock ACC: SCC-V's request and the ICBM
  button path that has to realise it. Under openpilot longitudinal the ratio is still only 0.19,
  so the request itself is the larger problem.
- `_A_LAT_REG_MAX = 2.0` is the comfort ceiling being missed; whether 2.0 is the right ceiling for
  the CX-5 is a separate question (see `carrot-lateral-evaluated` for the 4.5 m/s^2 variant that
  was measured and rejected).
