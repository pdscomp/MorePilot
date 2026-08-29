# Curve and limit speed planning: plan of record

Status: planned 2026-08-29, reviewed against the code the same day (every cited line and
mechanism verified; the review added the mpc seed consumer, the SLA mirror path, and the jerk
ramp sizing). Nothing implemented yet. Supersedes the proposal sections of
`docs/scc-vision-curve-entry.md` and `docs/icbm-restore-quiet-window.md`; both remain the
evidence of record for the measurements cited here.

Scope: SCC-Vision, SCC-Map, Speed Limit Assist, the plan-source arbitration, and the ICBM
restore gate - the whole path from "there is something ahead" to "the dash or the MPC acts".

## The principle

Hold the set speed as long as possible, then decelerate at the budget the platform can actually
deliver, arriving at the constraint at the correct speed. This is what the driver in our own logs
does: over the 4 s approach to a curve their median acceleration is -0.03 m/s2 (they coast) and
their p10 is -1.36 m/s2 (when they brake, they brake). SCC-V today does the opposite - a smear of
gentle nothing that never arrives.

Braking late is also the *better informed* choice: vision curvature at 150 m is noisy, at 60 m it
is solid. Starting early means committing on the worst estimate of that curve you will ever have.

## What the measurements established

From 11 limiter-bearing routes (49 segments of 1,802 swept), all CX-5 2022:

- 54-59% of curve apexes exceed `_A_LAT_REG_MAX` (2.0 m/s2); p90 2.62, p99 3.13.
- SCC-V's commanded decel is *stronger* than the curve required in 92% of cases, yet
  achieved/commanded is 0.19 (openpilot long) and 0.06 (stock ACC). 42-46% of ENTERING runs never
  slow at all.
- 43% of ENTERING frames publish a target at or above current speed, because
  `v_target = v_ego * sqrt(2.0 / max_pred_lat_acc)` is above `v_ego` for any predicted lateral
  acceleration under the 2.0 ceiling. Onset moves *later* the faster the approach.
- `RESTORE_QUIET_TIME`'s whole benefit is bought in the first second: W=1.0 gives 27.0% regret at
  3,619 mph*s lag, W=3.0 gives 26.2% at 6,549.

## Design

### One solver, a backward pass over the path

A curve is a region, not a point, and the horizon can hold several. Peak-picking plus a deadline
handles neither. The standard formulation does both, and yields the required deceleration as a
by-product:

```
v_allowed[i] = sqrt(a_lat_max / curvature[i])                     # geometry only
v_max[N]     = v_allowed[N]
v_max[i]     = min(v_allowed[i], sqrt(v_max[i+1]**2 + 2*a_budget*ds[i]))
```

Curvature must come from geometry - `orientationRate.z / velocity.x` - not from
`orientationRate.z * velocity.x`. The current lateral-acceleration form is speed-dependent, so as
the car slows the predicted value falls and trips `_ABORT_ENTERING_PRED_LAT_ACC_TH`: the
controller talks itself out of the slowdown it just started.

`ds` comes from `modelV2.position.x` deltas; the model horizon is 10 s of travel (33 points,
`T_IDXS[-1] = 10.0`), so lookahead distance scales with speed.

### Two output channels, both of which already exist

`longitudinalPlanSP` already carries `vTarget` (destination) and `aTarget` (how hard), and both
consumers already read them:

- **openpilot longitudinal** takes `vTarget` as `v_cruise`, and
  `get_cruise_accel` (`selfdrive/controls/lib/longitudinal_planner.py:51`) turns it into
  `np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)` with `A_CRUISE_MIN = -1.2`. That is a P
  controller with gain 1 s^-1 and a hard -1.2 m/s2 clip. Two consequences: the openpilot-long
  budget is **1.2, not the -1.45 p95 measured** (that came from the MPC's lead candidate, not from
  a speed target), and to command decel `a` the published target must sit `a` m/s **below**
  `v_ego`. So the output is `min(v_profile, v_ego - a_required)`. Against a *fixed* target a
  P controller fades exponentially and never quite arrives; the re-solve each frame keeps the
  target tracking `v_ego` down, holding the gap - and the commanded decel - until the profile is
  met. The moving target is the mechanism, not a side effect.
- **openpilot longitudinal also consumes `aTarget`** (found in review): the winning source's
  `aTarget` is fed to `mpc.set_cur_state` as the MPC's initial acceleration and into the
  `v_desired_filter` integration (`longitudinal_planner.py:116-119,157`). The MPC candidate built
  from that seed is **not** jerk-limited the way the cruise candidate is, so publishing a step
  from ~0 to -1.2 in one frame reaches the actuators as a snap for the first few frames. The
  published `aTarget` must therefore carry the profile's own jerk-in ramp
  (`max(a_required, a_prev - j*dt)`), not the raw budget.
- **stock ACC** goes through ICBM, whose decel-overshoot converts a requested decel into a dash
  gap when `LP_SP.aTarget < -min_decel and CS.vEgo > LP_SP.vTarget`
  (`intelligent_cruise_button_management/controller.py:133`). `DECEL_OVERSHOOT_SOURCES` already
  lists all three limiter sources.

The wire is single-valued but the budget is per-car, not per-consumer: a car is either
openpilot-long or stock ACC for the whole drive, so the planner picks the budget from
`CP.openpilotLongitudinalControl` at init.

Note the `+ a_target * _NO_OVERSHOOT_TIME_HORIZON` term in SCC-V is not conceptually wrong: the
target does have to lead `v_ego` to do anything. It is mis-sized - 4 s of an unrelated comfort
ramp instead of the decel actually required.

`MIN_V` (20 km/h) floors every limiter target, so the profile cannot command below ~5.6 m/s.
At a 2.2 budget that only binds for curvature above ~0.07 1/m (a parking-lot hairpin); keep the
floor.

### Per-path deceleration budget

| path | budget | why |
|---|---:|---|
| openpilot longitudinal | 1.2 m/s2 | `A_CRUISE_MIN`, a hard plumbing clip |
| stock ACC via MRCC | 0.75 m/s2 | measured `DECEL_OVERSHOOT_PARAMS` saturation (422k samples); deeper gaps buy nothing |

Both are close to the driver's own p10 (-1.36 / -0.90 on the two route groups), so they are
comfort-consistent, not merely what the hardware permits.

Where that puts the braking point, 45 -> 34 mph:

| budget | distance | hold set speed until |
|---|---:|---:|
| today's ramp (~0.5) | 175 m | 8.8 s out |
| stock ACC (0.75) | 117 m | 5.8 s out |
| openpilot long (1.2) | 73 m | **3.6 s out** |

**Horizon limit.** With `d_needed <= 10 * v_ego`, a 30% speed drop is coverable up to ~66 mph on
the stock budget and never binds on openpilot long. Above that, vision alone cannot start early
enough and the map source has to carry it - and map coverage is thin (11 frames in the whole
corpus).

### Margins

```
d_needed = (v_ego**2 - v_target**2) / (2 * a_budget)   # physics
         + v_ego * t_lead                              # actuation lead, see below
         + jerk ramp distance                          # from J_CRUISE_VALS, see below
trigger when d_remaining <= d_needed / commit_fraction  # commit_fraction ~0.8
```

- `t_lead` on openpilot long is `longitudinalActuatorDelay` (0.36 s for mazda). On stock ACC it is
  **dash traversal + MRCC response**, i.e. seconds: moving the dash 20 mph is ~2.3 s of 5 mph
  holds per `ICBMActuationProfile`. A deadline computed without this is systematically late on the
  stock path.
- The jerk ramp is not free to size: on openpilot long the cruise candidate is clipped to
  `J_CRUISE_VALS` (0.8 at 25 m/s, 0.6 at 40), so reaching -1.2 from 0 takes 1.5-2 s at highway
  speed. Compute the ramp distance from `J_CRUISE_VALS`, roughly `v_ego * a_budget / (2 * j)`
  (~13 m at 20 m/s), rather than assuming 0.5-1 s.
- `commit_fraction` at 0.8 leaves headroom. Triggering at exactly the budget means any error -
  a slope, a bad curvature sample, a slow actuator - has nowhere to go.
- Re-solve every frame so the profile releases the moment the constraint relaxes. This is also the
  mitigation for false positives, which late firm braking makes more noticeable than the current
  ignorable smear.

## Rejected during review

Recorded so they are not re-proposed.

- **Urgency-based arbitration** (replacing `min(targets, key=...)` in
  `sunnypilot/.../longitudinal_planner.py:81`). The premise was that a distant low limit outranks
  an imminent curve. It does not: every source self-gates on distance before publishing - the SLA
  resolver only swaps in the upcoming lower limit once
  `distance_to_speed_limit_ahead <= adapt_distance` (`speed_limit_resolver.py:150`), and SCC-Map
  only activates inside its braking distance. `min()` over already-gated "you need this now"
  values is correct. SCC-V is the only source that fails to self-gate, which is Phase 2.
  (Separately: that resolver gate carries a `# FIXME-SP: this is not working as expected`
  comment and deserves its own investigation.)
- **Peak-picking plus a deadline solver.** Superseded by the backward pass, which handles extended
  curves and multiple curves for the same cost.

## Phases

Each is independently landable. 0 and 4 depend on nothing.

**Phase 0 - restore quiet window.** `RESTORE_QUIET_TIME` 3.0 -> 1.0.
Files: `intelligent_cruise_button_management/controller.py`.
Validation: `test_icbm_cruise.py:245,262` compute from the constant.
Risk: minimal.

**Phase 1 - shared solver.** New `smart_cruise_control/speed_profile.py`: `allowed_speed`,
`backward_pass`, `required_decel`, `lead_distance`. Pure functions, no I/O.
Validation: unit tests for extended curve, two curves, horizon truncation, zero curvature,
degenerate `ds`.
Risk: none, nothing consumes it yet.

**Phase 2 - SCC-V adopts the solver.** Geometry curvature, backward pass, publish
`min(v_profile, v_ego - a_required)` and a real `a_target` - jerk-ramped, never a one-frame step
(it seeds `mpc.set_cur_state`, see the output-channels section). Keep the state machine for UI
and alerts; it stops being load-bearing.
Validation: replay the 49 limiter segments and compare the apex lateral-acceleration
distribution; the 54-59% overshoot rate should collapse. Existing `test_vision_controller.py`
asserts the current formula and will need rewriting with it.
Risk: **this is the behavioural cliff.** The feature is near-inert today; once it works it acts in
roughly half of all curves. Ship behind the existing `SmartCruiseControlVision` toggle and
validate on car before it becomes default.

**Phase 3 - per-path budget and lead time.** Budget from a per-car profile (1.2 / 0.75), selected
once at planner init from `CP.openpilotLongitudinalControl`; stock-ACC lead term from
`ICBMActuationProfile`.
Validation: replay both route groups separately; achieved/commanded should approach 1.0.
Risk: low, but the stock-path lead term is the easiest thing to get wrong.

**Phase 4 - map and SLA publish a real `a_target`.** Both currently
`return self.a_ego` (`map_controller.py:97`, `speed_limit_assist.py:128`, the latter under a
`# TODO-SP` acknowledging it). Because ICBM keys the dash-gap on `aTarget`, **map curves and
speed limits never get the overshoot treatment on stock ACC** except by coincidence when `a_ego`
happens to be negative enough. Three code sites, found in review:

- `speed_limit_assist.py` already *contains* the physics, dead: `acceleration_solutions`
  (built at `:89`, keyed by state - adapting gets `(limit^2 - v_ego^2) / (2d)`, active gets
  `v_offset / T_IDXS[CONTROL_N]`) is never called; `get_a_target_from_control` ignores it and
  returns `a_ego`. On the **pcm-op-long path** the fix is wiring the existing table in.
- On the **non-pcm path - every stock-ACC car including the CX-5 - the SLA machine runs in card
  and plannerd's SLA is `assist_mirror.py`, which hard-codes `output_a_target = a_ego`.** Fixing
  `speed_limit_assist.py` alone changes nothing on the car this project targets. The mirror
  should compute the decel locally from `vCap` and the resolver's distance (the resolver runs in
  plannerd on both paths), keeping the card wire unchanged.
- `map_controller.py` gets the solver's required decel (or, pre-Phase-1, the same
  `(v_target^2 - v_ego^2) / (2d)` form).

Validation: replay stock-ACC routes, confirm the overshoot gap engages on `sccMap` and
`speedLimitAssist` sources.
Risk: low. Bug fix, not redesign. Independent of Phases 1-3.

**Phase 5 - converge map and SLA onto the shared solver.** Retire three separate comfort budgets
(map: jerk -0.6 / accel -1.2 / offset 1 s; SLA: `LIMIT_ADAPT_ACC` -1.0; vision:
`_ENTERING_SMOOTH_DECEL_V`) onto one per-car profile. Mostly deletion.
Risk: low, but touches SLA behaviour that has its own on-car history.

**Phase 6 - ICBM lookahead gate.** The solver's constraint list answers "is another dip within N
seconds?" directly; replace the 1 s stillness rule with it, keeping 1 s as fallback where no
preview exists. On stock ACC feed ICBM `min(profile over the next T seconds)` - 1 mph taps cannot
track a continuous profile and the dash has to be pre-positioned anyway.
Validation: rerun the `W` sweep with the lookahead gate against the corpus.

## Open decisions

1. **Lateral acceleration ceiling.** `_A_LAT_REG_MAX` is 2.0; measured apex p50 is 2.01, so at 2.0
   this intervenes in half of all curves once it works. 2.2 (roughly the measured p75) matches how
   the car is actually driven. Recommend 2.2, exposed rather than hard-coded.
2. **Phase 2 rollout.** Default-off behind the existing toggle, or straight replacement given the
   current behaviour is near-inert.

## Caveats

- 11 routes, 79 ENTERING runs over 1 s, 36 driver curve approaches. The failures are structural
  and reproduce in closed form, but the constants (1.2 / 0.75 / 2.2) deserve a larger sample
  before they harden.
- The 0.75 stock-ACC figure is the *gap lever's* saturation, not the car's braking capability.
- Nothing here is on-car validated.
