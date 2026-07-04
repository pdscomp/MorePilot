# Mazda ICBM: set-speed desync and oscillation — analysis and fixes

Evidence base: 447 rlog segments (routes `00000230`–`0000024d`, 2026-06-27 → 2026-07-03),
~78 minutes engaged, scanned with `tools/mazda_long/scan_icbm_episodes.py` (100 Hz timelines of
internal `vCruise`, real dash `CRZ_SPEED`, ICBM state, planner `vTarget`, real wheel button
edges/levels from CAN src=0, and openpilot-sent forged frames from `sendcan`).

## Reported symptoms

1. Cruise set speed sometimes ends up far above the value the driver dialed, typically while
   pressing the gas as sunnypilot brings the speed back up.
2. Rough behavior when adjusting the set speed manually while smart cruise (vision/map) is
   also adjusting it.

## What the data showed

### Symptom 1 — two-integrator desync under gas override (confirmed, 3 severe episodes)

For non-`pcmCruiseSpeed` (ICBM) cars, two independent counters track the set speed: the body
ECU's real dash value (stepped by wheel presses: 1 mph/tap, 5 mph long-press steps at the ECU's
own cadence) and openpilot's internal `v_cruise_kph` (stepped by button *events*: 1.6 kph/tap,
8.0 kph long-press snaps at openpilot's cadence). Whenever their stepping disagrees, ICBM
afterwards "corrects" the real dash to openpilot's stale value:

- `00000233--…--22` t≈1328–1332: driver taps SET− then 6× SET+ **with the gas pedal down**.
  Internal `vCruise` inflates 16 kph ahead of the dash (oversized long-press snaps, plus one
  confirmed `cruise.py` vEgo-clip where SET− *raised* the target to vEgo). On gas release, ICBM
  forges SET+ until the dash reaches the inflated value: the driver dialed ≈+6 mph, the dash
  landed **+17 mph** higher.
- `0000023d--…--9` t≈564–569: single 3.3 s SET+ hold on gas; internal snaps out-ran the ECU,
  ICBM pushed the dash 5 mph past the release point.
- `0000023b--…--1` t≈79–85 (mirror direction): the ECU applied its final 5 mph long-press step
  ~10 ms *after* release, after openpilot stopped mirroring the dash; ICBM later dragged the
  dash back **down** 5 mph with no driver input.

All 79 large single-frame `vCruise` jumps in the dataset were physical "+" holds (stock-like
behavior); there were zero spontaneous dash rises without a held button.

### Symptom 2 — ICBM hunting a noisy sccVision target (confirmed, 7 heavy segments)

One forged press moves the dash **exactly 1 mph**, with ~51 ms latency (median, n=195; the old
"+1 to +3 mph per send" observation is wrong). The oscillation comes from the planner target:
`sccVision`'s `vTarget`, rounded to mph, changes on 17 % of frames (single-frame jumps up to
5 mph). With no deadband, ICBM re-evaluates at 100 Hz and saturates its 0.2 s button pacing
ping-ponging SET+/SET− around the noise (up to 50 % of sends were direction reversals).

Real-time fighting between driver presses and ICBM sends essentially never happens (0 presses
landed while ICBM was mid-send in 4 days of data; the ECU never miscounted interleaved frames).
14 of 17 flagged dash divergences were by-design smart-cruise limiting.

## Fixes

1. **Dash re-sync** (`sunnypilot/selfdrive/car/cruise_ext.py` + hook in `selfdrive/car/cruise.py`):
   while the driver presses SET+/SET− and for 1 s after the last press, the real dash is adopted
   into `v_cruise_kph` — the ECU is the source of truth for driver-initiated changes. Skipped
   when the dash didn't match `v_cruise` at press start (i.e. ICBM is intentionally holding the
   dash away for SCC/SLA). Kills both directions of the desync.
2. **vEgo clip disabled for ICBM cars** (`selfdrive/car/cruise.py`): SET− while on the gas
   decrements on the stock ECU; clipping `v_cruise` up to vEgo only makes sense where the SET
   button sets current speed (`pcmCruiseSpeed` cars).
3. **Reaction deadband + persistence** (`…/intelligent_cruise_button_management/controller.py`):
   leave `holding` only when the target-vs-dash error is ≥ 2 display units *and* has persisted
   0.3 s; once moving, still run to the exact target. Closed-loop replay over the recorded
   traces: sends −50 %, direction reversals −83 %, with a genuine 24 mph tracking maneuver
   unaffected (58→56 sends).
4. **Same-frame driver-press suppression** (`opendbc…/sunnypilot/car/mazda/icbm.py`): the ICBM
   interface skips sending while the driver holds SET+/SET−, closing the few-frame
   messaging-latency window of the selfdrived readiness gate (the carcontroller's
   `icbm_suppress` keeps covering only cancel/resume, as before).
5. **Ramp-adaptive send pacing** (`opendbc…/sunnypilot/car/mazda/icbm.py`): after 3 consecutive
   same-direction sends the pacing tightens 0.2 s → 0.1 s (dash confirms a press in ~50 ms,
   p90 ~80 ms, so 0.1 s leaves margin); any direction change or ≥0.5 s pause resets. Replay of
   the `0000024c--…--6` deep dip: 21→45 mph recovery 3.2 s → 1.2 s, same total send count.
   Needs one on-car validation drive: confirm the ECU still counts every press at 0.1 s spacing
   (compare `sent_setp` vs dash steps with the scanner; if presses drop, back off to 0.15 s).
6. **SCC map controller quadratic-root fix** (upstream PR #1816, applied): operator precedence
   in `map_controller.py` made the moderate-curve deceleration distance ~11x too small, so map
   slowdowns started far too late. With the fix a 40 m waypoint at 25 m/s is detected (~45 m
   deceleration envelope vs ~4 m before).
7. **Deceleration overshoot** (ICBM controller; per-brand plant table, only Mazda populated):
   the stock ACC's deceleration
   scales with the dash-vs-ACTUAL-speed gap, not dash-vs-target. Measured response curve
   (422k hands-off cruise samples): ~0.09 m/s² per mph of gap, dead below ~2 mph, saturating
   near −0.75 m/s² by ~9 mph; decel onset lag ~1.0–1.3 s (n=4, preliminary). Commanding
   dash = target therefore does almost nothing until the car is already several mph hot.
   When the planner demands decel (`aTarget < −0.15`, source sccVision/sccMap/SLA), ICBM now
   commands `dash = vEgo − gap(aTarget)` (inverse plant map), capped at the planner target
   from above — down-only, so a stale command fail-safes to the car slowing. The command
   tracks vEgo through the maneuver and releases back to the target automatically as the car
   converges and `aTarget` relaxes (slew: 10 mph/s apply, 3 mph/s release to avoid pumping
   the ECU's discrete coast/downshift/brake stages). Closed-loop sim against the measured
   plant, moderate curve approach (45→33 mph over 12 s): arrival overspeed 6.7 → 3.6 mph,
   overspeed exposure −48%. Deep dips are unaffected (plant already saturated; the cap makes
   overshoot a no-op there, verified on the recorded 45→21 mph episode).
   Opt-in via the `SmartCruiseDecelOvershoot` toggle (default off): selfdrived reads it and
   passes it into `icbm.run()` (same pattern as `is_metric`); exposed in the MICI cruise
   layout ("decel overshoot"), the TICI cruise settings, and sunnylink SDUI — all gated on
   ICBM active + brand present in `DECEL_OVERSHOOT_PARAMS`.

Tests: `sunnypilot/selfdrive/car/tests/test_icbm_cruise.py`,
`opendbc…/sunnypilot/car/mazda/tests/test_icbm_pacing.py`,
`sunnypilot/…/smart_cruise_control/tests/test_map_controller.py::test_moderate_curve`.

## Follow-ups (not addressed here)

- `sccMap` briefly commanded 22.5 kph at 80 km/h (`00000244--…--5`, single ~1 s glitch); the
  0.3 s persistence filter blunts it, but the planner-side map sampling deserves a look. Note
  the PR #1816 fix changes map-controller timing substantially — re-check map behavior on the
  next drives.
- Validation drive items: (a) confirm the ECU counts every press at the 0.1 s ramp pace
  (compare `sent_setp` vs dash steps with the scanner; back off to 0.15 s if presses drop);
  (b) more decel-onset-lag samples (only 4 clean events in the dataset); (c) observe the
  overshoot behavior on a real curve — the plant model is built from steady-state bins.
- The kph/mph lattice skew (openpilot 1.6/8.0 kph steps vs dash 1.609/8.047) leaves a harmless
  ≤0.5 kph residual; dash re-sync absorbs it around manual presses.
