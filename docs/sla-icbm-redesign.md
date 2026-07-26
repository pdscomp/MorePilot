# SLA + ICBM redesign — research, evidence, and target architecture

Status: **IMPLEMENTED 2026-07-25**. Main repo: `2da377fd0a` (slice 1, single-writer setpoint),
`1ff3482c6c` (slice 2, servo), `59773719a3` (slice 3, SLA guard), `57b07bcd91` (loop harness plus
two harness-caught reconciliation fixes), `65640115f3` (seg16 replay), `d042533245` (submodule
bump), `b53ae40f4e` (cleanup pass), `1cfa7ac46c` (press-timing sweeps and driver-interaction
tests). opendbc: `d57a5a8210` (slice 4, profile + emission), `632d7017f3`, `996a9a476b`.
Implementation notes vs the design: no press-sent feedback channel was needed (CAN controllers
don't self-receive, so buttonEvents are driver-only by construction and attribution is free);
the hold-fallback watchdog faults only on zero dash movement (an ECU reading holds as paced taps
makes tap-rate progress and needs no fallback); dash adoption additionally requires the dash to
have been at an intended resting value (setpoint or SLA target) at press start; SLA button
ownership is latched at the press edge; the restore quiet window is gated per-car on
decel_needs_stable_setpoint. Pending on-car: injected-hold integration, down-holds,
decel-overshoot feel. This document was the reference for a
"big-bang" refactor + release of Speed Limit Assist (SLA) and the Intelligent Cruise Button Management
(ICBM) set-speed control, addressing every defect found in the 2026-07-25 investigation. It is written
to be resumable in a fresh session and to be sliced into upstreamable sunnypilot PRs.

**Decisions locked with the owner (2026-07-25):**
1. **Wrong-limit escape / driver override:** upstream-default button feel. `+` while the servo is
   mid-decrease for SLA aborts and restores `driver_setpoint` (the one deviation from upstream, which
   would strand the dash mid-way). `+`/`-` once settled = plain ±1 / hold-to-climb; setpoint
   re-anchors; SLA → inactive until the next limit change (upstream's own guard, now firing only on
   genuine driver presses).
2. **Long-press actuation ships in this release.** Owner is confident and will test safely on-car.
   Runtime fallback to taps if the dash doesn't snap +5 (see §4.4).
3. **`+/-` confirmation is KEPT and fixed** (not deleted, no RES). Sunnypilot's intended confirm UX,
   made workable by press attribution (see §4.5). RES "follow limit up" is dropped from this release.
4. **SLA changes scope to the non-pcm (ICBM) path only.** The pcm-op-long state machine is untouched.
5. **Single release** on `mazda-dev` — all slices land together; slices exist for upstream authorship
   only.

Analysis scripts: `tools/mazda_long/icbm_sla/` (see [Reproducing the analysis](#reproducing-the-analysis)).
Prior related doc: `docs/mazda-icbm-desync-analysis.md` (the earlier desync fix, commit `a3c625fb16`).

---

## 0. TL;DR

- The set speed on a stock-ACC Mazda lives **inside the radar/ACC ECU**; the only input to it is the
  `CRZ_BTNS` button message. There is **no writable "set speed = X" frame**. Direct speed control is
  only possible via openpilot longitudinal (send `ACCEL_CMD`), which is the separate `mazda-dev-long`
  project. So while stock ACC is in charge, button injection is the only lever.
- Every SLA/ICBM defect found is an **emergent interaction between too many mutable states**. The fix
  is to collapse the state, not to patch each symptom.
- **Core design:** keep the driver's setpoint and the limiter target *separate*; make actuation a pure
  servo to `min(driver_setpoint, limit_target)`; actuate **decisively, then go silent**; abstract the
  actuator so op-long can replace button injection later.
- The bugs live in **sunnypilot shared files** — the fix belongs there, generalized, with a per-car
  actuation profile. This benefits every ICBM car and is upstreamable.

---

## 1. Scope and evidence base

Vehicle: Mazda CX-5 2022, `openpilotLongitudinalControl = False` (stock MRCC ACC),
`pcmCruise = True`, `pcmCruiseSpeed = False` (→ ICBM active, "non-pcm" SLA path).

Logs used (how each was obtained):

| Purpose | Route / file | Build commit | Notes |
|---|---|---|---|
| Confirm-bug capture | `952c07dea500f4e2/0000004f--fea08aad07/16` (local Desktop rlog) | `0a0ae2f6` (225 behind HEAD) | 65 mph zone, dash set 50 |
| Ratchet + curve/stop | `952c07dea500f4e2/000000c9--0850e1c117` seg 32–33 (via comma connect) | `a8fd4db3` (**= HEAD**) | stop sign after tight curve, ~16:36–16:38 |
| Long-press corpus | `tools/mazda_long/test_data/drive_*` (674 segments) | various mazda-dev | driver button holds |
| Parked (no SLA) | `2abbbcb9debdffe6_00000016--958fd5791a--0` | `ec8a065877` | ruled out — see §2 |

Connect access works via `~/.comma/auth.json`; `LogReader("<dongle>/<route>/<seg>")` fetches segments.

---

## 2. Method (how the CAN was decoded)

All findings come from replaying rlogs with `openpilot.tools.lib.logreader.LogReader` and correlating:

- **Set speed** = `CRZ_EVENTS` (msg 543) signal `CRZ_SPEED`: 16-bit Motorola @ start-bit 7,
  `phys_kph = raw*0.005 - 0.5`, `raw = (dat[0]<<8)|dat[1]`; ×0.621 → mph. Same value openpilot reads as
  `carState.cruiseState.speed` (`opendbc/car/mazda/carstate.py:89`). Validated: decoded values match the
  `cruiseState.speedCluster` timeline exactly.
- **Buttons** = `CRZ_BTNS` (msg 157) byte 0 bits: `CAN_OFF`=b0, `RES`=b2, `SET_P`=b4, `SET_M`=b5,
  `DISTANCE_LESS`=b7.
  - **Driver** presses = `can` stream, `src == 0` (powertrain bus).
  - **openpilot** injects = `sendcan` stream.
  - Driver *holds* are isolated by run-length: the wheel emits the button bit =1 for many consecutive
    50 Hz frames; openpilot never holds (it pulses one frame then waits ≥100 ms), so any run of ≥3
    consecutive `SET_P/SET_M`=1 frames is unambiguously a driver hold.
- **Acceleration command** = `CRZ_INFO` (msg 539) signal `ACCEL_CMD` (13-bit, checksum + counter) — the
  ECU's *output*; the channel op-long would overwrite.
- SLA/ICBM state from `longitudinalPlanSP` (`speedLimit.assist.state`, `resolver.*`,
  `longitudinalPlanSource`, `vTarget`) and `selfdriveStateSP.intelligentCruiseButtonManagement`
  (`state`, `sendButton`).

The parked Desktop segment was ruled out first: max 5.8 mph, 70 % standstill, cruise never engaged, no
valid speed limit all segment (offline maps = Canada with empty `OsmStateName`) → SLA stayed `disabled`.
It cannot show any SLA behavior.

---

## 3. Findings (each with evidence and how determined)

### F1 — SLA `+/-` confirmation self-destructs on stock-ACC cars

Data: confirm-bug log, event t≈977–988 (65 mph zone, dash set 50, SLA target 70 = 65 + 5 offset).
Timeline (`sla_icbm_timeline.py`): SLA → `preActive` ("press + to confirm"); driver taps `SET+`; stock
dash 50→51; SLA reads the button, `→ active`; **next frame** the +1 dash change propagates into
`CS.vCruiseCluster`, and SLA's own guard `if v_cruise_cluster_changed: state = inactive`
(`speed_limit_assist.py`, `update_state_machine_non_pcm_long`) fires → SLA deactivates. It can never
stick; the driver mashed `SET+` five+ times (dash 50→70), SLA flickering active/inactive the whole way.

Root cause: on a stock-ACC car the confirm press is *physically the same* `CRZ_BTNS` the ECU integrates,
so one tap both (a) bumps the dash ±1 and (b) trips the "manual set-speed change → inactive" guard.
`+/-` confirmation is structurally impossible here. Present in **HEAD** — the desync fix never touched
this guard. (It worked "once at high speed" because above the 50 mph `CONFIRM_SPEED_THRESHOLD` a limit
≥ threshold auto-applies with no press.)

### F2 — ICBM ratchets the set speed *down* and never restores to the driver's baseline

Data: curve/stop log seg 32, commit `a8fd4db3` (**HEAD**), t≈1921–2004 (`curve_stop_timeline.py`).
Driver baseline `vTarget = 60` the entire window. A tight curve makes SCC-vision dip `vTarget` to ~58
for a moment → ICBM `decreasing` → one `SET-` → dash 60→59. SCC releases instantly (`vTarget` back to
60), leaving dash 59, target 60, error **1 mph**. `REACT_DEADBAND = 2` (`controller.py:28`) → ICBM
`holding`, **never restores**. Dash sat 1 mph low for 75 s until the *driver* pressed `SET+` at t=2003.6
to fix it themselves.

Root cause: the deadband is symmetric on *error* but the *errors are asymmetric* — transient limiter
dips reliably clear the deadband going down (decrement), but the recovery gap is always sub-deadband
going up, so it is stranded. Across multiple curves this ratchets 60→59→57… ("random" low values).
Note `vTarget`/`v_cruise` stays correct (60) — driver intent is not lost; only the **dash** isn't walked
back. The fix is in the actuator, not intent storage.

### F3 — Two-integrator desync / `V_CRUISE_MAX` chase (already fixed, but the mechanism matters)

Data: confirm-bug log (old build `0a0ae2f6`), after the driver gave up, ICBM chased `vT = 90.1 mph`
(= `V_CRUISE_MAX`, 145 km/h) and drove the dash 70→**90** in a 65 zone until the driver cancelled at
72.9 mph. This is the desync fixed by `a3c625fb16` (dash re-sync + deadband). That user was simply on an
old build. The lesson for the redesign: the re-sync heuristic (`cruise_ext.py:131-153`, "adopt dash while
driver presses + 1 s, unless a limiter holds it away") is fragile — a ±2 mph agreement window can still
leak a small residual — and should be replaced by deterministic injection accounting (see §4.1).

### F4 — Faster button pacing is *counterproductive* (the ECU drops presses)

Data (`button_press_efficiency.py`, `button_press_bursts.py`): measured injected `SET_P` frames vs
achieved `CRZ_SPEED` delta.

| Build / cadence | mph per press | effective rate |
|---|---|---|
| `0a0ae2f6`, fixed 210 ms (~5 Hz) | **0.93** (reliable) | ~4.5 mph/s |
| `a8fd4db3` HEAD, ramp-adaptive 111 ms (~9 Hz) | **0.47 (~half dropped)** | ~2.9 mph/s |

The `057cd7d70b` ramp-adaptive pacing (`PACE_RAMP=0.1s` after 3 sends, `icbm.py`) doubles CAN traffic
but the ECU only registers ~half the presses — no speed gain, arguably slower. The body ECU reliably
counts **~1 discrete press per ~200 ms (~5/s)**. Likely mechanism: the `[1,1,0,None]` counter-offset
scheme collides at tight spacing (real `CRZ_BTNS` CTR advances only ~5–6 counts per 111 ms vs ~10 per
210 ms → duplicate-counter rejects). **Conclusion: drop the ramp, hold steady ~5 Hz.**

### F5 — Long-press profile (characterized from 52 corpus episodes)

Data (`longpress_scan.py`, `longpress_detail.py`, `longpress_corpus.py` →
`longpress_corpus_results.json`). Driver holds `SET_P/SET_M`; the stock ECU responds:

| Action | Result |
|---|---|
| Tap (<~0.3 s) | **+1 mph** |
| Short hold (~0.5–1.0 s) | **+5 mph**, snapped to next multiple of 5 |
| Long hold (>~1.1 s) | +5, then **+5 every ~0.55 s** (ramp **~7–9 mph/s**) |

- Ends on multiples of 5 (24/26 clean up-holds). Speed limits are also 5-multiples → one long-press
  lands a limit exactly.
- Step *k* lands ≈ `0.6 + (k-1)·0.55 s`. Step pattern `[1,4,5,5,…]` = +1 then +4 to align, then +5s.
- **Trailing +5** ~15 % of holds, only when released mid-cycle → release just after a step lands to
  avoid it.
- **Down direction under-sampled** (n=3; drivers rarely *hold* down): looks symmetric (−5, 5-aligned,
  ~−6.4 mph/s, no trailing seen) but **low confidence — confirm on-car / closed-loop before relying on
  it for the decel path.**

### F6 — MRCC will not decelerate until the set speed stops changing (owner, on-car)

The stock ACC does not begin decel while the set speed is still being adjusted; continuous button
"tracking" therefore *delays* the car's response. This makes flapping (F2's failure mode) doubly bad and
inverts the actuation goal from "track the target continuously" to **"make one decisive move, then go
silent so MRCC commits."**

---

## 4. Target architecture

One idea drives the simplification. Today four state machines interact (SLA ×2 variants, ICBM,
`cruise_ext` re-sync) and the planner **min's the driver's cruise target and the limiter targets into a
single `vTarget`**, so the actuator cannot distinguish "restore to the driver's baseline" from "track a
limiter." That lost distinction *is* F1 and F2.

**Keep the driver setpoint and the limiter target separate; servo to `min(them)`.** Three small
single-purpose pieces replace the four coupled machines.

### 4.1 Driver setpoint — one immutable source of truth

`driver_setpoint` is written by exactly one thing: driver button events (+ initial SET). No limiter, no
re-sync heuristic writes it. On button-injection cars, infer driver intent by **injection accounting**:
ICBM knows exactly which presses it sent, so `expected_dash = last_dash + our_injected_steps`; any
deviation is attributed to the driver. This replaces the fragile time-window re-sync (F3) with a
deterministic, log-replayable rule. Single-writer is a grep-enforceable invariant — the whole class of
"something moved my set speed" bugs becomes impossible.

### 4.2 Limiter arbitration — shadow down only

Each limiter (SLA, SCC-vision, SCC-map) publishes `(target, active)`. `limit_target = min` of active
ones. Publish **both** `driver_setpoint` and `limit_target` in `longitudinalPlanSP` (not just the final
min) so the actuator knows the regime. `desired = min(driver_setpoint, limit_target)`. Limiters only
ever slow you; raising above the driver setpoint is a separate opt-in (§4.5).

Driver presses while **SCC** (curve/map) is binding: the press re-anchors `driver_setpoint` (single
writer, always attributed) but SCC keeps binding — `min` still wins, matching upstream, where ICBM
would press back down; the servo simply holds the curve target and the restore-after-quiet then goes
to the *new* setpoint. Only SLA has the confirm/abort press semantics of §4.5.

### 4.3 Actuator abstraction — the op-long seam

```
SetSpeedActuator.command(desired, driver_setpoint, context)
```
- **DirectActuator** — `pcmCruiseSpeed` cars / future op-long: set `v_cruise = desired`. Trivial.
- **ButtonActuator** — ICBM: the servo in §4.4.

Everything above this seam is identical for all cars and both actuation modes. When `mazda-dev-long`
(op-long, send `ACCEL_CMD`) matures, it is a **backend swap** — SLA/SCC/driver-intent code is untouched.

### 4.4 ButtonActuator servo (shared logic, per-car profile)

Built from F4/F5/F6:
1. **Round the LIMITER target to the car's grid** (5 mph on Mazda; matches limits and the long-press
   step); round **down** for decel (owner's 40→35 example). Restore-to-driver moves are always exact
   (the driver can dial non-grid values like 62) — the servo has a grid mode (limiter coarse moves)
   and an exact mode (restore + remainder taps).
2. **Plan one decisive move:** long-press for the coarse part (timed from the car's long-press profile,
   F5), taps for the remainder. **Ships in this release** (owner decision; on-car test during release
   validation). Caveat: F5 characterized *physical* holds; an injected hold interleaves with the
   wheel's genuine `SET_P=0` frames at 50 Hz and may read as taps. **Runtime fallback:** if the dash
   doesn't snap +5 within the expected step window, degrade that move (and the session) to counted 5 Hz
   taps. After any long-press burst, reconcile expected-vs-dash during a settle window (~1 s, like
   `DASH_SYNC_SETTLE_TIME`) before attributing residuals to the driver — the ~15 % trailing +5 makes
   hold outcomes non-deterministic (§4.1 accounting must absorb this).
3. **Go silent** until the target moves more than the anti-flap band — forced by F6. Asymmetric
   patience: act fast going *down*; wait ~3–5 s of limiter quiet before restoring *up* (back-to-back
   curves — F2's own log — would otherwise churn the dash and delay curve-2 decel per F6).
4. **Restore to `driver_setpoint` is exact** (no deadband against the baseline — that was F2). The
   anti-flap band applies only to jittery *limiter* targets, filtered at the source (not the old
   symmetric servo deadband).
5. **Decel-overshoot composes with, not against, "decisive then silent":** the existing
   `DECEL_OVERSHOOT_*` mechanism (`controller.py:42-57`, measured from 422k samples) survives — a
   decisive move during planner-demanded decel means one burst down to the overshoot gap below vEgo,
   hold while the ECU commits, then the slow release (3 mph/s) walks it back as today. It is the one
   sanctioned exception to "silent": its release-side updates are slow and monotonic, which the ECU
   tolerates (measured), unlike target-tracking flapping (F6).

Per-car `ICBMActuationProfile` (lives in the opendbc car port, like `CarControllerParams`):
```
tap_increment            # Mazda: 1 mph
align_grid               # Mazda: 5 mph
has_longpress            # Mazda: True
longpress_step           # Mazda: 5 mph
longpress_rate           # Mazda: ~8 mph/s (step lands ≈ 0.6 + (k-1)*0.55 s)
longpress_threshold      # Mazda: hold ≳0.5 s → first +5; ramp after ~1.1 s
longpress_trailing       # Mazda: +1 step if released mid-cycle → release-just-after-step
tap_rate_hz              # Mazda: 5 (NOT the counterproductive ramp — F4)
decel_needs_stable_setpoint  # Mazda: True (F6)
```
Cars without a characterized profile get a **safe default**: taps-only, no grid, no long-press, current
behavior. No car regresses unless it opts in. Mazda fills in the measured profile.

### 4.5 SLA — keep sunnypilot's `+/-` confirm, fix it with press attribution (locked 2026-07-25)

An earlier draft deleted button-confirm. **Decision: keep the intended sunnypilot UX** — press `+` to
confirm a higher limit, `-` to confirm a lower one, auto-apply at/above the confirm-speed threshold —
and fix F1 by *attribution* instead. F1's root cause is not `+/-` itself; it's that the override guard
treats **every** dash change as a manual override. With single-writer + injection accounting (§4.1),
every dash change has a known provenance, and the guard distinguishes three cases:

1. **The confirm press** (while preActive with `req_plus`/`req_minus`): consumed as confirmation; its
   ±1 dash side-effect is *expected* and never trips the guard. SLA → active sticks.
2. **Our own injected presses** (ICBM walking the dash to the SLA target after confirm): self-attributed
   via injection accounting; never trip the guard.
3. **An unexpected driver press** — the only true override. Sunnypilot-default semantics, with one
   refinement (locked 2026-07-25: "must not feel different from sunnypilot at the wheel"):
   - **`+`/`-` while SLA is *settled* at a limit** (servo idle): plain stock button behavior — dash
     steps ±1 per tap (ECU snap on a physical hold), `driver_setpoint` **re-anchors** to the resulting
     dash value, SLA → inactive until the next limit change. This is upstream's manual-change guard,
     now firing only on genuine driver presses (never on the confirm press or our injections — F1).
     It doubles as the wrong-limit escape once settled: press/hold `+` and climb; SLA won't re-drag
     until the limit actually changes.
   - **`+` while the servo is actively lowering for SLA** (move in flight): abort the move and restore
     `driver_setpoint` exactly — don't strand the driver halfway down (upstream would leave the dash
     wherever it was and deactivate). SLA → inactive until the next limit change. The only deliberate
     user-facing deviation from upstream, and the primary wrong-limit escape.
   - **`-` while the servo is lowering**: driver takes over (readiness gate already yields); setpoint
     anchors to the dash result, SLA → inactive.

RES "follow limit up": **dropped from this release** (ECU-ignores-RES-while-engaged is unverified,
no RES parsing exists). Raising to a higher limit is confirm-by-`+`, or the driver dialing manually.

**Scope: non-pcm path only.** The pcm-op-long machine (`update_state_machine_pcm_op_long`) is
untouched — confirm works as designed there (buttons go to openpilot, no F1). The non-pcm machine keeps
{disabled, inactive, preActive, active}; what changes is the guard logic (attribution + the mid-move
abort/restore), not the confirm flow.

**Limit-end / limit-raise semantics (locked 2026-07-25): identical to upstream sunnypilot.**
- **New higher limit posted** (45 zone opens to 65): SLA → preActive, "press `+` to confirm" below the
  confirm-speed threshold; auto-apply at/above it. On confirm, the attributed press (case 1 above)
  sticks, and the servo walks the dash up to `min(driver_setpoint, new limit target)` — one tap, ICBM
  does the presses, the driver gets their dialed speed back (or the new limit if lower).
- **Limit disappears with no successor** (resolver holds `speedLimitFinalLast`): nothing moves until
  the driver presses — then normal button behavior per case 3 (re-anchor). Same as upstream.
- SLA never raises the actuated speed without either the CST auto-apply condition or a driver press.
- Curve (SCC-vision) restores remain fully automatic and never touch `driver_setpoint` — curves last
  seconds; zones last minutes.

---

## 5. What's shared vs per-car (the upstream split)

| Shared (sunnypilot, all cars) | Per-car (opendbc port) |
|---|---|
| driver-setpoint single-writer + injection accounting | `ICBMActuationProfile` (increments, grid, long-press, tap rate) |
| limiter arbitration (`min`, shadow-down) | long-press emission + hold timing (`icbm.py`) |
| `SetSpeedActuator` interface + `ButtonActuator` servo | `decel_needs_stable_setpoint` flag |
| SLA non-pcm guard fix (press attribution + mid-move abort/restore) | per-frame "press sent" feedback (emission truth for accounting) |

**Plumbing note:** injection accounting needs emission-level truth — which frames were actually sent —
and that lives in the opendbc `CarController` (`icbm.py`'s `last_button_frame`), not the sunnypilot
layer. A per-frame "button press sent" feedback channel (carOutput-SP analog in the cereal schema) must
be added; the shared accounting layer consumes it. Two attribution hazards it must absorb: (a) ~7 % of
taps drop even at 5 Hz (F4) — a dropped press is NOT a driver press; (b) our own forged frames are
parsed by `carstate.py:129` and generate buttonEvents — the online layer must not self-attribute to the
driver. Rule: attribute to the driver only deviations that survive the settle window AND correlate with
genuine (non-self) press timing; unexplained residuals adopt the dash conservatively.

**Yes, the sunnypilot shared implementation must change** (`speed_limit_assist.py`, ICBM
`controller.py`, `cruise_ext.py`). Fixing Mazda-only would fork three shared files (high merge cost,
helps no other car). Generalizing in the shared layer improves every ICBM car (any `pcmCruise` +
`pcmCruiseSpeed=False` car) and is what makes upstream maintainers accept it.

---

## 6. Safety & "what users expect"

- `driver_setpoint` single-writer → "something moved my set speed" is impossible by construction.
- Limiters only slow; the driver overrides instantly (a manual press sets the new baseline).
- Curves slow you, then restore to **exactly** what you dialed — no residual.
- Speed limits lower you predictably; buttons respond crisply; no flapping.
- op-long stays gated behind `ALLOW_DEBUG`; conservative per-car defaults mean no car regresses.

---

## 7. Release / upstream plan (PR slicing)

Ship as one release on `mazda-dev`, but author it as the sequence below so each piece is independently
upstreamable and changes no other car's behavior:

1. **Setpoint single-writer + injection-accounting attribution + press-sent feedback plumbing** —
   robust replacement for the re-sync heuristic (subsumes F3). Deletes
   `update_speed_limit_assist_v_cruise_non_pcm` (SLA writing `v_cruise`) and `update_dash_sync`.
2. **Actuator abstraction + `ButtonActuator` servo + `ICBMActuationProfile`** — fixes F2 and F4; safe
   defaults preserve every car. Drops the F4 ramp; steady 5 Hz taps + grid/exact modes + asymmetric
   restore patience + decel-overshoot composition.
3. **SLA non-pcm guard fix** — press attribution + mid-move abort/restore (fixes F1, keeps `+/-`
   confirm as sunnypilot intended; settled presses behave stock and re-anchor the setpoint).
   pcm-op-long path untouched.
4. **Mazda profile + long-press actuation with tap-fallback** — the car-specific piece. No RES work.
5. **Replay-test harness** with the seg16 (confirm) and seg32 (ratchet) logs as regression fixtures.
   Replay validates pacing/attribution/state machines only — ECU reactions (injected hold, drop rate)
   are validated by the owner's on-car release test.

Consolidating the logic into the shared servo also *reduces* the fork's merge surface vs today's
scattered edits across four shared files.

---

## 8. Open items (validated during the owner's on-car release test)

- **Injected long-press feasibility** — the #1 unknown. F5 characterized physical holds; forged `=1`
  frames interleave with the wheel's genuine `=0` frames at 50 Hz and may register as taps. The
  tap-fallback (§4.4) makes this safe to ship; the on-car test decides whether the fast path is real.
- **F5 down-hold behavior** (n=3) — low confidence; owner tests `SET-` holds safely on-car. Fallback
  covers the decel path too.
- **F4 counter-offset scheme** — decided: drop the ramp, steady 5 Hz taps. (Revisit the `[1,1,0,None]`
  scheme only if the injected hold works and needs tighter counter management.)
- **Metric units** — the long-press grid/steps were measured in mph (imperial). Confirm the km/h grid
  (likely 5 km/h) for metric users; the profile must carry both. Until confirmed, metric gets the
  taps-only safe default.
- **op-long convergence** — keep the actuator seam clean so `mazda-dev-long` plugs in as a backend.

---

## 9. Key code references (HEAD)

- SLA state machine + confirm bug: `openpilot/sunnypilot/selfdrive/controls/lib/speed_limit/speed_limit_assist.py`
  (`update_state_machine_non_pcm_long`, `_update_non_pcm_long_confirmed_state`, the
  `v_cruise_cluster_changed → inactive` guard).
- Planner arbitration + SLA input: `openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py`
  (`:48` `v_cruise_cluster = CS.vCruiseCluster`; `:62` `sla.update`; `:69` `min` arbitration).
- ICBM controller (deadband/ratchet, decel-overshoot): `openpilot/sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py`
  (`REACT_DEADBAND` `:28`, `update_state_machine`, `DECEL_OVERSHOOT_*`).
- Set-speed ownership + re-sync: `openpilot/sunnypilot/selfdrive/car/cruise_ext.py`
  (`update_dash_sync` `:131-153`, `update_speed_limit_assist_v_cruise_non_pcm` `:177`).
- Mazda button emission (pacing/counter): `opendbc_repo/opendbc/sunnypilot/car/mazda/icbm.py`
  (`PACE_NORMAL/PACE_RAMP`, `[1,1,0,None]` counter offset).
- Mazda button parsing (no RES today): `opendbc_repo/opendbc/car/mazda/carstate.py:126-141`.
- Mazda CAN: `CRZ_BTNS`=157, `CRZ_EVENTS`=543 (`CRZ_SPEED`), `CRZ_INFO`=539 (`ACCEL_CMD`),
  DBC `opendbc_repo/opendbc/dbc/mazda_2017.dbc`.
- Prior desync fix: commit `a3c625fb16`; ramp pacing: `057cd7d70b`.

---

## Reproducing the analysis

Scripts in `tools/mazda_long/icbm_sla/` (run from repo root, venv active):

- `sla_triage.py <rlog…>` — per-log summary: branch, SLA states reached, speed-limit validity, cruise %,
  speed range, button events. Use to find logs that actually exercise SLA.
- `sla_icbm_timeline.py` — compressed transition timeline (SLA state, dash set, `vTarget`, ICBM state,
  driver buttons). Edit `P=` to the log. Produced F1.
- `curve_stop_timeline.py` — connect-route timeline through a curve→stop→resume (adds SCC-vision state,
  standstill). Produced F2.
- `button_press_efficiency.py` / `button_press_bursts.py` — injected `SET_P` frames vs `CRZ_SPEED`
  delta: mph/press, cadence, effective rate. Produced F4.
- `longpress_scan.py <rlog…>` — detect driver holds and their net speed change.
- `longpress_detail.py <rlog> <t0> <t1>` — sample-by-sample `CRZ_SPEED` + button edges for one hold.
- `longpress_corpus.py` — parallel scan of all `test_data` segments → `longpress_corpus_results.json`
  (needs `SPX` env = output dir; has `__main__` guard for macOS spawn). Produced F5.

Corpus data: `tools/mazda_long/test_data/` (674 CX-5 2022 segments, gitignored). Connect auth:
`~/.comma/auth.json`; `LogReader("952c07dea500f4e2/000000c9--0850e1c117/32")`.

---

## 7. Cruise arbiter refactor (2026-07-26) — intents + published session

Status: IMPLEMENTED on mazda-dev (`9da6aad967` schema, `96e2e995c6` arbiter, `80f14eebd5`
servo freeze), replacing the §4 guard-based design's *mechanics* while keeping its locked
semantics. Motivation: every first-drive bug (see `tools/mazda_long/icbm_sla/
replay_drive_incidents.py`) traced to three structural weaknesses:

1. Four modules independently interpreted the same +/- press (increment path, ownership
   latch, confirm/dismiss latches in plannerd, servo pause), bridged by 0.5 s wall-clock
   latches sized to the 20 Hz plannerd machine two processes from the buttons.
2. Intent traveled as a single number through the planner `min()`: SLA could not say
   "raise the setpoint" (up-confirm inert) or "hold, decision pending" (the preActive
   plan-target fake), and the servo's effective command was invisible.
3. The session existed nowhere as a datum: it was smeared across sla_state (plannerd),
   v_cruise (card), icbm_state (selfdrived), and the reconcile window.

As-built:

- **`cruise_arbiter.py` (card, 100 Hz)** — single owner of button meaning and the SLA
  session, in the same frame as the buttons and the setpoint writer. Classification at
  press edges: dismiss (press on an active session, owned), confirm/decline (release of
  a press that started AND released during a prompt; long presses resolve at the first
  repeat tick), increment/decrement otherwise. Timers are DT_CTRL frame counts; no
  `time.monotonic()` anywhere, so log replays are deterministic.
- **`carStateSP.cruiseSession`** — state, vCap (active target / frozen prompt hold),
  lastIntent, announceCounter. The counter makes 100 Hz alert-worthy transitions
  visible to 20 Hz consumers: the plannerd mirror (`assist_mirror.py`) republishes the
  old `speedLimit.assist` wire format (UI untouched) and fires `speedLimitActive` on
  announce deltas, `speedLimitPreActive` on level.
- **Prompt freeze, three layers**: the session's vCap holds the old target through the
  prompt (plan min unchanged), the servo parks (`prompt_frozen`, restore patience held
  at zero so decline/timeout waits a FULL quiet window), and card vetoes
  `sendButton` with same-frame state before `CI.apply` (the servo's own gate is one
  message hop stale). Found while testing: the servo's preActive entry path bypassed
  the quiet window entirely after any driver press (the t≈393 dash-slam) — gated now.
- **pcm-op-long unchanged**: `speed_limit_assist.py` keeps only that machine.

Deliberate deviation from §4.5's locked "min(setpoint, limit)": an upward confirm ADOPTS
the limit into the setpoint (`max(v_cruise, target)`, never lowering it) — owner decision
2026-07-26 after the first drive showed min() semantics make `+` do nothing when the
setpoint is at or below the old limit.

Validation: 45 loop-harness scenarios (transport delays and the dash-vs-cluster split
retained), arbiter unit tests, seg16 F1 replay, and the drive-0b incident replay all
green. Future upstream note: the reconcile settle-window mechanics still key on raw
press timers; expressing them fully in intents is cosmetic and left for the upstream PR.
