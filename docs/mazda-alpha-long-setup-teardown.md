# Mazda alpha-long setup/teardown: fault evidence and redesign research

Status: IMPLEMENTED 2026-07-29 (opendbc `0f0dad66ac` + main repo follow-up) — pending on-car
validation. §6 experiment run 2026-07-28 (route 25) resolved the gate question, see §4.5.
Log evidence from drives 2026-07-26/27/28, pulled to
`tools/mazda_long/test_data/alpha_long_logs/` (gitignored).

As-built notes (deviations from §4):
- Gate bits are NO_ERR_BIT + BIT2 only. LINE_VISIBLE is real lane-line state while
  driving (set for long stretches in routes 23/27/28), so it is excluded; a latched
  fault (ERR_BIT) also holds the gate closed, which keeps the teardown off an
  already-faulted car and preserves its AEB.
- accFaulted semantics unchanged (silent < 1 s): the "Cruise Fault" alert now covers the
  longer pre-teardown phase too; replacing it with a friendlier handover alert is the
  remaining task (#5).
- The teardown also waits for any active stock MRCC engagement to end (driver SET before
  the gate passes on a warm boot), so the radar is never silenced mid-stock-control;
  openpilot independently cancels stock cruise while accFaulted, so this window is brief.
- Hand-back detection reuses accFaulted as "stock radar heard": card asserts
  CarControlSP.stockEcuHandBack, waits for accFaulted (or 8 s, past the S3 fallback),
  then writes OnroadCycleRequested itself. The UIs now write only the param.
- Session state machine: STOCK -> SILENCING (10 02 at 2 Hz) -> SILENCED (tester present +
  synthetics) -> HANDBACK (10 01 at 2 Hz, synthetics keep flowing) -> back to STOCK
  (quiet). SILENCED -> SILENCING on S3 recovery (auto re-silence).

## 1. Symptoms

1. Cold start with `AlphaLongitudinalEnabled=1`: LKAS malfunction + smart-cruise malfunction
   latch shortly after drive-off. The latch survives ignition cycles (with either toggle
   state) and clears only after the car sits long enough for module power-down (~13 min
   observed).
2. Disabling alpha long while driving: same latch, immediately.
3. Empirical workaround discovered by the owner: start the car stock, key off, enable alpha
   long, restart within a minute or two ("warm start") — works every time observed.

## 2. Evidence

Every op-long transition ever logged:

| Event | Conditions | Result |
|---|---|---|
| `0000000a` enable (7/26 16:07) | warm — 20 s after `09` blip; silenced t≈6; moving t≈19 | clean, 5 min, engaged |
| `0a`→`0b` disable (7/26) | key-off with radar muted; `0b` boots 4 min later | clean (radar S3-recovers while car off) |
| `0000001a` enable (7/27 01:05) | cold — 55 min parked; silenced t=7.8 | FSC faults at first motion t=30.3 |
| `1b,1c,1f` | restarts after latch | pre-faulted from boot |
| `1e,20,21` | alpha OFF, stock safety, after latch | still faulted → latch is in the car |
| `00000023` enable (01:27) | warm — 27 s after `22` | clean, 7 min, engaged |
| `23`→`24` disable (01:34) | onroad toggle → mid-drive op restart | fault latched in restart gap |
| recovery | ~13 min parked before `22` | clean |
| `00000025` enable (7/28 19:31) | COLD — ~18 h parked; silenced t=10.5 | CLEAN — engaged t=43, no fault |

Key CAN facts:

- Fault surface is the FSC camera itself. Captured flip in `1a` at t=30.29:
  `CAM_LANEINFO 4201000000001040 → 4300000000011040` (`LANE_LINES 1→0`, `ERR_BIT` set) with
  `CAM_LKAS ERR_BIT_1` following at 30.32. Nothing else on the bus changed at that moment.
- The radar-owned periodic set (CRZ_INFO 0x21B, CRZ_CTRL 0x21C, tracks 0x361-6, 0x499) is
  exactly what the port synthesizes — nothing is missing. Radar boot status is identical
  cold vs warm (one 0x03 CRZ_INFO frame, 0x01 within 200 ms).
- Mid-drive disable (`23`→`24`): tester present + synthetic frames die with the op restart;
  radar returns only via UDS S3 timeout (first stock CRZ_INFO at `24` t=3.83) → multi-second
  radar blackout at ~27 mph; camera already faulted at `24` frame 0.
- Pre-existing wart: at every op-long boot there is a ~0.5–1 s two-master overlap (our
  synthetic CRZ frames start before the stock radar goes silent — `1a` sec 7).
- Logging gotcha: with the harness relay closed (pre-init) bus 0/2 physically mirror; after
  the relay opens, panda-forwarded frames are not logged. Absence of an ID on the other bus
  in rlogs is a logging artifact, not starvation.

Interpretation (RESOLVED by route 25, 2026-07-28): the FSC runs its radar-presence check in
the few seconds after its own boot sequence completes — NOT at first motion and NOT on a
fixed wake timer. The check verdict merely *displays* at first motion (route `1a` fault
broadcast at t=30.3 was decided by ~t=10). The FSC's boot completion is broadcast on
CAM_LANEINFO (§4.5). Route 25 was a cold boot that stayed clean purely because openpilot's
init happened to fire UDS 7.7 s after the FSC settled instead of 1.9 s (route `1a`).

## 3. Why the toggle latches mid-drive (mechanism)

`AlphaLongitudinalEnabled` is read exactly once, at fingerprint (`card.py:101`). The UI
toggle (allowed onroad when not engaged, both layouts) therefore writes the param and
`OnroadCycleRequested` (`developer.py:191-211`, mici via `common.py:5`); `hardwared` drops
`started` for 1 s (`hardwared.py:36,201-205`) and manager restarts all onroad processes.
Upstream carries a TODO acknowledging our exact failure (`developer.py:123`: "alpha long
toggle requires a deinit function to re-enable radar and not fault"). `deinit()` exists for
five brands and is dead code everywhere (stale signatures; zero call sites).

Critical timing fact: pandad is `always_run` and survives the cycle; it forces
`NO_OUTPUT` (TX blocked, relay open→closed passthrough) within ~100 ms of `started=False`
(`pandad.cc:216-220`) — before card's only shutdown hook (the `card_thread` finally,
`card.py:320-330`, reached via SIGINT/KeyboardInterrupt) could run a UDS exchange. pandad's
`can_send_thread` also drops sendcan older than 1 s. Conclusion: **no shutdown-time deinit
can work; the radar hand-back must complete before the cycle is requested.**

## 4. Redesign

### 4.1 Deferred teardown (fixes cold start)

Move `enter_radar_programming_session` out of `CarInterface.init` (also removes the
"miss a few cycles" blocking-ISO-TP wart at `card.py:278-283`). Carcontroller emits the
session-entry frame fire-and-forget (`CanData(0x764, [02 10 02 …], 0)` at ~2 Hz while the
gate has passed and the stock radar is still heard). The panda tx hook already allows
session types 1 and 2 on 0x764 main bus (`mazda.h:211-218`; `test_mazda.py:174-180`).
No UDS response reading needed — verification is the established infer-from-bus pattern
(`carstate.py:109-116` stock CRZ_INFO silence counter).

- Gate condition (RESOLVED — FSC boot-settle signal, §4.5): CAM_LANEINFO
  `NO_ERR_BIT == 0 AND BIT2 == 0 AND ERR_BIT == 0` held continuously for ~10 s
  (LINE_VISIBLE excluded as-built: it is live lane-line state while driving).
  Computed in carstate, read by carcontroller off the CarState object; no side channel.
  Total boot delay ≈ settle (3–6 s) + 10 s margin ≈ 13–16 s — still well inside stock
  MRCC's own cold-boot arming time (24–42 s), so zero perceived cost.
- `accFaulted` semantics change: today it means "stock radar heard recently", which under
  deferral would display "Cruise Fault" (PERMANENT, `events.py:749-753`) throughout the
  intentional pre-teardown phase. New: `accFaulted = gate_passed and stock radar heard`.
- Synthetic CRZ/track TX and tester present gate on stock-radar-silent — closes the
  two-master overlap at boot.
- Cost: op-long engagement unlocks a few seconds after the gate (stock MRCC arming itself
  takes ~24 s on a cold boot; negligible in practice).
- Optional warm fast path (skip deferral if last onroad exit < ~1 min): requires a new
  PERSISTENT wall-clock param written on card exit — no existing param records ignition-off
  time (`UptimeOffroad` is a lifetime accumulator; hardwared's edge timestamps are
  monotonic locals).

### 4.2 Ordered hand-back (fixes onroad disable)

UI toggle-off writes only the param. card's 10 Hz `params_thread` (`card.py:309-318`) also
reads `AlphaLongitudinalEnabled`; on mismatch with `CP.openpilotLongitudinalControl`
(Mazda op-long only): force disengage → carcontroller stops synthetic frames + tester
present → emits default-session frame (`02 10 01`) at ~2 Hz → carstate detects the stock
radar returning (CRZ_INFO heard) → card writes `OnroadCycleRequested` itself (card holds
`Params`; sunnylinkd's remote blocklist is unaffected). Timeout ~3 s without radar return →
abort, resume synthetic frames, alert "will apply at next ignition". Enable direction
onroad needs no radar work — card requests the cycle immediately. Offroad, no cycle at all
(param is read at next fingerprint).

Unknown to validate on-car: radar wake latency after an explicit `10 01` (expected ≪ the
~5 s S3 path; first test at low speed).

### 4.3 Residual risk (documented limitation)

card crash mid-drive (`restart_if_crash` is False for card) or SIGKILL: tester present
stops, radar S3-revives into a moving car, FSC latches. Not fixable from card; would need
pandad-side help. Best-effort: card's finally attempts `request_radar_default_session` only
if pandaStates still shows a car safety mode (likely a no-op given §3 timing, but cheap).

### 4.4 Latched-fault driver guidance

SP-only event (custom.capnp `OnroadEventSP.EventName` next free `@24`; `EVENTS_SP` has no
completeness test). Emit from `CarSpecificEventsSP.update`, which can also remove the
upstream `steerUnavailable`/`invalidLkasSetting` events (precedent: removes
`belowSteerSpeed`). Text pattern: `relayMalfunction`-style two-line permanent alert —
"LKAS fault latched by car / Park for ~15 min to clear". Boot-latch discriminator in mazda
carstate (fault flags true within first N frames after cam parser valid; first CS.update is
post-fingerprint and pre-any-TX, so cleanly observable), plumbed via `CarStateSP`
(custom.capnp next field `@2`) from `carstate_ext.py`. Verify: SP-only NO_ENTRY may not
affect `ss.engageable` (computed from upstream Events) — may need to keep an upstream
NO_ENTRY event alongside.

### 4.5 The FSC boot-settle broadcast IS the gate signal (found via route 25)

The earlier bit-level search (4 cold vs 3 warm stock boots, settling window 8–60 s) returned
zero candidates — because the real signal settles at 2.8–6.0 s, below the 8 s filter floor.
Route 25 (2026-07-28, the §6 experiment) exposed it: a COLD boot with alpha long enabled that
did NOT fault, because openpilot's UDS happened to land 7.7 s after FSC settle.

CAM_LANEINFO (0x440) broadcasts an FSC boot-in-progress state: payload `4361 …` =
`LINE_VISIBLE=1, NO_ERR_BIT=1, BIT2=1` (bits 0, 14, 13), settling to `4201 …` (all three
clear) when the FSC finishes booting. Settle times observed: 2.8/2.9/3.0 s (routes 25/19/23)
or 5.8/5.9/6.0 s (15/16/1a/22) — bimodal, always < 6 s. On a warm boot where the FSC never
power-cycled (`0a`, 4 min gap) there is no transient at all — payload is settled from frame 0.

Gap between FSC settle and radar silencing decides the outcome:

| route | settle | UDS | gap | outcome |
|---|---|---|---|---|
| `1a` cold | 5.9 | 7.8 | **1.9 s** | latched |
| `23` warm | 3.0 | 8.8 | 5.8 s | clean |
| `0a` warm | ≤0.4 | 7.3 | ≥6.9 s | clean |
| `25` cold | 2.8 | 10.5 | **7.7 s** | clean |

True threshold is somewhere in (1.9, 5.8] s after settle; 10 s margin doubles the largest
observed-clean gap. The "warm start ritual" always worked because a warm FSC either skips
the transient entirely or completes its check before openpilot's ~7–10 s init-time UDS.
Cold boots were a coin flip on init timing (`1a` lost, `25` won). During the handover the
driver sees the transient "Cruise Fault" (accFaulted) on comma — expected and harmless;
under §4.1 its semantics change so it only shows during the intentional handover window.
Radar-side readiness was never the failing part (CRZ_INFO status 0x03→0x01 within ~200 ms;
first UDS always answered).

## 5. Precedent notes (for upstreaming)

- All five brands with ECU knockout (`disable_ecu`: Honda/Hyundai/Toyota/Subaru; session:
  Mazda) silence at `init` and maintain with tester present from the controller
  (Honda 10 Hz, Toyota 5 Hz, Hyundai/Subaru 1 Hz, Mazda 2 Hz). None re-enters a session
  from the control loop; none has a live deinit. This design would be the first ordered
  hand-back — directly answers upstream's `developer.py:123` TODO.
- Hyundai `enable_radar_tracks` (sunnypilot) is the in-tree precedent for init-time UDS
  whose success feeds back into CarParams (`radarUnavailable`).
- Test seam: `TestLongitudinalIntegration` (`test_mazda_controller.py:272`) builds a real
  CarController via `get_params(alpha_long=True)` and counts emitted addresses incl. tester
  present — extend with gate-transition emission sequences. Nothing tests `longitudinal.py`
  today.

## 6. Experiment (RUN 2026-07-28, route 25 — resolved §4.1 gate)

Cold boot (~18 h parked), alpha long enabled, current code. Result: NO fault — neither
while parked nor at drive-off (reverse at t=23, forward motion t=42, engaged t=43, clean
for the whole route). This falsified both the pure-motion and the pure-wake-timer
hypotheses and, combined with the `1a`/`23`/`0a` timing table, localized the FSC check to
the seconds immediately following its boot-settle broadcast (§4.5). Remaining on-car
validations: radar wake latency after explicit `10 01` (§4.2) and the 10 s margin itself
(first deferred-teardown drive should log settle→UDS gap and outcome).

## 7. Field failures after first implementation (2026-07-30 / 08-01)

**Routes 2b/2e (2026-07-30, fixed opendbc `dc5a9a6378`)**: card crashed on its first
`CS.update` — `cp_cam.vl_all["CAM_LANEINFO"]` KeyError. `CANParser.vl` lazily registers
messages on access; `vl_all` does not, so every message read through `vl_all` must be in
`get_can_parsers()`. Symptoms all cascaded from the dead card: canError ("Unknown Vehicle
Variant"), dead toggle monitor, and on the enable-while-driving attempt an FSC ERR_BIT
latch (relay open + no CAM_LKAS publisher). Regression: `test_mazda_carstate.py` runs the
real parsers through the interface both ways.

**Route 34 (2026-08-01, fixed main `ee7bc34d3e`)**: with the crash fixed, enabling onroad
still latched ERR_BIT at t=5.7 s — before the settle gate could act (no UDS all session).
Root cause: pandad races manager's onroad-transition param clearing across the
OnroadCycleRequested restart (the offroad gap is only ~1.7 s). pandad won, consumed stale
FirmwareQueryDone/ControlsReady/CarParams, applied the PREVIOUS session's safety (mazda
param 0, stock) during the gap and latched `safety_configured_`. Two consequences: the
harness relay opened ~8 s before controls came up (camera LKAS frames blocked from the
car with no replacement -> car LKAS fault + FSC ERR_BIT ~7 s into the blackout), and the
whole session ran under stock safety, which would also have blocked every op-long TX
(0x764 UDS, synthetic CRZ/radar frames are in MAZDA_LONG_TX_MSGS only). Normal boots are
immune because ELM327/NO_OUTPUT/SILENT keep the relay CLOSED and the car safety mode is
only applied at ControlsReady, when controls TX starts within the same second. Fix:
hardwared clears ControlsReady + FirmwareQueryDone when it consumes OnroadCycleRequested,
making the cycle sequence exactly like a normal boot. The race exists upstream too (any
OnroadCycleRequested user).

## 8. Force-offroad and the S3-recovery degraded state (2026-08-01 drive)

Routes 38-3b validated the whole enable flow end to end: fresh LONG safety applied
(post-race-fix), teardown at ~14 s, accFaulted cleared, op-long engaged and drove
(route 39). Then "Always Offroad" was toggled mid-drive: card died before any hand-back
(pandad NO_OUTPUT ~200 ms later), the radar sat silent until its ~5 s S3 timeout, and
came back MID-DRIVE in a degraded state - stock CRZ_CTRL alternating healthy
`0201010000000000` with fault `1a01010002000600`, body ECU cycling PEDALS.ACC_OFF every
1-25 s, CRZ_EVENTS chiming 64<->84. The cycling persisted across openpilot restarts and
two more teardown sessions (3a, 3b) and blocked every cruise SET; only an ignition cycle
clears it. Key contrast: an ordered `10 01` hand-back (route 39's teardown counterpart)
returns the radar cleanly, and a teardown from a healthy stock state engages fine - it
is specifically the unmanaged S3 recovery at speed that leaves the car cranky.

Fix (main `042865e84e`): "Always Offroad" writes OffroadModeRequested; the card monitor
runs the ordered hand-back and then grants OffroadMode (offroad wins over a pending
toggle cycle); hardwared grants directly when offroad already or after a 10 s timeout so
the request can never silently fail. Remaining unmanaged-blackout paths: openpilot crash
mid-drive and ignition-off (harmless - car powers down with it).
