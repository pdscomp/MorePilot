# What the stock Mazda LKAS camera actually transmits

CX-5 2022 (plus one CX-9 cross-check). Measured from our own captured drives, not
from the DBC. Tooling:

- `tools/mazda_long/cam_bus_census.py` — provenance: which addresses are the camera's
- `tools/mazda_long/cam_payload_decode.py` — per-message bit/signal characterisation
- `tools/mazda_long/cam_distance_validate.py` — 0x244 vs radar, 0x21d payload dump
- `tools/mazda_long/cam_distance_slots.py` — 0x244 slot-structure test

## Reading the buses correctly (two traps)

**The harness relay starts open.** For the first ~8.5 s of every log bus 2 is shorted
to bus 0, so bus 2 shows the *entire car* — 104 addresses. Once the relay closes
(first `src == 130` echo) bus 2 carries only the camera, 11 addresses. Every number
below is gated on relay-closed. Ungated, the census is garbage.

**Some routes were logged relay-open the whole way.** `tja_cts_route_28/29/2a` have
`src 0` and `src 2` frame counts *exactly* equal and zero `src 130` echoes: the panda
never took the bus. Those are useless for provenance but are the *best* content
source, because they are genuinely stock drives with openpilot never in the loop.
`drive_1e`, `drive_25`, `route_27`, `route_53` are properly intercepted.

src encoding: 0/2 = received on that bus, 128/130 = panda TX echo onto that bus.
So the camera's own output is exactly `src == 2` after the relay closes.

## The camera sends 11 addresses, and only 11

Identical set across `drive_1e`, `route_53`, and `route_27`; zero `src 130` on any of
them (nothing the panda forwards *to* the camera), and the panda forwards all 11 on to
the car. It is exactly the DBC's `CAM_*` set — no unnamed camera frames exist.

| addr | DBC name | rate | period | what the payload does |
|---|---|---|---|---|
| 0x21d | CAM_EMPTY | 50 Hz | 20 ms | status + a mirror of the speed-sign word (below) |
| 0x242 | CAM_LANETRACK | 16.6 Hz | 60 ms | lane geometry, counter + checksum, mostly live |
| 0x243 | CAM_LKAS | 16.6 Hz | 60 ms | the steering command, counter + checksum |
| 0x244 | CAM_DISTANCE | 16.6 Hz | 60 ms | two 4-byte records, idle `64 f0` (below) |
| 0x245 | CAM_IDK3 | 16.6 Hz | 60 ms | all 8 bytes live, no counter/checksum, unidentified |
| 0x246 | CAM_LANEMAYBE | 50 Hz | 20 ms | lane geometry at 3x the rate of 0x242 |
| 0x25d | CAM_PEDESTRIAN | 50 Hz | 20 ms | near-static; only a counter moves |
| 0x35f | CAM_TRAFFIC_SIGNS | 10 Hz | 100 ms | speed sign, FCW, 10 distinct payloads total |
| 0x440 | CAM_LANEINFO | 2 Hz | 500 ms | LKAS/TJA state + HUD warnings |
| 0x485 | CAM_SETTINGS | 10 Hz | 100 ms | **frozen**, 1 payload for a whole drive |
| 0x488 | CAM_Empty3 | 10 Hz | 100 ms | **frozen**, 1 payload for a whole drive |

Rates must be measured per segment. A non-contiguous glob (`seg0, seg1, seg10`)
inflates the span and deflates every rate ~3.5x.

## The stock camera steers at 16.6 Hz; we transmit at 100 Hz

`create_steering_control` is appended every controls frame in
`opendbc/car/mazda/carcontroller.py`, i.e. 100 Hz, and the panda TX echo of 0x243
measures exactly 100.0 Hz. The real camera sends 0x243 every 60 ms — 15.7 Hz
measured, dt median 60.3 ms, p95 70.5 ms. We are 6x the stock cadence.

`CAM_LANEINFO` is the one we match: `frame % 50` = 2 Hz against the camera's 2.0 Hz.

## 0x243 CAM_LKAS — what stock actually commands

Two stock CTS drives, LKAS_REQUEST (`3|12@0+`, offset −2048):

| route | nonzero | range | p99 abs | max abs |
|---|---|---|---|---|
| tja_cts_route_29 | 45.9% | −660 .. +567 | 432 | 660 |
| tja_cts_route_28 | 13.9% | −764 .. +996 | 700 | 996 |

Peak by speed on route 28 reaches 996 at 0–5 m/s and 710 at 15–20 m/s. This does
**not** contradict the 620 EPS ceiling in `mazda-tja-cts-torque`: that ceiling is on
the *effective* torque the EPS applies, and the camera clearly *requests* well past
it. Request and effective are different measurements.

Slew is bounded: max single-frame step 120–121 counts per 60 ms frame on both routes,
p99 79–94, median 0.

Signals that never move in any stock drive, so they are not stock features:
`ERR_BIT_1`, `ERR_BIT_2`, `LDW`, `ANGLE_ENABLED`, `STEERING_ANGLE` (constant 0),
`BIT_1` (constant 1). 39 of 64 bits are static. `STEERING_ANGLE` being permanently
zero is worth noting — the camera does not echo angle back on this platform.

## 0x21d CAM_EMPTY is not empty — it mirrors the speed-sign word

The DBC maps one byte (`STATUS`) and leaves 32 active bits unmapped. Measured:

**Bytes 3–4 are a verbatim copy of 0x35f bytes 0–1** (the `SPEED_SIGN` /
`SPEED_SIGN_ON` word) — 22662/22678 frames, 99.93%, against the most recent 0x35f.
On route 28, where no speed signs were read, both are `00 00` all drive.

Bytes 0–2 are a two-state mux, and bytes 6–7 only carry data in the active state:

| byte0 | byte1 | byte2 | bytes 6–7 | share |
|---|---|---|---|---|
| 0x7f | 0x3f | 0xff | `ff ff` | 92% — idle / no data |
| 0x50–0x54 | 0x10–0x14 | 0x43–0x53 | `ce 73`, `d2 94`, `d6 b5` | 8% — active |

byte0 = byte1 + 0x40, and byte2 = 4·byte1 + 3 exactly across all observed values, so
bytes 1–2 are one scaled field, not two. Byte 5 (`0x2a`/`0x6a`) also tracks 0x35f's
byte 1. This is the message `carstate.py` already reads for `stockFcw`.

## 0x244 CAM_DISTANCE is not a distance

Structure is clear: **four (byteA, byteB) slots that idle at (100, 240) = `64 f0`**,
so a fully idle payload is `64 f0 64 f0 64 f0 64 f0`. Slots 0/1 activate together and
slots 2/3 activate together, so it is really two 4-byte records.

The DBC's `DISTANCE` label does not survive contact with the data:

- On `tja_cts_route_28` the message sat at the idle pattern for **100.0%** of 7537
  frames while the radar reported a lead **89.5%** of the time. A lead-distance field
  that never moves during a drive spent almost entirely behind a lead is not a
  lead-distance field.
- On `tja_cts_route_29` slots are active only 15.4% of the time vs 88.6% radar lead
  presence; agreement 21.7%.
- byteB does rise with radar `dRel` (corr +0.79) but wildly non-linearly — raw 0–19 →
  35.4 m mean (sd 14.5), raw 20–39 → 81.9 m, raw 40–59 → 90.6 m. Residual sd 17.8 m
  against a linear fit. That is a confound, not a calibration.
- Activity instead tracks the camera's own lane-line count: mean active slots is 0.00
  when `CAM_LANEINFO.LANE_LINES` is 1 or 4, and 0.94 when it is 2.

So: two 4-byte records populated only in a specific perception state, contents
unidentified. The `S1..S6`/`DISTANCE` byte split in the DBC is wrong regardless,
because it cuts across the 2x4 grouping the idle pattern reveals.

## 0x485 / 0x488 are constants

`CAM_SETTINGS` = `00 84 07 12 00 00 00 00` and `CAM_Empty3` =
`00 00 80 00 00 00 00 00`, one distinct payload each across a whole drive, 64/64 bits
static. The DBC's 11 `CAM_SETTINGS` signals (LKAS sensitivity, LDW alert, SBS
distance) are real menu settings but they only change when the driver changes them in
the HUD, which nobody did on these routes. Byte 3 differs between routes (`0x12` on
29, `0x11` on 27), consistent with a settings snapshot.

## Other quick facts

- **0x25d CAM_PEDESTRIAN** is inert: 52/64 bits static, `PED_WARNING`,
  `BRAKE_WARNING`, `PED_BRAKE` and `AEB_NOT_ENGAGED` constant. Only `CTR`/`S1`
  counters move. No pedestrian event in our corpus.
- **0x246 CAM_LANEMAYBE** carries the highest-rate lane data (50 Hz vs 0x242's
  16.6 Hz), byte 2 is checksum-like (256 distinct flat-distributed values), and byte 0
  has 6 fast-toggling bits (1800–6400 transitions) that no DBC signal covers.
- **0x242 CAM_LANETRACK** has a clean 4-bit counter in byte0 bits 4–7 (+1 mod 16 on
  100.0% of steps) and a byte-7 checksum. Same counter in 0x243.
- **0x35f CAM_TRAFFIC_SIGNS** emitted only 10 distinct payloads over a whole drive.
  `SPEED_SIGN` took values {0, 20, 50, 80}. `STOP_SIGN` never fired.
- **0x440 CAM_LANEINFO** at 2.0 Hz confirms the 0.563 s worst-case period already
  encoded in `CarControllerParams.CAM_LANEINFO_PERIOD_T`. `TJA` took values 0/3/4/5;
  all four HUD warning bits (`HANDS_ON_STEER_WARN`, `HANDS_ON_STEER_WARN_2`,
  `LDW_WARN_LL`, `LDW_WARN_RL`) stayed 0 all drive.

## What openpilot currently reads

`opendbc/car/mazda/carstate.py` subscribes to 5 of the 11: `CAM_LANEINFO`,
`CAM_TRAFFIC_SIGNS`, `CAM_EMPTY`, `CAM_LKAS`, `CAM_PEDESTRIAN`. It transmits 2:
`CAM_LKAS` and `CAM_LANEINFO`. The other 6 (`CAM_LANETRACK`, `CAM_DISTANCE`,
`CAM_IDK3`, `CAM_LANEMAYBE`, `CAM_SETTINGS`, `CAM_Empty3`) are forwarded through
untouched and never parsed. Of those, `CAM_LANEMAYBE` at 50 Hz and `CAM_LANETRACK` at
16.6 Hz are the only ones carrying meaningful live signal.

## Implications of our 6x LKAS cadence (measured 2026-08-27)

`tools/mazda_long/eps_slew_stock_vs_op.py`. Stock route 29 (relay open, camera
commanding at 16.6 Hz) against route 53 (openpilot commanding at 101.0 Hz),
`LKAS_EFFECTIVE` and `LKAS_REQUEST` both read out of 0x241 `STEER_RATE` on bus 0.

### The EPS rate limit is per unit time, not per frame

This corrects the framing in `docs/eps-rate-summary.md`, which concluded "12 units per
frame" from data captured entirely at our own 100 Hz. At one cadence the two readings
are indistinguishable. A stock drive separates them, and the answer is unambiguous:

| | stock cmd @16.6 Hz | openpilot cmd @101 Hz |
|---|---|---|
| `\|Δeff\|` per 10 ms sample | max 12, p99 12, steps {0,4,8,12} | max 20, p99 12, same steps |
| slew p99 | 1245 units/s | 1234 units/s |
| max `\|Δeff\|` over a 60 ms window | p99 68 | p99 68 |

The EPS ramps at ~1200 units/s internally and keeps ramping between commands — it does
not gate on frame arrivals. Stock commands a 120-unit step once per 60 ms and the EPS
takes ~100 ms to walk there. **So the 6x cadence buys no extra steering authority.**
The numbers are identical to within noise.

### But the cadence is load-bearing for the current tune

`STEER_DELTA_UP`/`DOWN` are per-frame and `STEER_STEP = 1`, so the commanded slew
ceiling is `DELTA * 100`:

- `DELTA_UP = 12` → 1200 units/s, which matches the EPS limit exactly. That match is a
  coincidence of two independently chosen numbers, not a derivation.
- `DELTA_DOWN = 25` → 2500 units/s on release, about 2x what the EPS can deliver.

The trap: "fixing" the cadence to stock's 16.6 Hz without rescaling `DELTA_UP` would
drop commanded slew to 200 units/s, 6x slower than the hardware allows, and lateral
response would collapse. `STEER_STEP` and `STEER_DELTA_*` have to move together.

### Tracking is tighter at the median and much worse in the tail

`|LKAS_REQUEST - LKAS_EFFECTIVE|` inside 0x241, frames with `|req| > 20`:

| | p50 | p90 | p99 | max | >100 units |
|---|---|---|---|---|---|
| stock | 27 | 87 | 183 | 262 | 7.0% |
| openpilot | 18 | 177 | 808 | 1022 | 14.6% |

The better median is real. The tail is **not** a cadence effect — it follows from
`STEER_MAX = 1200` against an EPS that clips near 620, plus much larger jumps
(our `|Δcmd|` max 353 vs stock's 120). At p99 we are asking for ~800 units the EPS
never delivers. Request and effective diverge by design here.

### Everything else is minor

- **Bus load:** 83.4 extra 8-byte frames/s. At 500 kbps a standard 8-byte frame with
  stuffing is ~120 bits ≈ 0.24 ms, so ~20 ms/s ≈ 2% extra. Non-issue.
- **Counter:** `ctr = frame % 16` at 100 Hz rolls every 160 ms vs stock's 960 ms.
  Nothing on the car validates it — it works — but it does make our traffic trivially
  distinguishable from stock.
- **No double-command risk:** `MAZDA_LKAS` carries `check_relay = true` in
  `opendbc/safety/modes/mazda.h`, and the census confirms only our 100 Hz stream
  reaches bus 0; the camera's own 16.6 Hz 0x243 is relay-blocked.
- **Untested:** whether the EPS would fault if we dropped *to* 16.6 Hz. Stock proves
  the EPS is happy at that rate from the camera, but our engage/disengage cadence step
  from 16.6 Hz to 100 Hz has no stock analogue.

## The EPS ceiling curve, corpus-wide (2026-08-27)

`tools/mazda_long/eps_ceiling_curve.py` (collection + hysteresis splits) and
`eps_ceiling_lookup.py` (lookup fit). 4798 segments, **11,408,748 clean frames**
(not `LKAS_BLOCK`, not `steeringPressed`, vEgo > 2), 1,181,665 of them with the EPS
clipping. Both signals come out of 0x241 `STEER_RATE`, which the EPS itself
transmits, so the measurement is independent of who commanded and of the relay.

Ceiling scored only on frames genuinely pushed (`|req| >= 700` and clipping):

| mph | nPushed | max \|eff\| | mph | nPushed | max \|eff\| |
|---|---|---|---|---|---|
| 4–18 | 165k | **1148** | 28–30 | 8.7k | 808 |
| 18–20 | 19.7k | 1132 | 30–32 | 11.4k | 676 |
| 20–22 | 9.0k | 1092 | 32–34 | 9.3k | **620** |
| 22–24 | 6.6k | 1048 | 34–52 | 60k | **620** |
| 24–26 | 9.9k | 1012 | 52–70 | 7.7k | **620** |
| 26–28 | 9.5k | 920 | | | |

**Above 32.5 mph: 7,490,617 clean frames, max `|eff|` = 620, frames above 620 = 0
(0.0000%).** Below 18 mph: 1,518,542 frames, max 1148, none above. Both rails are
absolute, not statistical.

### It is a function of instantaneous speed — verified

A lookup indexed on speed is only valid if the ceiling has no memory, so the rail was
re-scored split by longitudinal acceleration:

- 32–60 mph: decel, steady and accel rails are **all exactly 620, spread 0** across
  every 2 mph bin. No hysteresis crossing the threshold in either direction.
- Below 32 mph: spread ≤ 40 counts, i.e. a few 4-unit quantization steps.
- Left vs right: deltas ≤ 36 counts, symmetric.

The `<-- HYSTERESIS` flags above 60 mph in the raw output are a detector artifact:
torque demand there is low, so a modal-`|eff|` estimator lands on a cruising torque
rather than a rail. Conditioning on `|req| >= 700` removes them and every bin returns
620.

### Proposed lookup

Fit against the measured ceiling, `max err` in counts (4 counts = 1 quantization step):

| candidate | max err | mean \|err\| | mean over-command |
|---|---|---|---|
| current `([0,14.2,14.5] -> [1200,1200,800])` | 524 | 158.9 | **159** |
| 2-point `([0,8.0,14.5] -> [1144,1144,620])` | 124 | 23.8 | 0 |
| **3-point `(+11.2 -> 992)`** | **26** | **6.8** | 1 |
| 4-point `(+10.7/12.5)` | 48 | 9.7 | 1 |

Snapped to the measured values:

```python
self.STEER_MAX = 1148
self.STEER_MAX_LOOKUP = ([0., 8.0, 11.2, 14.5], [1148, 1148, 1012, 620])
```

This costs no steering authority. Above 32.5 mph the EPS delivered nothing above 620
in 7.5M frames, so commanding 621–800 puts identical torque at the wheel; below
18 mph it delivered nothing above 1148 against requests reaching 1500. What changes is
that `pid.set_limits()` clamps the integrator to a reachable range and
`pid_log.saturated` becomes truthful, which also restores the driver-facing saturation
alert during the episodes it exists for. `STEER_DELTA_UP`/`DOWN` are untouched, so ramp
rate to the rail is unchanged.

The `# 22% PID headroom above EPS ceiling (620)` comment in `values.py` is exactly the
blind spot: it set 800 above a rail already known to be 620, and the corpus now shows
that rail is absolute.

Caveat: this is a steering-behaviour change in opendbc and should ride the V2 tune's
pending on-car pass, not a separate validation drive. Dependence on steering angle or
EPS temperature was not separately tested; speed, acceleration sign and direction were.
