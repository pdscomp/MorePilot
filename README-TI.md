# Torque Interceptor (TI1) Support — Full Protocol & Porting Spec

This branch (`ZoomPilot/more-konik`) adds Mazda GEN1 Torque Interceptor support to
ZoomPilot, layered on the Konik backend layer (`ZoomPilot/konik`).

The TI communication protocol was ported from the MoreTore fork's `mazda-frogpilot`
branch. Credit for the original reverse engineering goes there. This document is
written to be complete enough that an agent (human or AI) can reimplement TI1 support
from scratch on a fresh sunnypilot/frogpilot/starpilot tree using only this file.

---

## 1. Why this fork defaults to the Konik backend

comma.ai connect has been reported to block users running a Torque Interceptor, and
possibly even users running a branch that merely supports one. To protect users, this
fork:

- **Defaults to the Konik backend** (`UseKonikServer` defaults to `1`).
- **Locks the backend once TI has ever been enabled.** Enabling TI sets the
  `KonikInterlock` param, which prevents switching back to the comma.ai backend — even
  if you later disable TI or flash a branch that defaults to comma connect.
- **Blocks TI-related params from upload** (`TorqueInterceptorEnabled`,
  `TorqueInterceptorEnableRequest`, `KonikInterlock` are PERSISTENT|DONT_LOG and on
  the sunnylink block list).

The only way back to comma.ai after enabling TI is a **factory reset** (multi-tap the
screen during boot), which wipes locally stored driving data that could otherwise be
uploaded and reveal the TI. Do not bypass the interlock.

---

## 2. Hardware topology and bus map

The TI1 is an inline CAN device paired with a GEN1 Mazda (camera-based LKAS cars:
CX-5, CX-9, Mazda3, etc. — `MazdaFlags.GEN1`). It taps a second CAN bus and accepts
steering torque commands on a private side channel, bypassing the camera-path torque
limits and the stock EPS steer-lockout behavior.

| Bus | openpilot name | Role |
|-----|----------------|------|
| 0 | `Bus.pt` / `MAZDA_MAIN` | Car main bus: stock camera-path `CAM_LKAS` (0x243), `CRZ_BTNS`, `CAM_LANEINFO`, radar/long |
| 1 | `Bus.body` / `MAZDA_AUX` | **TI side channel**: commands `CAM_LKAS2` (0x249) out, feedback `TI_FEEDBACK` (0x24A) in |
| 2 | `Bus.cam` / `MAZDA_CAM` | Camera bus: forwarded stock `CAM_LKAS`/`CAM_LANEINFO` from the FSC |

GEN2/GEN3 Mazdas do not use this protocol (they steer via `EPS_LKAS` on bus 1 with
`STEER_FEEL=10000`). TI1 support must be gated to GEN1.

---

## 3. Command protocol: `CAM_LKAS2` (0x249 / 585)

Sent by openpilot **every 10 ms (100 Hz)** on bus 1, unconditionally, whenever TI mode
is active — including disengaged (torque 0). The TI needs the continuous stream.

### Wire format (8 bytes)

| Byte(s) | Field | Encoding |
|---------|-------|----------|
| 0 (low nibble), 1 | `LKAS_REQUEST` | 12-bit unsigned, big-endian: `raw = (data[0] & 0x0F) << 8 \| data[1]`; torque = `raw − 2048`. High nibble of byte 0 must be 0. |
| 2 (low nibble), 3 | `CHKSUM` | **Duplicate of LKAS_REQUEST**, same 12-bit encoding (not a real checksum). High nibble of byte 2 must be 0. |
| 4–7 | `KEY` | Magic constant, literal bytes `C4 61 CE 60`. |

Idle frame (torque 0): `08 00 08 00 C4 61 CE 60`.

### DBC definition

```
BO_ 585 CAM_LKAS2: 8 XXX
 SG_ LKAS_REQUEST : 3|12@0+ (1,-2048) [0|2048] "" XXX
 SG_ CHKSUM : 19|12@0+ (1,-2048) [0|2048] "" XXX
 SG_ KEY : 39|32@0+ (1,0) [3294744159|3294744161] "" XXX
```

(`KEY` = 3294744160 = 0xC461CE60; the DBC's big-endian 32-bit packing emits the literal
bytes above. Do not "fix" the DBC value to the wire bytes — the packer handles it.)

### Torque limits (must be enforced in software AND panda)

| Limit | Value (base / 2022+ EPS) | Notes |
|-------|-------|-------|
| `STEER_MAX` | 600 / 1200 ≤ ~32 mph, 800 above | symmetric ±; 2022+ EPS uses `STEER_MAX_LOOKUP` ([0, 14.2, 14.5] m/s → [1200, 1200, 800]), matching the stock tune — the EPS clips injected torque past its per-speed ceiling either way |
| `STEER_DELTA_UP` | 6 / 12 per 10 ms | wind-up; 12 = EPS hardware rate limit |
| `STEER_DELTA_DOWN` | 15 / 25 per 10 ms | fast release |
| `STEER_MAX_RT_DELTA` | 192 / 384 | real-time ramp ceiling |
| `STEER_RT_INTERVAL_NS` | 250 000 000 | rt reference window (250 ms) |
| `STEER_DRIVER_ALLOWANCE` | 15 | driver override headroom |
| `STEER_DRIVER_MULTIPLIER` | 40 | driver torque influence |
| `STEER_DRIVER_FACTOR` | 1 | |

The 2022+ EPS values are gated on `MazdaFlags.STEER_TO_ZERO` (EPS firmware identity), exactly like
the stock-path tune; older EPS stays on the conservative base envelope.

Distinct from the camera path (±800, up 10 / down 25). Panda enforces the same numbers
independently; openpilot must stay inside them or panda blocks the frame.

---

## 4. Feedback protocol: `TI_FEEDBACK` (0x24A / 586)

Sent by the TI at **50 Hz** on bus 1.

### Wire format (8 bytes)

| Byte | Field | Encoding |
|------|-------|----------|
| 0 | `TI_TORQUE_SENSOR` | unsigned − 127 → signed driver torque. Healthy firmware mirrors byte 1 exactly. |
| 1 | `CHKSUM` | same value as byte 0 |
| 2 | `VERSION_NUMBER` | firmware id. **Known-good: `0x01` and `0x10` (16, current firmware)** |
| 3 | `STATE` | 0=DISCOVER, 1=OFF, 2=DRIVER_OVER, 3=RUN |
| 4 | `VIOL` | violation flags; must be 0 in RUN |
| 5 | `ERROR` | must be 0 |
| 6 | `RAMP_DOWN` | must be 0 |
| 7 | spare | observed `0x30`, ignored |

Healthy idle frame: `7F 7F 10 03 00 00 00 30`.

**Boot sequence:** on power-up the TI walks STATE 1 → 2 → 3 over ~2–3 s (observed
~150 frames at 1, ~90 at 2). `VIOL=0x12` during the OFF state at boot is transient and
normal. Only STATE=3 with all fault bytes 0 is healthy.

**Torque mirror:** when torque is applied, byte 0/1 move together away from the 0x80
center (verified across ~11k full-rate frames; 100% mirror). A unit whose byte 0 stays
pinned while byte 1 moves is not relaying.

### DBC definition

```
BO_ 586 TI_FEEDBACK: 8 XXX
 SG_ TI_TORQUE_SENSOR : 7|8@0+ (1,-127) [-85|85] "" XXX
 SG_ CHKSUM : 15|8@0+ (1,-127) [-127|128] "" XXX
 SG_ VERSION_NUMBER : 23|8@0+ (1,0) [0|255] "" XXX
 SG_ STATE : 31|8@0+ (1,0) [0|3] "" XXX
 SG_ VIOL : 39|8@0+ (1,0) [0|255] "" XXX
 SG_ ERROR : 47|8@0+ (1,0) [0|255] "" XXX
 SG_ RAMP_DOWN : 55|8@0+ (1,0) [0|1] "" XXX
 SG_ SPARE : 63|8@0+ (1,0) [0|255] "" XXX
```

---

## 5. opendbc changes

All under `opendbc/car/mazda/` + `opendbc/safety/`.

### values.py

- `MazdaFlags.TORQUE_INTERCEPTOR = 4` (car flag, set at params time)
- `MazdaSafetyFlags.TORQUE_INTERCEPTOR = 2` (panda safetyParam bit)
- `TorqueInterceptorState(IntEnum)`: DISCOVER=0, OFF=1, DRIVER_OVER=2, RUN=3
- `TorqueInterceptorControllerParams`: the limits table from §3

### mazdacan.py

```python
def create_ti_steering_control(packer, apply_torque):
  return packer.make_can_msg("CAM_LKAS2", 1, {
    "LKAS_REQUEST": apply_torque,
    "CHKSUM": apply_torque,     # duplicate, not a checksum
    "KEY": 0xC461CE60,
  })
```

### carcontroller.py

Every frame, in addition to the normal camera-path `CAM_LKAS`:

- If `CP.flags & TORQUE_INTERCEPTOR`:
  - When `CC.latActive and CS.ti_lkas_allowed`:
    `ti_new_torque = round(CC.actuators.torque * ti_steer_max)` (speed-interpolated via
    `STEER_MAX_LOOKUP` on the 2022+ EPS, flat `STEER_MAX` otherwise),
    then `apply_driver_steer_torque_limits(...)` against `CS.out.steeringTorque`
    (which is the TI sensor in TI mode), then the rt delta clamp (192 or 384 per 250 ms window).
  - Otherwise torque 0 and reset the rt limiter state.
  - Append `create_ti_steering_control(packer, ti_apply_torque)` **every frame** —
    engaged or not. The stream must never stop while TI mode is on.

### carstate.py

- Add a third CANParser: `Bus.body`, messages `[("TI_FEEDBACK", 50)]`, bus 1 — only
  when the TI flag is set.
- In TI mode:
  - `ret.steeringTorque = TI_TORQUE_SENSOR` (replaces the EPS `STEER_TORQUE` source);
    `steeringPressed` threshold `abs > 6`.
  - `ti_lkas_allowed = can_valid and VERSION_NUMBER in (1, 16) and STATE == RUN
    and not any(VIOL, ERROR, RAMP_DOWN)`
  - `ret.steerFaultTemporary = not ti_lkas_allowed` (in place of the stock
    speed/fault logic; non-TI keeps the stock path)
  - Optionally mirror `ti_lkas_allowed` onto `CarStateSP.torqueInterceptorReady` for a
    specific onroad alert (see §7).

### Interface wiring (`opendbc/sunnypilot/car/interfaces.py` in this fork)

Gated on a `TorqueInterceptorEnabled` param read at params time:

```python
if CP.brand != 'mazda' or not params_dict.get("TorqueInterceptorEnabled", False):
  return
if not CP.flags & MazdaFlags.GEN1:
  raise ValueError("Torque Interceptor requires Mazda GEN1")
CP.flags |= MazdaFlags.TORQUE_INTERCEPTOR.value
CP.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.TORQUE_INTERCEPTOR.value
CP.dashcamOnly = False
CP.minSteerSpeed = 0
CP.steerAtStandstill = True
```

### dbc

Add the two message definitions from §3/§4 to `opendbc/dbc/mazda_2017.dbc`.

---

## 6. Panda safety changes (`opendbc/safety/modes/mazda.h`)

Constants: `MAZDA_TI_LKAS 0x249`, `MAZDA_TI_FEEDBACK 0x24A`, `MAZDA_AUX 1`,
`MAZDA_PARAM_TI 2`, feedback timeout 40 000 µs (two missed 50 Hz frames).

1. **Init**: `mazda_ti = GET_FLAG(param, MAZDA_PARAM_TI)`. When set, swap to TI rx/tx
   configs:
   - RX: stock array **plus** `{0x24A, bus 1, len 8, 50 Hz, ignore_checksum,
     ignore_counter, ignore_quality_flag = false}` — the quality flag is the health gate.
   - TX: stock list **plus** `{0x249, bus 1, 8}`.
2. **Quality flag** (the heart of the health gate):

```c
bool valid = (addr == 0x24A) && (bus == MAZDA_AUX) &&
             ((data[2] == 0x01) || (data[2] == 0x10)) &&  // version byte: both known firmwares
             (data[3] == 3) &&                             // STATE == RUN
             (data[4] == 0) && (data[5] == 0) && (data[6] == 0);
```

   Track `mazda_ti_feedback_healthy` + last-valid timestamp; feedback is "fresh" when
   healthy and ≤40 ms old.
3. **Driver torque sampling**: when `mazda_ti`, sample `torque_driver` from
   `0x24A` byte 0 − 127 (and **stop** sampling from `STEER_TORQUE` 0x240 — that path
   is `!mazda_ti` only).
4. **TX validation** for 0x249 on bus 1:
   - decode `raw = ((data[0] & 0x0F) << 8) | data[1]`, torque = raw − 2048
   - violation if: `!mazda_ti`, high nibble of byte 0 or 2 set, duplicate field ≠
     request field, or key bytes ≠ `C4 61 CE 60`
   - torque checks: if `(controls_allowed || controls_allowed_lateral) &&
     feedback_fresh` → enforce max 1200 (looser backstop; the controller self-caps
     per EPS: 600 flat on older EPS, 1200/800 speed-tapered on 2022+),
     driver-limit (allowance 15 / multiplier 40, rate 12/25), rt delta 384 over the
     standard `MAX_RT_INTERVAL`; else **any nonzero torque is a violation**
   - any violation → block the frame and reset TI torque state
5. **Forwarding**: block 0x249 relay between camera and main buses both directions
   (`fwd_hook`), so the side channel never leaks onto the car bus.
6. **Longitudinal combination**: if your fork also has Mazda alpha-long, the TI
   entries must be added to **both** config pairs — this branch carries four rx/tx
   array sets (`mazda_rx_checks`/`mazda_ti_rx_checks` and
   `mazda_long_rx_checks`/`mazda_long_ti_rx_checks` with their TX lists), selected
   as `long ? (ti ? long_ti : long) : (ti ? ti : stock)`.
7. Non-TI mode must be **untouched**: TI rx/tx arrays selected only when the param
   bit is set; the 0x249 tx block also catches stray frames in non-TI mode
   (fail-closed).

### Board-level: exposing bus 1 (the step everyone misses)

Stock panda never delivers bus-1 frames to the host. In `panda/board/main.c`,
`set_safety_mode` must switch the second CAN transceiver on for TI:

```c
case SAFETY_MAZDA:
  set_can_mode(GET_FLAG(param, MAZDA_PARAM_TI) ? CAN_MODE_OBD_CAN2 : CAN_MODE_NORMAL);
  break;
```

(Reference: `pdscomp/panda` commit `82482b20` "expose TI1 auxiliary CAN".) Without
this you get zero `0x24A` frames and every health gate stays closed with no clue why.

> **Trap that cost us a bring-up:** the original port gated the version byte on
> `== 0x01` only. Current TI firmware reports `0x10`. Result: feedback never
> "healthy" → panda blocked all torque → plus continuous quality-flag failure →
> `safetyRxChecksInvalid` ("controls mismatch"). Frog works because its panda has no
> quality gate at all. If you add a gate, accept both known version bytes — and keep
> the panda gate and the carstate gate in lockstep forever.

---

## 7. Parent-repo (openpilot) changes

- **Params**: `TorqueInterceptorEnabled`, `TorqueInterceptorEnableRequest`,
  `KonikInterlock` — all PERSISTENT|DONT_LOG. `UseKonikServer` defaults to `1` (§1).
- **Two-phase enable**: the settings toggle sets `TorqueInterceptorEnableRequest`;
  at next manager startup `finalize_ti_enable()` runs before backend enforcement and
  calls `enable_ti()` → sets `KonikInterlock` (backend lock) + `TorqueInterceptorEnabled`.
  Staging exists so the panda safety reinit happens on a clean boot with the interlock
  already in place. Hand-setting `TorqueInterceptorEnabled` bypasses the interlock —
  don't.
- **Backend lock**: `is_konik_locked()` is true when any of the lock markers exist —
  the Konik sentinel file, `KonikLockout`, or `KonikInterlock` — and enforcement also
  forces `UseKonikServer=1`. While locked, the backend cannot be switched back to
  comma.ai (§1).
- **Upload hygiene**: the TI/interlock params are on the sunnylink block list.
- **Alert (optional but recommended)**: `CarStateSP.torqueInterceptorReady` +
  `EventNameSP.torqueInterceptorNotReady` raised for TI Mazdas when feedback is
  unhealthy → PERMANENT alert ("Torque Interceptor Not Ready" / "Check interceptor
  wiring and ODB2 power") + `NO_ENTRY`. Plumbing: add the field to the `CarStateSP`
  dataclass in `opendbc/car/structs.py` and to `cereal/custom.capnp`
  (`torqueInterceptorReady @2`), add the event (`torqueInterceptorNotReady @25`),
  and thread `carStateSP` into `CarSpecificEventsSP.update` (signature change — the
  sole caller is `selfdrived.py`). Names the failure instead of generic
  controls-mismatch guesswork.

---

## 8. Porting checklist (fresh fork)

1. DBC: add `CAM_LKAS2` + `TI_FEEDBACK` (§3/§4) to the GEN1 Mazda dbc.
2. values: TI car flag, safety flag, state enum, controller params (§5).
3. mazdacan: `create_ti_steering_control`.
4. carcontroller: TI torque law + unconditional 100 Hz stream (§5).
5. carstate: bus-1 parser, TI torque source, `ti_lkas_allowed`, fault mapping (§5).
6. interface: param-gated flag/safety wiring, GEN1-only raise (§5).
7. panda: safety param bit, rx quality gate (both version bytes!), freshness, TX
   validation, torque limits, fwd block, driver-torque source switch (§6).
8. panda/board: expose bus 1 via `set_can_mode(... CAN_MODE_OBD_CAN2 ...)` in
   `set_safety_mode` when the TI param bit is set (§6 "Board-level").
9. Parent: enable toggle (staged), interlock/backend policy per your fork's stance,
   not-ready alert.
10. Tests: at minimum — healthy frame accepted, wrong bus/length rejected, each fault
    byte rejected, both version bytes accepted, stale-feedback blocks torque,
    boot states (0/1/2) rejected, mirror of command structure (key/duplicate/nibbles).
    Reference suites: `opendbc/safety/tests/test_mazda.py` (TI mixins) and
    `opendbc/car/mazda/tests/test_mazda_carstate.py` on this branch.

## 9. Gotchas

- Version bytes `0x01` and `0x10` must be accepted in **both** gates (panda + carstate)
  and updated together if new firmware appears.
- The `CHKSUM` fields (both directions) are duplicates of the torque value, not
  checksums. Panda's rx check runs with `ignore_checksum = true`; nothing validates a
  real checksum anywhere on this protocol.
- Keep the `CAM_LKAS2` stream running while disengaged; the TI expects the stream.
- Idle feedback center is `0x7F`/`0x80` (±1 LSB noise) — both are zero torque.
- `VERSION_NUMBER=16` plus `STATE=1` plus `VIOL=0x12` at boot is a healthy unit
  powering up, not a fault — wait for RUN.
