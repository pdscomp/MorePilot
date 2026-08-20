# Torque Interceptor (TI1) Support

This branch (`ZoomPilot/more-konik`) adds Mazda GEN1 Torque Interceptor support to
ZoomPilot, layered on top of the Konik backend layer (`ZoomPilot/konik`).

The TI communication protocol was ported from the MoreTore fork's `mazda-frogpilot`
branch. Credit for the original reverse engineering and protocol work goes there.

## Why this fork defaults to the Konik backend

comma.ai connect has been reported to block users running a Torque Interceptor, and
possibly even users running a branch that merely supports one. To protect users, this
fork:

- **Defaults to the Konik backend** (`UseKonikServer` defaults to `1`). Driving data,
  dongle identity, and telemetry go to Konik, not comma.ai.
- **Locks the backend once TI has ever been enabled.** Enabling TI sets the
  `KonikInterlock` param, which prevents switching back to the comma.ai backend-even
  if you later disable TI or flash a branch that defaults to comma connect.
- **Blocks TI-related params from sunnylink upload** (`TorqueInterceptorEnabled`,
  `TorqueInterceptorEnableRequest`, `KonikInterlock`, and friends are on the
  `DONT_LOG` list and the sunnylink block list).

The only way back to the comma.ai backend after enabling TI is a **factory reset**
(multi-tap the screen during boot). That is deliberate: the factory reset wipes all
locally stored driving data that could otherwise be uploaded to comma connect and
reveal that the device ran a TI. If you simply switched branches without the reset,
that data would be sitting there waiting to tell on you.

Do not bypass the interlock. If you think you have a reason to, you don't.

## How TI mode differs from normal ZoomPilot

With `TorqueInterceptorEnabled=0`, behavior is byte-identical to stock ZoomPilot:
same panda safety config, same CAN traffic, same carstate parsing. Every TI path is
gated on the flag.

With TI enabled (`_initialize_mazda` in `opendbc/sunnypilot/car/interfaces.py`):

- `MazdaFlags.TORQUE_INTERCEPTOR` is set, `MazdaSafetyFlags.TORQUE_INTERCEPTOR` is
  OR'd into the panda safety param, `minSteerSpeed=0`, `steerAtStandstill=True`,
  `dashcamOnly=False`. GEN1 only-enabling on GEN2/GEN3 raises.
- **Steering commands** go out two ways every 10 ms:
  - `CAM_LKAS` (0x243, bus 0) — normal camera impersonation, unchanged.
  - `CAM_LKAS2` (0x249, bus 1) — TI side channel: 12-bit `LKAS_REQUEST`, a duplicate
    of the request in the `CHKSUM` field, and the magic `KEY` (0xC461CE60 on the wire).
- **Driver torque** is read from the TI's own torque sensor in the feedback frame
  instead of the EPS `STEER_TORQUE` message.
- **TI torque limits** (panda-enforced): max ±600, rate up 6 / down 15 per 10 ms step
  (slow wind-up, fast release), real-time delta 192, driver allowance 15 with
  multiplier 40. Distinct from the camera-path limits (±800, up 10 / down 25).

### Feedback frame (0x24A, bus 1, 50 Hz)

| Byte | Field | Notes |
|------|-------|-------|
| 0 | `TI_TORQUE_SENSOR` | signed, offset −127; mirrors byte 1 on healthy firmware |
| 1 | `CHKSUM` | same value as byte 0 |
| 2 | `VERSION_NUMBER` | known-good: `0x01` and `0x10` (current firmware) |
| 3 | `STATE` | 0=OFF, 1=PRE_FAULT, 2=PRE_RUN, 3=RUN; 1→2→3 is the normal ~2 s boot |
| 4 | `VIOL` | must be 0 |
| 5 | `ERROR` | must be 0 |
| 6 | `RAMP_DOWN` | must be 0 |
| 7 | spare | observed 0x30 |

Example healthy frame: `7f7f100300000030`.

### Health gating (fail-closed, two layers)

- **Panda** (`opendbc/safety/modes/mazda.h`): an rx quality flag on `0x24A` requires
  bus 1, a known version byte, `STATE==RUN`, and zero fault bytes. Feedback must also
  be fresh (≤40 ms). If either fails, panda blocks all nonzero TI torque commands.
  TX validation additionally requires the magic key, matching duplicate torque, and
  clear reserved bits.
- **carstate**: `ti_lkas_allowed` mirrors the same conditions. While TI is enabled
  and not allowed, `steerFaultTemporary` is set and the onroad event
  `torqueInterceptorNotReady` fires ("Torque Interceptor Not Ready — check
  interceptor wiring and ODB2 power") with `NO_ENTRY`, so a sick TI blocks
  engagement instead of silently degrading.

### Two-phase enable

Toggling TI in settings sets `TorqueInterceptorEnableRequest`; on the next manager
startup `finalize_ti_enable()` completes activation by calling `enable_ti()`, which
sets the `KonikInterlock` (locking the backend per above) and then
`TorqueInterceptorEnabled`. The staging exists so the panda safety reinit happens on
a clean boot with the interlock already in place.

## Where the changes live

**Main repo (this one):**
- `openpilot/common/params_keys.h` — `TorqueInterceptorEnabled`,
  `TorqueInterceptorEnableRequest`, `KonikInterlock` (all PERSISTENT|DONT_LOG),
  `UseKonikServer` (defaults to `1`).
- `openpilot/common/api/backend.py` — `is_konik_locked()` reads `KonikInterlock`;
  `enable_interlock()`, `enable_ti()`, `request_ti_enable()`, `finalize_ti_enable()`.
- `openpilot/system/manager/manager.py` — `finalize_ti_enable()` runs before backend
  enforcement at startup.
- `openpilot/sunnypilot/sunnylink/athena/sunnylinkd.py` — TI/interlock params blocked
  from upload.
- `openpilot/cereal/custom.capnp` — `CarStateSP.torqueInterceptorReady`,
  `EventNameSP.torqueInterceptorNotReady`.
- `openpilot/sunnypilot/selfdrive/car/car_specific.py` — raises the not-ready event
  for TI Mazdas with unhealthy feedback.
- `openpilot/sunnypilot/selfdrive/selfdrived/events.py` — alert text + `NO_ENTRY`.
- Settings UI — TI toggle (staged enable) and Konik server selection.

**opendbc submodule (`pdscomp/opendbc`, branch `zoompilot/more-konik`):**
- `opendbc/car/mazda/mazdacan.py` — `create_ti_steering_control()` (CAM_LKAS2).
- `opendbc/car/mazda/carcontroller.py` — TI torque computation + rate limiting, sent
  alongside the normal camera path.
- `opendbc/car/mazda/carstate.py` — TI feedback parser, torque sensor source,
  `ti_lkas_allowed`, `torqueInterceptorReady`.
- `opendbc/sunnypilot/car/interfaces.py` — `_initialize_mazda()` flag/safety wiring.
- `opendbc/car/mazda/values.py` — `MazdaFlags.TORQUE_INTERCEPTOR`,
  `MazdaSafetyFlags.TORQUE_INTERCEPTOR`, TI controller params.
- `opendbc/dbc/mazda_2017.dbc` — `CAM_LKAS2` and `TI_FEEDBACK` message definitions.
- `opendbc/safety/modes/mazda.h` — panda safety: TI rx quality flag + freshness,
  TX validation, torque limits, fail-closed on stale/unhealthy feedback.

**panda submodule (`pdscomp/panda`, branch `zoompilot/add-ti-konik`):**
- Exposes the TI1 auxiliary CAN bus (bus 1) for the side channel.

## Notes and gotchas

- TI firmware version byte: gates accept `0x01` and `0x10`. If a future TI firmware
  reports something else, both gates (panda + carstate) must move together or the car
  will show "Torque Interceptor Not Ready" and refuse to steer.
- Frog's panda has no feedback quality gate at all; ours is deliberately stricter
  while remaining compatible with observed hardware.
- Setting `TorqueInterceptorEnabled` by hand (params CLI) bypasses the interlock and
  is exactly how you end up uploading evidence to comma.ai. Don't.
