# Mazda alpha long: the gas pedal hands the powertrain back mid-decel

Reported 2026-08-01: "Something's a little off with the acceleration and the auto speed. When I
try lightly giving some more gas when it slows down in the city it'll rev up and multiply how
much I press x10."

Reporter route: `7c735af5fce56485/00000008--42875af0a9` (12 of 25 segments uploaded).

## What happens today

`CS.gasPressed` is `PEDAL_GAS > 0`, so any pedal input at all raises `gasPressedOverride`
(`selfdrive/car/car_specific.py`), which is an `OVERRIDE_LONGITUDINAL` event, which clears
`CC.longActive` in `controlsd.py`. The Mazda carcontroller keyed everything off `CC.longActive`,
so in the same frame:

- `ACCEL_CMD` snapped from whatever it was to exactly `0.000`
- `CRZ_INFO.ACC_ACTIVE` and `CRZ_CTRL.CRZ_ACTIVE` dropped to 0
- the stop-and-go state machine reset to `CRUISING`

The PCM therefore left ACC mode and reverted to pure driver demand, in whatever gear the ACC
decel had selected. All of openpilot's braking vanished at the instant the driver added a little
throttle, so the acceleration swing was far larger than the pedal alone accounts for. That is the
"x10".

Measured on the reporter's route, 14 gas presses while alpha long was active:

| | |
|---|---|
| `longActive` dropped during the press | 14 / 14 |
| commanded accel during the press | `0.000` (mean and median) |
| presses that started while openpilot was braking | 6 |
| largest accel swing | -0.27 to +1.57 m/s2 at 44 kph |
| worst case (city creep) | -0.56 to +1.98 m/s2 at 3.7 kph, revs 700 to 2000 |

Regenerate the timeline figure with
`tools/mazda_long/plot_gas_override.py <route>` (docs images are not tracked: `.lfsconfig` points
LFS at sunnypilot's GitLab, which we cannot push to, so a new LFS object here would break device
installs). It shows the command and the wire dropping to zero in one frame, the engaged bits
going with them, and measured accel swinging +2.5 m/s2.

## What stock MRCC does instead

`tools/mazda_long/analyze_gas_override.py` over 576 stock segments found 31 gas presses while
MRCC was engaged, 11 of them starting while stock was commanding a real decel:

- stock keeps `ACC_ACTIVE` / `CRZ_ACTIVE` / `PEDALS.ACC_ACTIVE` set through 9 of those 11. The
  two that dropped were genuine disengagements (command went to the `+4.094` standby pattern).
- stock keeps commanding. It does not release the brake for roughly the first half second, then
  eases off at about 0.6 m/s2 per second:

  | time into the press | command minus pre-press command |
  |---|---|
  | +0.10 s | -0.135 m/s2 |
  | +0.50 s | -0.122 m/s2 |
  | +1.00 s | +0.143 m/s2 |
  | +2.00 s | +0.762 m/s2 |

- the driver's pedal still wins: the car accelerates anyway (for example -1.13 to +0.87 m/s2 with
  the command held at -1.84). The PCM arbitrates driver torque against the ACC request, which is
  why stock can stay engaged and still feel proportional.

## Fix

### 1. Stay engaged through the override (`carcontroller.py`)

`gas_override = CC.enabled and (CC.cruiseControl.override or CS.out.gasPressed)`, and
`long_engaged = CC.longActive or gas_override` now drives `ACC_ACTIVE`, `CRZ_ACTIVE`, the
stop-and-go state machine, the synthetic lead, and the gap display. The command still goes to
zero, as it does on every other port.

This is Honda's pattern. `create_acc_commands` takes `enabled` and `active` separately and
drives the engaged bit off the former:

```python
# opendbc/car/honda/hondacan.py
control_on = 5 if enabled else 0        # CC.enabled: survives a gas override
accel_command = accel if active else 0  # CC.longActive: zeroed by it
```

Toyota has no enable bit at all (the ACC frame flows unconditionally). Ford does gate its
`Cmbb_B_Enbl` on `longActive`, so there is no universal convention; the tiebreaker is what the
car's own stock system sends, which is what the 576-segment measurement above settles.

Panda already allows it: `controls_allowed` tracks `PEDALS.ACC_ACTIVE`, not the pedal.

Side effect: `StopAndGoStateMachine`'s `gas_override` argument was unreachable before this. It
was only consulted when `long_active` was true, and `long_active` was false whenever the gas was
pressed. Gas now releases a standstill hold the way stock does.

### 2. Slew limit the command (Toyota's pattern)

Toyota rate limits the command on the active path and tracks the previous value straight through
inactive periods, so retaking control ramps instead of stepping:

```python
pcm_accel_cmd = actuators.accel
if CC.longActive:
    pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, WINDDOWN, WINDUP)
self.prev_accel = pcm_accel_cmd   # tracked even when inactive
```

Mazda now does the same, with `accel_last` tracked through overrides. The standstill hold and
resume commands are byte-exact stock replays and bypass the limit.

The limits are **asymmetric**, unlike Toyota's symmetric 4.0 m/s3, and that is deliberate:

| | value | why |
|---|---|---|
| `ACCEL_WINDUP_LIMIT` | 4.0 m/s3 | above the p99 of the plan's own up-slew on the reporter's route (p99 +3.2, p99.9 +6.3, max +34), so it only clips state-transition steps. Windup is the direction that dumps the brake, which is the driver-felt problem. |
| `ACCEL_WINDDOWN_LIMIT` | -10.0 m/s3 | loose on purpose. A tight winddown limit delays real braking (4.0 m/s3 would take 0.875 s to reach `ACCEL_MIN`) for no measured benefit; -10.0 clips only the p99.9+ steps. |

An earlier revision instead held the decel through the override and eased it off at the measured
stock rate of 0.6 m/s3. That was dropped: no port does it, it means commanding brake while the
driver is on the accelerator, and the measured swing (+2.54 m/s2) is far larger than the brake
command that vanished (0.56 m/s2), so the mode exit is what matters, not the brake release.

### 3. Report the value actually sent

`new_actuators.accel = self.accel_last`. Toyota, Ford and Honda all set this; Mazda did not, so
`carOutput.actuatorsOutput.accel` carried the plan's value rather than the wire's, missing the
envelope clip, the hold commands, and the override zero. Any log analysis of what the car was
actually told was wrong before this.

## What Toyota does that we deliberately do not

Checked against the reporter's route rather than adopted on faith:

- **Pitch compensation.** Toyota adds `sin(pitch) * g` to the request and uses it to gate
  `PERMIT_BRAKING`. If our gravity term were uncompensated, the tracking residual would regress
  on `sin(pitch)*g` with slope ~1.0. Measured slope is **-0.13**, correlation -0.09, so the
  residual is not gravity. The `livePose` pitch on this route also never leaves -6.8 to -0.6 deg,
  which is a device mount offset dominating any real grade, so this route cannot settle it
  properly either. Not adopted; revisit with a route that has real hills.
- **A carcontroller-level PID** on `a_ego_future` (jerk-extrapolated) correcting the PCM's
  tracking. Not needed here: accel error over 38215 engaged samples is mean **+0.011**, std
  **0.209** m/s2. The Mazda PCM tracks the request well enough that closed-loop correction has
  nothing to fix.
- **`permit_braking` / `standstill_req`.** Toyota-specific signals with no CRZ_INFO equivalent;
  our stop-and-go state machine already replays the stock stop bits.

Also checked and left alone: `gasPressed = PEDAL_GAS > 0`. The pedal reads a clean zero 87.6% of
frames, and only 0.22% of nonzero readings are below 8 counts (full scale ~2048), so the
threshold is not picking up rest noise. `vEgoStopping`, `vEgoStarting`, `stoppingDecelRate` and
`startingState` exist in `car.capnp` but nothing in the current tree reads them.

## Not reproduced

- **Re-engagement after the pedal comes up** is prompt: median 0.07 s from gas release to
  `longActive` across 42 releases on the reporter's route. The long tail (p90 22 s) is presses
  where the driver kept driving manually.
- **Longitudinal tracking while engaged is fine**: accel error mean +0.009, std 0.208 m/s2 over
  39285 engaged samples. Commanded accel p5/p50/p95 = -1.06 / 0.00 / +0.59 m/s2, well inside the
  2.0 / -3.5 envelope.
- **The "auto speed" half is not openpilot**. openpilot transmitted zero `CRZ_BTNS` frames in the
  whole route, so it never touched the set speed. The 8 kph (5 mph) steps every 0.6 s in
  `CRZ_SPEED` are the wheel's own repeat rate while "+" is held. Needs a more specific report.

## Tools

- `tools/mazda_long/analyze_gas_override.py` - stock MRCC override behaviour, engaged-bit
  retention, and the command release profile.
- `tools/mazda_long/analyze_gas_override_op.py` - the same episodes on an alpha long drive.
  Accepts a route identifier as well as globs.
- `tools/mazda_long/plot_gas_override.py` - single-episode timeline figure.
