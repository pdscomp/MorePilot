# Mazda CX-5 2022: steeringPressed threshold analysis

## Problem

The Mazda EPS asserts `LKAS_BLOCK` during driver steering override (not just real faults).
`car_specific.py` suppresses `steerFaultTemporary` alerts when `steeringPressed` is True,
but the original threshold (15) with no hysteresis causes flicker during gentle turns when
driver torque hovers near the boundary.

## Data (from device logs)

### Our drive — route `98673c0899e64dff|00000016--82725de551` (50 segments, 295,692 frames)

**Passive driving (openpilot active, no fault, hands on wheel):**
- P50=1, P90=2, P95=2, P99=5, P99.9=24, Max=34
- 99.74% of frames are below threshold 15

**During steerFaultTemporary (driver override):**
- Override (steeringPressed=True): Min=16, P50=25, P90=35, Max=50
- Unpressed during fault (flicker): torques 0-15

**Alert impact:**
- Without hysteresis: 191 silent alert frames across 3 episodes
- With hysteresis (min_count=5): 91 silent alert frames (52% reduction)
- Zero loud alerts in both cases

### User route — `eeeddc9b7ebf4be6|0000000b--dd572f22cb` (2,848 frames)

**During steerFaultTemporary:**
- Override (pressed): Min=17, P5=18, Median=26
- Unpressed during fault: values [0, 1, 1, 1, 1, 4, 5, 5, 5, 5, 5, 5, 6, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 10, 11, 11, 14, 15, 15, 15, 15]

**Three distinct torque zones:**
| Zone | Torque | Meaning |
|------|--------|---------|
| Hands off | 0-11 | Should trigger alert |
| Flicker | 12-16 | Threshold boundary noise |
| Active override | 17+ | Should suppress alert |

### Threshold comparison across cars

| Car | Threshold | Hysteresis | Notes |
|-----|-----------|------------|-------|
| Mazda (current) | 15 | none | Causes flicker |
| Toyota | 100 | none | EPS has distinct fault codes |
| Hyundai | 150-250 | 5-frame | |
| Ford | 1.0 Nm | 5-frame | |
| Tesla | 1 Nm | 5-frame | |
| Rivian | 1.0 Nm | 5-frame | |
| Subaru | 75-80 | none | |
| Chrysler | 120 | none | |
| VW | 60-80 | none | |
| GM | 1.0 Nm | none | |

## Decision

- **Keep threshold at 15** — correctly separates hands-off (0-11) from override (17+)
- **Add `update_steering_pressed` hysteresis with min_count=5** — the modern pattern used by
  Hyundai, Ford, Tesla, Rivian. Debounces the 12-16 flicker zone with 60ms filtering.
  Does not affect hands-off detection (torque 0-11 is well below threshold).

## Verification

Replaying both routes through car_specific.py filter logic:
- Zero loud alerts before and after
- Silent alert frames reduced 52% (our drive) and 31% (user route)
- Remaining silent alerts are from genuine 900ms+ unpressed gaps — correct behavior
