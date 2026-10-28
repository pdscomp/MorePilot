"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v42 import (
  KD_INTERP, KD_INTERP_SPEEDS, LatControlTorque as LatControlTorqueV42,
)

VERSION = 43
# TEMPORARY: trial search point (screen 0, 0.05, 0.10, 0.15, 0.20 s).
# Replace with a validated value and rationale after calibration review; remove this note then.
HIGHWAY_KD = 0.10


class LatControlTorque(LatControlTorqueV42):
  VERSION = VERSION
  # Preserve the low-speed tune and fade out above the experimental band.
  KD_SCHEDULE = [
    [*KD_INTERP_SPEEDS, 17.5, 25.0, 30.0],
    [*KD_INTERP, HIGHWAY_KD, HIGHWAY_KD, 0.0],
  ]
