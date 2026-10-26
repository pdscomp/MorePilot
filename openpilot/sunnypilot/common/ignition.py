"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import log


# Once CAN ignition is seen on any valid panda, it owns ignition for the rest of
# this process. Mazda's ignitionLine stays high for roughly 30 s after key-off;
# using it as a grace period keeps selfdrived onroad while the CAN bus stops.
_ignition_can_seen = False


def get_ignition_state(panda_states) -> bool:
  global _ignition_can_seen

  valid = [ps for ps in panda_states if ps.pandaType != log.PandaState.PandaType.unknown]
  if not valid:
    return False

  if any(ps.ignitionCan for ps in valid):
    _ignition_can_seen = True
    return True

  return False if _ignition_can_seen else any(ps.ignitionLine for ps in valid)
