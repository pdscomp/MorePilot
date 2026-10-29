"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog


# Once CAN ignition is seen on any valid panda, it owns ignition for the rest of
# this process. Mazda's ignitionLine stays high for roughly 30 s after key-off;
# using it as a grace period keeps selfdrived onroad while the CAN bus stops.
_ignition_can_seen = False

# Brief CAN ignition dropouts (connector wobble, ECU hiccups) used to bounce the
# whole driving stack offroad and back. Require the drop to persist this long
# before letting go; real key-off still goes offroad one debounce later.
CAN_IGNITION_DROP_DEBOUNCE_S = 2.0
_ignition_can_drop_since = None
# Set once the debounce has expired for the current dropout episode, so the
# expiry is logged a single time instead of every frame.
_ignition_can_drop_expired = False


def get_ignition_state(panda_states, now=None) -> bool:
  global _ignition_can_seen, _ignition_can_drop_since, _ignition_can_drop_expired

  valid = [ps for ps in panda_states if ps.pandaType != log.PandaState.PandaType.unknown]
  if not valid:
    # Link loss joins the dropout episode instead of starting a fresh one: keep the
    # debounce clock running so a reconnect with the CAN signal still low cannot
    # manufacture a new 2 s ignition window out of thin air.
    if _ignition_can_seen and _ignition_can_drop_since is None:
      _ignition_can_drop_since = time.monotonic() if now is None else now
      cloudlog.warning("ignition: CAN dropout started (no valid panda)")
    return False

  if any(ps.ignitionCan for ps in valid):
    if _ignition_can_seen and _ignition_can_drop_since is not None:
      cloudlog.warning("ignition: CAN recovered after %.1fs", (time.monotonic() if now is None else now) - _ignition_can_drop_since)
    _ignition_can_seen = True
    _ignition_can_drop_since = None
    _ignition_can_drop_expired = False
    return True

  if not _ignition_can_seen:
    return any(ps.ignitionLine for ps in valid)

  now = time.monotonic() if now is None else now
  if _ignition_can_drop_since is None:
    _ignition_can_drop_since = now
    cloudlog.warning("ignition: CAN dropout started")
  if not _ignition_can_drop_expired and now - _ignition_can_drop_since >= CAN_IGNITION_DROP_DEBOUNCE_S:
    _ignition_can_drop_expired = True
    cloudlog.warning("ignition: CAN dropout debounce expired, letting ignition go")
  return now - _ignition_can_drop_since < CAN_IGNITION_DROP_DEBOUNCE_S
