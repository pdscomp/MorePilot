from cereal import log


# Once CAN ignition is seen on any valid panda, stop trusting ignitionLine:
# on Mazda it lags ~30s after key-off while ignitionCan falls promptly.
# The latch never resets; resetting would oscillate onroad/offroad.
ignition_can_seen = False


def get_ignition_state(panda_states) -> bool:
  global ignition_can_seen

  valid = [ps for ps in panda_states if ps.pandaType != log.PandaState.PandaType.unknown]
  if not valid:
    return False

  if any(ps.ignitionCan for ps in valid):
    ignition_can_seen = True
    return True

  return False if ignition_can_seen else any(ps.ignitionLine for ps in valid)
