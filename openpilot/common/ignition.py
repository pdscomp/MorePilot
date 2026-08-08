from cereal import log


# Once CAN ignition is seen on any valid panda, stop using ignitionLine to determine
# ignition status. ignitionLine on Mazda stays high for 30s after ignition off.
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
