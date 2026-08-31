"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from typing import cast

from openpilot.common.params import Params


def get_fixed_lat_delay(params: Params, steer_actuator_delay: float) -> float:
  software_delay = cast(float, params.get("LagdToggleDelay", return_default=True))
  return steer_actuator_delay + software_delay


def get_lat_delay(params: Params, stock_lat_delay: float, steer_actuator_delay: float) -> float:
  # live learning on: use what lagd publishes.
  # off: derive the fixed total directly, independent of LagdValueCache update order.

  if params.get_bool("LagdToggle"):
    return stock_lat_delay

  return get_fixed_lat_delay(params, steer_actuator_delay)
