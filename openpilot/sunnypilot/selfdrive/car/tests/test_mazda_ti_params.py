import pytest

from openpilot.sunnypilot.selfdrive.car.interfaces import initialize_params


class FakeParams:
  def __init__(self, enabled):
    self.enabled = enabled

  def get(self, key, return_default=False):
    assert return_default
    return self.enabled if key == "TorqueInterceptorEnabled" else False


@pytest.mark.parametrize("enabled", [False, True])
def test_torque_interceptor_param_is_forwarded_to_opendbc(enabled):
  params = initialize_params(FakeParams(enabled))
  assert {"TorqueInterceptorEnabled": enabled} in params
