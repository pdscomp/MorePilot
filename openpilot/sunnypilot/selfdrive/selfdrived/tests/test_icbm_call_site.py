"""Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import inspect

import pytest

from openpilot.cereal import custom
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD


class TestIcbmCallSite:
  """selfdrived hands icbm.run the session state at carStateSP.zoompilot.cruiseSession.state.
  The 2026-09 restack dropped `.zoompilot`, and capnp readers raise AttributeError on the
  flat path at runtime - every ICBM press frame, on-device only, while every harness that
  stubs the message stayed green."""

  def test_call_site_uses_the_nested_path(self):
    source = inspect.getsource(SelfdriveD.update_events)
    assert ".zoompilot.cruiseSession.state" in source
    assert "carStateSP'].cruiseSession" not in source

  def test_flat_path_really_raises(self):
    msg = custom.CarStateSP.new_message()
    msg.zoompilot.cruiseSession.state = "preActive"
    assert msg.zoompilot.cruiseSession.state == "preActive"
    with pytest.raises(AttributeError):
      msg.cruiseSession
