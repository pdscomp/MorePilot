"""Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import inspect
import threading
from pathlib import Path

import pytest

from openpilot.cereal import custom, messaging
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

  def test_subscription_and_replay_coverage(self):
    # sm['carStateSP'] is a bare dict lookup over subscribed sockets: the service must be in
    # selfdrived's SubMaster list AND in process_replay's pubs for selfdrived, or the read is a
    # first-frame KeyError on-car and replay feeds the ICBM call a silent default session.
    sm_block = inspect.getsource(SelfdriveD.__init__).split("messaging.SubMaster([", 1)[1].split("frequency=", 1)[0]
    assert "'carStateSP'" in sm_block

    replay_src = (Path(__file__).resolve().parents[5] /
                  "openpilot/selfdrive/test/process_replay/process_replay.py").read_text()
    selfdrived_pubs = replay_src.split('proc_name="selfdrived"', 1)[1].split("subs=", 1)[0]
    assert '"carStateSP"' in selfdrived_pubs

  def test_sm_roundtrip_reads_nested_state(self):
    # real message over real msgq transport, read back with the exact production expression
    sm = messaging.SubMaster(['carStateSP'])
    pm = messaging.PubMaster(['carStateSP'])
    msg = messaging.new_message('carStateSP')
    msg.valid = True
    msg.carStateSP.zoompilot.cruiseSession.state = "pending"
    threading.Timer(0.05, pm.send, args=("carStateSP", msg)).start()
    sm.update(2000)
    assert sm.updated['carStateSP']
    assert sm['carStateSP'].zoompilot.cruiseSession.state == "pending"
