"""The on-device ui.py import chain (MainLayout -> settings -> sub-layouts) must import clean.

No host test builds the tici settings sub-layouts, so a stale import in one only
shows up as a ui crashloop on device (found the hard way: lane_change_settings
imported PACE_* symbols the level-based smoothing no longer exports).
"""
import importlib

import pytest

SUB_LAYOUT_MODULES = [
  "openpilot.selfdrive.ui.sunnypilot.layouts.settings.steering_sub_layouts.lane_change_settings",
]


@pytest.mark.parametrize("module", SUB_LAYOUT_MODULES)
def test_settings_sub_layout_imports(module):
  importlib.import_module(module)
