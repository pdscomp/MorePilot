"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The tizi (comma three / 3X) settings: the top of Cruise owns the alpha switch
(offroad-only, no status line) and experimental mode, which Developer and Toggles no longer
show. Its own module: the big UI is chosen at import, so it cannot share a window with the mici
tests.
"""
import os

import pytest

os.environ["BIG"] = "1"
os.environ.setdefault("SCALE", "0.5")  # a full-size 2160x1080 hidden window aborts on macOS


def test_alpha_toggle_is_offroad_only(params, monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.ui_state import ui_state
  layout = AlphaLongitudinalToggles()
  monkeypatch.setattr(ui_state, "started", True)
  assert not layout._alpha_long_toggle.action_item.enabled
  monkeypatch.setattr(ui_state, "started", False)
  assert layout._alpha_long_toggle.action_item.enabled


def test_moved_switches_appear_once(params):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise import CruiseLayout
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.developer import DeveloperLayoutSP
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.toggles import TogglesLayoutSP

  developer, toggles = DeveloperLayoutSP(), TogglesLayoutSP()
  assert developer._alpha_long_toggle not in developer._scroller._items
  assert not toggles._toggles["ExperimentalMode"].is_visible
  assert not hasattr(CruiseLayout(), "dec_toggle")

  alpha = AlphaLongitudinalToggles()
  assert [key for key, _ in alpha._refresh_toggles] == [
    "AlphaLongitudinalEnabled", "ExperimentalMode", "ExperimentalModeSetSpeed", "ExperimentalModeLeadGap",
    "DynamicExperimentalControl"]


def test_alpha_longitudinal_heads_cruise(params, monkeypatch):
  from opendbc.car.structs import car
  from openpilot.cereal import custom
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise import CruiseLayout
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(alphaLongitudinalAvailable=True, openpilotLongitudinalControl=True))
  monkeypatch.setattr(ui_state, "CP_SP", custom.CarParamsSP.new_message())
  monkeypatch.setattr(ui_state, "has_longitudinal_control", True)
  params.put_bool("ExperimentalModeSetSpeed", True, block=True)
  cruise = CruiseLayout()
  # plain toggles, no sub-panel
  assert cruise._scroller._items[:len(cruise._alpha_long.items)] == cruise._alpha_long.items
  cruise.show_event()
  cruise._update_state()
  assert cruise._alpha_long._alpha_long_toggle.is_visible
  assert cruise._alpha_long._set_speed_toggle.action_item.get_state()
  params.remove("ExperimentalModeSetSpeed")


def test_experimental_mode_text_matches_upstream(params, monkeypatch):
  # upstream writes this text inline in TogglesLayout._update_toggles; the panel keeps a copy with our name
  from opendbc.car.structs import car
  from openpilot.selfdrive.ui.layouts.settings.toggles import TogglesLayout
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import EXPERIMENTAL_MODE_DESCRIPTION
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "update_params", lambda: None)
  monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(openpilotLongitudinalControl=True))
  monkeypatch.setattr(ui_state, "has_longitudinal_control", True)
  toggles = TogglesLayout()
  toggles._update_toggles()
  assert toggles._toggles["ExperimentalMode"].description.replace("sunnypilot", "zoompilot") == EXPERIMENTAL_MODE_DESCRIPTION


def test_set_speed_greys_out_under_dec(params, monkeypatch):
  from opendbc.car.structs import car
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(alphaLongitudinalAvailable=True, openpilotLongitudinalControl=True))
  monkeypatch.setattr(ui_state, "has_longitudinal_control", True)
  layout = AlphaLongitudinalToggles()
  layout._dec_toggle.action_item.set_state(False)
  layout.update_state()
  assert layout._set_speed_toggle.action_item.enabled
  layout._dec_toggle.action_item.set_state(True)
  layout.update_state()
  assert not layout._set_speed_toggle.action_item.enabled


@pytest.fixture
def isolated_longitudinal_params(params):
  keys = ("CarParamsPersistent", "CarParamsSPPersistent", "AlphaLongitudinalEnabled", "ExperimentalMode",
          "LiveTorqueParameters", "LiveTorqueParametersSP")
  for key in keys:
    params.remove(key)
  yield
  for key in keys:
    params.remove(key)


@pytest.mark.parametrize("started", [False, True])
@pytest.mark.parametrize("stored", [True, False, None])
def test_unknown_cp_preserves_longitudinal_preferences(params, isolated_longitudinal_params, monkeypatch, started, stored):
  from openpilot.selfdrive.ui.ui_state import ui_state

  for key in ("CarParamsPersistent", "CarParamsSPPersistent"):
    params.remove(key)
  monkeypatch.setattr(ui_state, "CP", None)
  monkeypatch.setattr(ui_state, "CP_SP", None)
  monkeypatch.setattr(ui_state, "started", started)
  monkeypatch.setattr(ui_state, "has_longitudinal_control", False)
  keys = ("AlphaLongitudinalEnabled", "ExperimentalMode")
  for key in keys:
    if stored is None:
      params.remove(key)
    else:
      params.put_bool(key, stored, block=True)
  before = {key: params.get(key) for key in keys}

  ui_state.update_params()
  ui_state.update_params()

  assert {key: params.get(key) for key in keys} == before
  assert not ui_state.has_longitudinal_control


def test_quarantine_does_not_erase_longitudinal_preferences(params, isolated_longitudinal_params, monkeypatch, tmp_path):
  from opendbc.car.structs import car
  from openpilot.selfdrive.ui.ui_state import ui_state
  from openpilot.sunnypilot.system import params_migration

  monkeypatch.setattr(params_migration, "_mazda_meta_dir", lambda _params: tmp_path)
  for key in params_migration._MAZDA_CACHE_ARCHIVE_KEYS:
    params.remove(key)
  cp = car.CarParams.new_message(
    brand="mazda", carFingerprint="MAZDA_CX5", flags=35,
    alphaLongitudinalAvailable=True, openpilotLongitudinalControl=True,
  )
  cp.lateralTuning.init("torque")
  cp_bytes = cp.to_bytes()
  params.put("CarParamsPersistent", cp_bytes, block=True)
  for key in ("LiveTorqueParameters", "LiveTorqueParametersSP"):
    params.put(key, b"untrusted learning", block=True)
  for key in ("AlphaLongitudinalEnabled", "ExperimentalMode"):
    params.put_bool(key, True, block=True)
  monkeypatch.setattr(ui_state, "CP", None)
  monkeypatch.setattr(ui_state, "CP_SP", None)
  monkeypatch.setattr(ui_state, "started", False)
  monkeypatch.setattr(ui_state, "has_longitudinal_control", False)

  params_migration.migrate_mazda_control_cache(params)
  assert params.get("CarParamsPersistent") is None
  assert params.get("LiveTorqueParameters") is None
  assert params.get("LiveTorqueParametersSP") is None
  ui_state.update_params()
  assert params.get_bool("AlphaLongitudinalEnabled")
  assert params.get_bool("ExperimentalMode")
  assert not ui_state.has_longitudinal_control

  params.put("CarParamsPersistent", cp_bytes, block=True)
  ui_state.update_params()
  assert ui_state.has_longitudinal_control
  assert params.get_bool("AlphaLongitudinalEnabled")
  assert params.get_bool("ExperimentalMode")


@pytest.mark.parametrize("alpha_available,op_long,alpha_enabled,expected_alpha,expected_experimental", [
  (True, True, True, True, True),
  (True, False, False, False, None),
  (False, False, True, None, None),
  (False, True, True, None, True),
])
def test_known_cp_longitudinal_constraints(params, isolated_longitudinal_params, monkeypatch, alpha_available, op_long, alpha_enabled,
                                           expected_alpha, expected_experimental):
  from opendbc.car.structs import car
  from openpilot.selfdrive.ui.ui_state import ui_state

  params.remove("CarParamsSPPersistent")
  cp = car.CarParams.new_message(alphaLongitudinalAvailable=alpha_available, openpilotLongitudinalControl=op_long)
  params.put("CarParamsPersistent", cp.to_bytes(), block=True)
  params.put_bool("AlphaLongitudinalEnabled", alpha_enabled, block=True)
  params.put_bool("ExperimentalMode", True, block=True)
  monkeypatch.setattr(ui_state, "CP", None)
  monkeypatch.setattr(ui_state, "CP_SP", None)
  monkeypatch.setattr(ui_state, "started", False)
  monkeypatch.setattr(ui_state, "has_longitudinal_control", False)

  ui_state.update_params()

  assert params.get("AlphaLongitudinalEnabled") == expected_alpha
  assert params.get("ExperimentalMode") == expected_experimental
  assert ui_state.has_longitudinal_control == (alpha_enabled if alpha_available else op_long)


def test_hidden_settings_preserve_preferences_with_unknown_cp(params, isolated_longitudinal_params, monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.developer import DeveloperLayoutSP
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.toggles import TogglesLayoutSP
  from openpilot.selfdrive.ui.ui_state import ui_state

  for key in ("CarParamsPersistent", "CarParamsSPPersistent"):
    params.remove(key)
  monkeypatch.setattr(ui_state, "CP", None)
  monkeypatch.setattr(ui_state, "CP_SP", None)
  monkeypatch.setattr(ui_state, "started", False)
  monkeypatch.setattr(ui_state, "has_longitudinal_control", False)
  for key in ("AlphaLongitudinalEnabled", "ExperimentalMode"):
    params.put_bool(key, True, block=True)

  monkeypatch.setattr(ui_state, "_engaged_transition_callbacks", ui_state._engaged_transition_callbacks.copy())
  DeveloperLayoutSP()._update_toggles()
  TogglesLayoutSP()._update_toggles()

  assert params.get_bool("AlphaLongitudinalEnabled")
  assert params.get_bool("ExperimentalMode")
  assert not ui_state.has_longitudinal_control
