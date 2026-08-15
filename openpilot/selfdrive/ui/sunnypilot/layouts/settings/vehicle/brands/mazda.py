"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.api.backend import put_bool_checked, request_ti_enable
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog


TI_CONFIRMATION = tr_noop(
  "Enabling Torque Interceptor permanently switches Connect to Konik Stable and blocks comma.ai until a factory reset. "  # noqa: ISC002
  "Existing and future drive uploads will go to Konik. A reboot is required. Reboot now?"
)
TI_DESCRIPTION = tr_noop("Use Mazda GEN1 Torque Interceptor v1 steering hardware. Offroad only; a reboot is required.")
TI_PENDING_DESCRIPTION = tr_noop("Torque Interceptor enablement is pending reboot.")


class MazdaSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.ti_toggle = toggle_item_sp(
      tr("Torque Interceptor"),
      "",
      initial_state=ui_state.params.get_bool("TorqueInterceptorEnabled"),
      callback=self._on_ti_toggled,
    )
    self.items = [self.ti_toggle]

  def _restore_toggle(self):
    self.ti_toggle.action_item.set_state(ui_state.params.get_bool("TorqueInterceptorEnabled"))

  def _on_ti_toggled(self, enabled):
    def handle_reboot(result):
      try:
        if result != DialogResult.CONFIRM or not ui_state.is_offroad():
          return
        if enabled:
          request_ti_enable(ui_state.params)
        else:
          put_bool_checked(ui_state.params, "TorqueInterceptorEnabled", False)
        try:
          put_bool_checked(ui_state.params, "DoReboot", True)
        except Exception:
          put_bool_checked(ui_state.params, "TorqueInterceptorEnableRequest" if enabled else "TorqueInterceptorEnabled", not enabled)
          raise
      except Exception:
        cloudlog.exception("Failed to stage Torque Interceptor change")
        gui_app.push_widget(alert_dialog(tr("Failed to save Torque Interceptor change. Please try again.")))
      finally:
        self._restore_toggle()

    message = tr(TI_CONFIRMATION) if enabled else tr("Disabling Torque Interceptor requires a reboot. Reboot now?")
    gui_app.push_widget(ConfirmDialog(message, tr("Reboot"), callback=handle_reboot))

  def update_settings(self):
    self.ti_toggle.action_item.set_enabled(ui_state.is_offroad())
    pending = ui_state.params.get_bool("TorqueInterceptorEnableRequest")
    self.ti_toggle.set_description(tr(TI_PENDING_DESCRIPTION if pending else TI_DESCRIPTION))
