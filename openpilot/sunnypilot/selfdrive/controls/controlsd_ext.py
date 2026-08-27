"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom

from opendbc.car import structs
from openpilot.common.params import ParamKeyFlag, Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_fixed_lat_delay
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.icbm_latch import icbm_active
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.locationd.speed_bin_learner import LIVE_TORQUE_PARAMETERS_SP_SERVICE
from openpilot.sunnypilot.selfdrive.controls.controls_lateral_zp import ControlsLateralZP
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral


# Drive-start params dump: every set key except DONT_LOG secrets and oversized blobs,
# published once on the first carControlSP of the drive so rlogs record the exact
# settings that produced them (CX-8 controller attribution took a day without this).
PARAM_DUMP_VALUE_CAP = 512


def build_param_dump(params: Params) -> list[dict]:
  dont_log = set(params.all_keys(flag=ParamKeyFlag.DONT_LOG))
  dump = []
  for key_raw in sorted(k for k in params.all_keys() if k is not None):
    if key_raw in dont_log:
      continue
    key = key_raw.decode()
    key_type = params.get_type(key)
    value = params.get(key)
    if value is None:
      continue
    raw = value if isinstance(value, bytes) else str(value).encode()
    if len(raw) > PARAM_DUMP_VALUE_CAP:
      continue
    # ParamKeyType ordinals match CarControlSP.ParamType exactly
    dump.append({"key": key, "type": int(key_type), "value": raw})
  return dump


def lateral_extension_of(lac):
  """Which torque-space extension currently owns the output, if any. v2's host
  disables the overrides, so this reads none there even with NNLC params set."""
  ext = getattr(lac, "extension", None)
  if ext is None or not ext.overrides_output:
    return custom.LateralExtension.none
  if ext._nnlc_enabled:
    return custom.LateralExtension.neuralNetwork
  if ext._jerk_aware_enabled:
    return custom.LateralExtension.jerkAware
  return custom.LateralExtension.none
from openpilot.sunnypilot.selfdrive.controls.lib.turn_assist import TurnAssistController


class ControlsExt(ModelStateBase, ControlsLateralZP):
  lagd_toggle: bool

  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self.lagd_toggle = CP.lateralTuning.which() == 'torque' and params.get_bool("LagdToggle")
    if CP.lateralTuning.which() == 'torque' and not self.lagd_toggle:
      self.lat_delay = get_fixed_lat_delay(params, CP.steerActuatorDelay)
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()
    self.turn_assist = TurnAssistController(CP)
    ControlsLateralZP.__init__(self, CP)

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP', 'carStateSP', LIVE_TORQUE_PARAMETERS_SP_SERVICE]
    self.pm_services_ext = ['carControlSP']
    self._param_dump_pending = True

  @property
  def pcm_cruise_speed(self) -> bool:
    """CarParamsSP.pcmCruiseSpeed as card has it this frame: False while ICBM drives the set speed."""
    return not icbm_active(self.sm['carStateSP'], self.CP_SP)
  def get_params_sp(self, sm: messaging.SubMaster) -> None:
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.blinker_pause_lateral.get_params()
      self.turn_assist.get_params()
      self.lane_change_smoothing.get_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lagd_toggle = self.params.get_bool("LagdToggle")
        if not self.lagd_toggle:
          self.lat_delay = get_fixed_lat_delay(self.params, self.CP.steerActuatorDelay)

      self._param_update_time = time.monotonic()

  def _get_lat_active(self, sm: messaging.SubMaster) -> bool:
    if self.blinker_pause_lateral.update(sm['carState']):
      return False

    ss_sp = sm['selfdriveStateSP']
    if ss_sp.mads.available:
      return bool(ss_sp.mads.active)

    # MADS not available, use stock state to engage
    return bool(sm['selfdriveState'].active)

  @staticmethod
  def get_lead_data(_lead, src: log.RadarState.LeadData) -> None:
    _lead.dRel = src.dRel
    _lead.yRel = src.yRel
    _lead.vRel = src.vRel
    _lead.aRel = src.deprecated.aRel
    _lead.vLead = src.vLead
    _lead.dPath = src.deprecated.dPath
    _lead.vLat = src.deprecated.vLat
    _lead.vLeadK = src.vLeadK
    _lead.aLeadK = src.aLeadK
    _lead.fcw = src.deprecated.fcw
    _lead.status = src.present
    _lead.aLeadTau = src.aLeadTau
    _lead.modelProb = src.modelProb
    _lead.radar = src.radar
    _lead.radarTrackId = src.radarTrackId

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    # Once per drive: the exact settings this log was produced under
    if self._param_dump_pending:
      CC_SP.params = build_param_dump(self.params)
      self._param_dump_pending = False
    CC_SP.zoompilot.lateralExtension = lateral_extension_of(getattr(self, "LaC", None))

    self.get_lead_data(CC_SP.leadOne, sm['radarState'].leadOne)
    self.get_lead_data(CC_SP.leadTwo, sm['radarState'].leadTwo)

    # MADS state
    mads_src = sm['selfdriveStateSP'].mads
    CC_SP.mads.state = mads_src.state
    CC_SP.mads.enabled = mads_src.enabled
    CC_SP.mads.active = mads_src.active
    CC_SP.mads.available = mads_src.available

    # ICBM state
    icbm_src = sm['selfdriveStateSP'].intelligentCruiseButtonManagement
    CC_SP.intelligentCruiseButtonManagement.state = icbm_src.state
    CC_SP.intelligentCruiseButtonManagement.sendButton = icbm_src.sendButton
    CC_SP.intelligentCruiseButtonManagement.vTarget = icbm_src.vTarget
    CC_SP.zoompilot.icbmHold = sm['selfdriveStateSP'].zoompilot.icbmHold

    # lateral assist telemetry, for offline validation of the turn hold and pace clamp
    CC_SP.turnAssist.holdCurvature = float(self.turn_assist.hold)
    CC_SP.turnAssist.leadCurvature = float(self.turn_assist.lead_applied)
    # lane-change pace clamp telemetry, for offline validation
    CC_SP.zoompilot.laneChangeSmoothing.jerkFactor = float(self.lane_change_smoothing.jerk_factor)

    return CC_SP

  @staticmethod
  def publish_ext(CC_SP: custom.CarControlSP, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = sm['carState'].canValid
    cc_sp_send.carControlSP = CC_SP

    pm.send('carControlSP', cc_sp_send)

  def run_ext(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    CC_SP = self.state_control_ext(sm)
    self.publish_ext(CC_SP, sm, pm)
    self.run_lateral_zp(sm)
