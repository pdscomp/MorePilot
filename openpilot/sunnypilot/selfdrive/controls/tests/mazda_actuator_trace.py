"""Contract-B differential trace driver (rebase plan 2026-10-06, section B).

Boots real Mazdas through `get_car` with mocked CAN I/O and cached FW, applies both setup
layers (opendbc's `_initialize_mazda` via get_car, then the parent's `setup_interfaces`) plus
repeated CP-aware retune calls, then records: tune values, semantic flags, controller/ti
params, speed-dep tables, scale consumers, and integer wire counts — CAM_LKAS 0x243 companion
and CAM_LKAS2 0x249 TI — across the contract's vEgo sweep.

Every number comes from the tree's own get_car / CarInterface / LatControlTorque (v0, the
file is tree-identical) / CarController; nothing here reimplements controller logic. The
lateral loop's demand generation uses the toy VM from test_latcontrol_torque_v2.py; the
carcontroller CS uses the SimpleNamespace pattern from opendbc's mazda controller tests.

Runs against whichever tree PYTHONPATH points at. The OLD expected traces are generated from
the untouched zoom-more worktree in an isolated process (old root abbreviated as $OLD):

  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$OLD/msgq_repo:$OLD/opendbc_repo:$OLD" \
  /home/paul/sandbox/zoompilot/.venv/bin/python \\
    /home/paul/sandbox/zoompilot-worktrees/zoom-more-rebase-20261006/openpilot/sunnypilot/selfdrive/controls/tests/mazda_actuator_trace.py \\
    mazda_actuator_units_old_traces.json
"""

import math
import sys
from types import SimpleNamespace

from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import get_car
from opendbc.car.mazda.values import MazdaFlags, MazdaSafetyFlags
from opendbc.sunnypilot.car import interfaces as sp_interfaces
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.car import interfaces as sp_car_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import LIVE_TORQUE_PARAMETERS_SP_SERVICE

# Contract-B sweep: the TI standstill-zero boundary (0.99/1.0), seed bin centers (6.5, 9.5,
# 21, 35), the stepped companion-scale step (14.2/14.5), and standstill itself (0).
VEGO_SWEEP = (0.0, 0.99, 1.0, 6.5, 9.5, 14.2, 14.5, 21.0, 35.0)
SCRIPTED_DEMANDS = (0.95, 0.3, -0.3, -0.95)  # normalized; 0.95*600 stays inside the TI rail
DRIVEN_SPEEDS = (1.0, 6.5, 14.2, 21.0, 35.0)  # lac-driven points (demand ~ v^2, skip ~zero)
SCRIPTED_FRAMES = 150  # worst-case ramp: 1140 counts at 12/frame
DRIVEN_FRAMES = 200  # 100-frame request buffer + settle
DESIRED_CURVATURE = 0.001  # rad/m; ~0.7 normalized at 35 m/s on the raw tune

# EPS firmware blobs: the donor 2022+ EPS (steer-to-zero) and a legacy-floor EPS.
KSD5_EPS_FW = b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
LEGACY_EPS_FW = b'K319-3210X-B-00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
TEST_VIN = "JM3KFBCL6L0123456"  # JM3/KF/L: a 2020 CX-5, the swapped car's body

# Toy steering geometry from test_latcontrol_torque_v2.py: curvature proportional to angle.
CURV_PER_DEG = 2e-4
VM = SimpleNamespace(calc_curvature=lambda angle_rad, v_ego, roll: math.degrees(angle_rad) * CURV_PER_DEG)
LP = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)

# Learned-bins inputs: the OLD seven centers with distinctive learned values, all-valid and
# all-invalid (the invalid path must fall back to the ti_legacy seeds).
LEARNED_CENTERS = [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]
LEARNED_LAF = [2.91, 3.13, 2.63, 1.51, 1.47, 1.83, 2.01]
LEARNED_FRIC = [0.191, 0.162, 0.141, 0.172, 0.151, 0.131, 0.112]
LEARNED_GLOBAL_LAF = 2.05
LEARNED_GLOBAL_FRIC = 0.147

SCENARIOS = {
  # Paul's car: 2020 body, donor EPS, TI.
  'swapped_cx5_ti': {'fingerprint': 'MAZDA_CX5', 'eps_fw': KSD5_EPS_FW, 'ti': True},
  # Same car with the manual override and NNLC switches on.
  'swapped_cx5_ti_manual_nnlc': {'fingerprint': 'MAZDA_CX5', 'eps_fw': KSD5_EPS_FW, 'ti': True, 'manual_nnlc': True},
  # Native 2022 body with an explicit TI: OLD seven-bin table, seed version 1.
  'cx5_2022_ti': {'fingerprint': 'MAZDA_CX5_2022', 'eps_fw': KSD5_EPS_FW, 'ti': True, 'learned': True},
  # Legacy-floor EPS + TI: OLD companion stays on the stock 800-count envelope.
  'cx9_2021_legacy_ti': {'fingerprint': 'MAZDA_CX9_2021', 'eps_fw': LEGACY_EPS_FW, 'ti': True},
  # Owner's CX-8: TI is intrinsic to the platform, no toggle.
  'cx8_2022_ti': {'fingerprint': 'MAZDA_CX8_2022', 'eps_fw': LEGACY_EPS_FW, 'ti': False},
  # Native cohorts (contract-B intentionally changed side; asserted against target semantics).
  'native_cx5_2022': {'fingerprint': 'MAZDA_CX5_2022', 'eps_fw': KSD5_EPS_FW, 'ti': False},
  'native_swapped_cx5': {'fingerprint': 'MAZDA_CX5', 'eps_fw': KSD5_EPS_FW, 'ti': False},
  'native_legacy_cx5': {'fingerprint': 'MAZDA_CX5', 'eps_fw': LEGACY_EPS_FW, 'ti': False},
  # Learned bins on the OLD-unlisted body: the speed-dep config is {} and bins fall back to
  # the global filtered values on both trees.
  'swapped_cx5_ti_learned': {'fingerprint': 'MAZDA_CX5', 'eps_fw': KSD5_EPS_FW, 'ti': True, 'learned': True},
}


def _can_recv(wait_for_one=False):
  return [[]]  # one empty packet per call; can_fingerprint walks its frame counter out


def _can_send(msgs):
  pass


def _cached_params(eps_fw):
  cp = structs.CarParams.new_message()
  cp.carVin = TEST_VIN
  cp.init('carFw', 1)
  fw = cp.carFw[0]
  fw.ecu = structs.CarParams.Ecu.eps
  fw.fwVersion = eps_fw
  fw.address = 0x730
  fw.subAddress = 0
  fw.brand = 'mazda'
  return cp


def _tune(CP):
  t = CP.lateralTuning.torque
  return {'laf': float(t.latAccelFactor), 'friction': float(t.friction), 'offset': float(t.latAccelOffset)}


def _jsonable(obj):
  if isinstance(obj, dict):
    return {str(k): _jsonable(v) for k, v in sorted(obj.items(), key=lambda kv: str(kv[0]))}
  if isinstance(obj, (list, tuple)):
    return [_jsonable(v) for v in obj]
  if isinstance(obj, (bool, int, float, str)) or obj is None:
    return obj
  raise TypeError(f"not trace-safe: {type(obj)}")


def _counts(dat):
  # CAM_LKAS/CAM_LKAS2 LKAS_REQUEST, as the port's own controller tests decode it.
  return (((dat[0] & 0xF) << 8) | dat[1]) - 2048


def _controller_state(v, driver_torque=0.0):
  out = structs.CarState()
  out.canValid = True
  out.vEgoRaw = v
  out.vEgo = v
  out.standstill = v < 0.01
  out.steeringTorque = driver_torque
  out.steeringPressed = False
  out.gasPressed = False
  out.brakePressed = False
  out.cruiseState.available = False
  out.cruiseState.enabled = False
  return SimpleNamespace(
    out=out,
    ti_lkas_allowed=True,  # healthy TI feedback: RUN state, fresh
    lkas_rejected=0,
    steer_undelivered=False,
    steer_first_engage_hold=False,
    crz_btns_counter=0,
    cancel_button=0,
    resume_button=0,
    accel_button=0,
    decel_button=0,
    tja_button=0,
    mrcc_button=0,
    distance_button=0,
    mrcc_armed_raw=False,
    radar_handback_active=False,
    radar_was_silenced=False,
    lkas_allowed_speed=True,
    lkas_setting_on=False,
    cam_lkas={"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0},
    cam_laneinfo=dict.fromkeys(("LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "ERR_BIT", "S1", "S1_HBEAM"), 0),
    cam_laneinfo_raw=None,
    cam_laneinfo_live=False,
  )


def _control(demand):
  cc = structs.CarControl()
  cc.enabled = True
  cc.latActive = True
  cc.actuators.torque = demand
  return cc.as_reader()


def _wire_point(CI, v, demand, frames):
  """Fresh interface (fresh controller state), one constant demand, settled. Records the last
  frame's 0x249/0x243 counts and bytes plus the reported actuators."""
  CI = type(CI)(CI.CP, CI.CP_SP)
  cc, cc_sp = _control(demand), structs.CarControlSP()
  ti_counts = []
  record = None
  for f in range(frames):
    actuators, sends = CI.CC.update(cc, cc_sp, _controller_state(v), f * 10_000_000)
    by_addr = {addr: dat for addr, dat, _bus in sends}
    ti_dat = by_addr.get(0x249)
    if ti_dat is not None:
      ti_counts.append(_counts(ti_dat))
    record = (by_addr, actuators)
  if ti_counts:  # native scenarios send no 0x249; a broken TI path shows as a trace diff instead
    assert len(set(ti_counts[-3:])) == 1, f"wire counts not settled: {ti_counts[-6:]}"
  by_addr, actuators = record
  cam_dat = by_addr[0x243]
  return {
    'ti': None if 0x249 not in by_addr else _counts(by_addr[0x249]),
    'ti_hex': None if 0x249 not in by_addr else by_addr[0x249].hex(),
    'cam': _counts(cam_dat),
    'cam_hex': cam_dat.hex(),
    'rep_torque': float(actuators.torque),
    'rep_counts': int(actuators.torqueOutputCan),
  }


def _lat_cs(v):
  # steeringAngleDeg chosen so the toy VM's measured curvature equals the desired one.
  angle = 0.0 if v == 0 else -DESIRED_CURVATURE / CURV_PER_DEG
  return SimpleNamespace(vEgo=v, aEgo=0.0, steeringAngleDeg=angle, steeringRateDeg=0.0, steeringPressed=False)


def _model_v2():
  msg = messaging.new_message('modelV2')
  n = len(ModelConstants.T_IDXS)
  msg.modelV2.orientation.x = [0.0] * n
  msg.modelV2.orientation.y = [0.0] * n
  msg.modelV2.acceleration.y = [0.0] * n
  return msg.modelV2


def _make_lac(CI, nnlc_live):
  lac = LatControlTorqueV0(CI.CP.as_reader(), CI.CP_SP, CI, DT_CTRL)
  if nnlc_live:
    lac.extension.update_model_v2(_model_v2())
  return lac


def _driven_point(CI, lac, v, frames=DRIVEN_FRAMES):
  cs = _lat_cs(v)
  lat_delay = float(CI.CP.steerActuatorDelay) + 0.2
  demand = 0.0
  for _ in range(frames):
    steer, _, _ = lac.update(True, cs, VM, LP, False, DESIRED_CURVATURE, None, False, lat_delay)
    demand = float(steer)
  return lac, demand


def _learned_msg(valid):
  tp = messaging.new_message('lateralTorqueParameters').lateralTorqueParameters
  tp.useParams = True
  tp.latAccelFactorFiltered = LEARNED_GLOBAL_LAF
  tp.frictionCoefficientFiltered = LEARNED_GLOBAL_FRIC
  tp.latAccelOffsetFiltered = 0.0
  tp_sp = getattr(messaging.new_message(LIVE_TORQUE_PARAMETERS_SP_SERVICE), LIVE_TORQUE_PARAMETERS_SP_SERVICE)
  tp_sp.speedBinCenters = list(LEARNED_CENTERS)
  tp_sp.speedBinLatAccelFactors = list(LEARNED_LAF)
  tp_sp.speedBinFrictions = list(LEARNED_FRIC)
  tp_sp.speedBinValid = list(valid)
  return tp, tp_sp


def run_scenario(name):
  spec = SCENARIOS[name]
  with OpenpilotPrefix():
    params = Params()
    # block=True: the async (non-blocking) put path needs the params dir to already exist.
    params.put_bool("EnforceTorqueControl", True, block=True)  # explicit: the retune pass must run on both trees
    if spec['ti']:
      params.put_bool("TorqueInterceptorEnabled", True, block=True)
    if spec.get('manual_nnlc'):
      params.put_bool("TorqueParamsOverrideEnabled", True, block=True)
      params.put("TorqueParamsOverrideLatAccelFactor", 1.9, block=True)  # not the defaults, so scaling shows
      params.put("TorqueParamsOverrideFriction", 0.12, block=True)
      params.put_bool("NeuralNetworkLateralControl", True, block=True)

    CI = get_car(
      _can_recv,
      _can_send,
      lambda _on: None,
      False,
      False,
      cached_params=_cached_params(spec['eps_fw']),
      fixed_fingerprint=spec['fingerprint'],
      init_params_list_sp=sp_car_interfaces.initialize_params(params),
    )
    assert str(CI.CP.carFingerprint) == spec['fingerprint']

    tunes = [_tune(CI.CP)]  # after get_car (opendbc layer ran inside)
    sp_car_interfaces.setup_interfaces(CI, params)
    tunes.append(_tune(CI.CP))  # after the parent's setup layer
    for _ in range(2):  # second and third retune must not rescale
      sp_car_interfaces._initialize_torque_lateral_control(CI, CI.CP, True, False)
      tunes.append(_tune(CI.CP))

    CP = CI.CP
    safety_param = int(CP.safetyConfigs[0].safetyParam)
    ccp, tip = CI.CC.params, CI.CC.ti_params
    tune_scale = getattr(sp_interfaces, 'get_tune_scale', None)

    trace = {
      'flags': {
        # Semantic bits only, read through each tree's own enum: the v1->v2 flag renumbering
        # (contract A) is the migration tests' surface, not this module's.
        'gen1': bool(CP.flags & MazdaFlags.GEN1),
        'stz': bool(CP.flags & MazdaFlags.STEER_TO_ZERO_EPS),
        'legacy_fw': bool(CP.flags & MazdaFlags.LEGACY_FW_EPS),
        'ti': bool(CP.flags & MazdaFlags.TORQUE_INTERCEPTOR),
        'min_steer_speed': float(CP.minSteerSpeed),
        'steer_at_standstill': bool(CP.steerAtStandstill),
        'safety_ti': bool(safety_param & MazdaSafetyFlags.TORQUE_INTERCEPTOR),
      },
      'tunes': tunes,
      'controller_params': {
        'steer_max': int(ccp.STEER_MAX),
        'steer_max_lookup': _jsonable(getattr(ccp, 'STEER_MAX_LOOKUP', None)),
        'delta_up': int(ccp.STEER_DELTA_UP),
        'delta_down': int(ccp.STEER_DELTA_DOWN),
        'driver_multiplier': int(ccp.STEER_DRIVER_MULTIPLIER),
        'eps_ceiling': _jsonable(getattr(ccp, 'EPS_CEILING_LOOKUP', None)),
        'ti_steer_max': int(tip.STEER_MAX),
        'ti_delta_up': int(tip.STEER_DELTA_UP),
        'ti_delta_down': int(tip.STEER_DELTA_DOWN),
        'ti_rt_delta': int(tip.STEER_MAX_RT_DELTA),
        'ti_rt_interval_ns': int(tip.STEER_RT_INTERVAL_NS),
        'ti_standstill_zero_speed': float(tip.STANDSTILL_ZERO_SPEED),
      },
      'speed_dep': _jsonable(sp_interfaces.get_speed_dep_config_for_car(CP)),
      'tune_scale': None if tune_scale is None else float(tune_scale(CP)),
      'wire_scripted': {(v, d): _wire_point(CI, v, d, SCRIPTED_FRAMES) for v in VEGO_SWEEP for d in SCRIPTED_DEMANDS},
    }

    if spec.get('manual_nnlc'):
      lac = _make_lac(CI, nnlc_live=True)
      tp = lac.torque_params
      lac.update(True, _lat_cs(6.5), VM, LP, False, DESIRED_CURVATURE, None, False, 0.34)
      trace['override_written'] = {'laf': float(tp.latAccelFactor), 'friction': float(tp.friction)}
      trace['nnlc_scale'] = float(getattr(lac.extension, '_nn_torque_scale', 1.0))  # OLD: no scale, i.e. 1.0
      driven = {}
      for v in DRIVEN_SPEEDS:
        lac, demand = _driven_point(CI, _make_lac(CI, nnlc_live=True), v)
        driven[v] = {'demand': demand, **_wire_point(CI, v, demand, SCRIPTED_FRAMES)}
      trace['lac_driven'] = driven

    if spec.get('learned'):
      learned = {}
      for mode, valid in (('valid', [True] * len(LEARNED_CENTERS)), ('invalid', [False] * len(LEARNED_CENTERS))):
        per_speed = {}
        for v in VEGO_SWEEP:
          lac = _make_lac(CI, nnlc_live=False)
          lac.extension.update_speed_dep_torque(*_learned_msg(valid))
          lac, demand = _driven_point(CI, lac, v)
          per_speed[v] = {
            'laf': float(lac.torque_params.latAccelFactor),
            'friction': float(lac.torque_params.friction),
            'demand': demand,
            **_wire_point(CI, v, demand, SCRIPTED_FRAMES),
          }
        learned[mode] = per_speed
      trace['learned'] = learned

    return trace


def run_all():
  return {name: run_scenario(name) for name in SCENARIOS}


def _wire_key(v, d):
  return f"{v}|{d}"


def _unwire_key(key):
  v, d = key.split('|')
  return float(v), float(d)


def dump(traces, path):
  """JSON fixture form: tuple/float keys become strings; floats round-trip exactly."""
  import json

  out = {}
  for name, trace in traces.items():
    t = dict(trace)
    t['wire_scripted'] = {_wire_key(v, d): r for (v, d), r in trace['wire_scripted'].items()}
    for sub in ('lac_driven',):
      if sub in t:
        t[sub] = {str(v): r for v, r in trace[sub].items()}
    if 'learned' in t:
      t['learned'] = {mode: {str(v): r for v, r in per.items()} for mode, per in trace['learned'].items()}
    out[name] = t
  with open(path, 'w') as f:
    json.dump(out, f, indent=1, sort_keys=True)
    f.write('\n')


def load(path):
  """Inverse of dump(): the exact key/value types run_scenario produces."""
  import json

  with open(path) as f:
    raw = json.load(f)
  for trace in raw.values():
    trace['wire_scripted'] = {_unwire_key(k): r for k, r in trace['wire_scripted'].items()}
    if 'lac_driven' in trace:
      trace['lac_driven'] = {float(v): r for v, r in trace['lac_driven'].items()}
    if 'learned' in trace:
      trace['learned'] = {mode: {float(v): r for v, r in per.items()} for mode, per in trace['learned'].items()}
  return raw


if __name__ == '__main__':
  import opendbc.car.mazda.values as _check

  print(f"# tree under test: {_check.__file__}", file=sys.stderr)
  dump(run_all(), sys.argv[1])
