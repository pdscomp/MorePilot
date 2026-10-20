"""Tests for ICBM (non-pcmCruiseSpeed) cruise handling.

The setpoint-reconciliation and button-actuator servo scenarios upstream split out into
test_icbm_reconcile.py and test_icbm_servo.py (on the shared icbm_servo_harness) were
retired to those suites. What stays here:
- decel overshoot gating: cruise-source plans, non-Mazda brands, and the toggle all
  keep the lever off; a saturated plant is a no-op
- openpilot-longitudinal guardrails: the planner brakes for curves itself, so the servo
  keeps the dash on the driver's setpoint and the overshoot lever stays off
"""
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement


class TestDecelOvershoot:
  """Down-only overshoot: command the dash below the planner target so the stock ACC
  delivers the requested deceleration (its decel scales with dash-vs-vEgo gap)."""

  def make_icbm(self, brand="mazda"):
    return IntelligentCruiseButtonManagement(car.CarParams(pcmCruise=True, brand=brand),
                                             custom.CarParamsSP(pcmCruiseSpeed=False))

  def run_frames(self, icbm, target_mph, v_ego_mph, a_target, n=1, source='sccVision', enabled=True):
    for _ in range(n):
      CS = car.CarState(vEgo=v_ego_mph * CV.MPH_TO_MS,
                        cruiseState={"speedCluster": target_mph * CV.MPH_TO_MS})
      CC = car.CarControl(enabled=True)
      LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * CV.MPH_TO_MS, aTarget=a_target)
      LP_SP.longitudinalPlanSource = source
      LP_SP.smartCruiseControl.vision.aTarget = float(a_target)
      LP_SP.smartCruiseControl.map.aTarget = float(a_target)
      LP_SP.speedLimit.assist.aTarget = float(a_target)
      icbm.run(CS, CC, LP_SP, is_metric=False, decel_overshoot_enabled=enabled)

  def test_deep_dip_is_a_no_op(self):
    """When the target is already far below vEgo the plant is saturated; never go deeper."""
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=20, v_ego_mph=45, a_target=-1.0, n=100)
    assert icbm.v_target == 20, icbm.v_target

  def test_cruise_source_never_overshoots(self):
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100, source='cruise')
    assert icbm.v_target == 40, icbm.v_target

  def test_mazda_only(self):
    icbm = self.make_icbm(brand="hyundai")
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target == 40, icbm.v_target

  def test_toggle_off_disables_overshoot(self):
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100, enabled=False)
    assert icbm.v_target == 40, icbm.v_target


class TestOpLongGuardrails:
  """Under openpilot longitudinal the planner brakes for curves itself: the servo keeps
  the dash on the driver's setpoint and the overshoot lever stays off."""

  def make_icbm(self, op_long):
    return IntelligentCruiseButtonManagement(car.CarParams(pcmCruise=True, brand="mazda",
                                                           openpilotLongitudinalControl=op_long),
                                             custom.CarParamsSP(pcmCruiseSpeed=False))

  def run_frames(self, icbm, target_mph, v_ego_mph, a_target, n=1, source='sccVision'):
    for _ in range(n):
      CS = car.CarState(vEgo=v_ego_mph * CV.MPH_TO_MS, vCruise=60 * CV.MPH_TO_KPH,
                        cruiseState={"speedCluster": 60 * CV.MPH_TO_MS})
      CC = car.CarControl(enabled=True)
      LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * CV.MPH_TO_MS, aTarget=float(a_target))
      LP_SP.longitudinalPlanSource = source
      icbm.run(CS, CC, LP_SP, is_metric=False, decel_overshoot_enabled=True)

  def test_dash_ignores_curve_target_under_op_long(self):
    icbm = self.make_icbm(op_long=True)
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target == 60  # the driver's setpoint, never the 40 mph curve target

  def test_overshoot_lever_off_under_op_long(self):
    icbm = self.make_icbm(op_long=True)
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.overshoot_mph == 0.0

  def test_stock_cruise_still_walks_the_curve(self):
    icbm = self.make_icbm(op_long=False)
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target <= 40  # unchanged non-OP-long behavior: the dash follows the plan
