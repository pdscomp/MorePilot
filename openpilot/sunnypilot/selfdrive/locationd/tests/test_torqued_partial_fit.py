"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Partial-fit live learn: with self-tune on, the filtered torque params track the TLS fit once
calibration passes PARTIAL_FIT_MIN_CALPERC, before every steer bucket fills; valid stays False
until full validity. The Mazda floor override feeds the learner below upstream's 15 m/s.
"""
import numpy as np
import pytest

from openpilot.selfdrive.locationd.torqued import (
  MIN_VEL,
  MIN_VEL_MAZDA,
  PARTIAL_FIT_MIN_CALPERC,
  STEER_BUCKET_BOUNDS,
  TorqueEstimator,
)
from openpilot.sunnypilot.selfdrive.locationd.tests.speed_dep_helpers import (
  SPEED_DEP_FINGERPRINT,
  FakePubMaster,
  make_cp,
)

pytestmark = pytest.mark.skipif(SPEED_DEP_FINGERPRINT is None, reason="speed-dependent torque TOML is missing")

OFFLINE_LAF = 1.25
OFFLINE_FRICTION = 0.125
TRUE_LAF = 1.5

# 7 buckets at their minimum, one middle bucket at half: past PARTIAL_FIT_MIN_CALPERC
# (~57%) but short of validity.
COUNTS_PARTIAL = [100, 300, 500, 250, 500, 500, 300, 100]
COUNTS_SPARSE = [20] * 8


def seed_buckets(est, counts):
  """Noiseless points on a known slope so the TLS fit has one answer: latacc = TRUE_LAF * steer."""
  for (lo, hi), n in zip(STEER_BUCKET_BOUNDS, counts, strict=True):
    for x in np.linspace(lo, hi, n, endpoint=False):
      est.filtered_points.add_point(float(x), TRUE_LAF * float(x))


class TestMazdaVelFloor:
  def test_mazda_overrides_min_vel_with_self_tune(self, fake_params):
    est = TorqueEstimator(make_cp(brand='mazda', lat_accel_factor=OFFLINE_LAF, friction=OFFLINE_FRICTION))
    assert est.min_vel == MIN_VEL_MAZDA == 10

  def test_mazda_keeps_upstream_floor_without_self_tune(self, fake_params_off):
    est = TorqueEstimator(make_cp(brand='mazda', lat_accel_factor=OFFLINE_LAF, friction=OFFLINE_FRICTION))
    assert est.min_vel == MIN_VEL == 15

  def test_other_brand_keeps_upstream_floor(self, fake_params):
    est = TorqueEstimator(make_cp(brand='toyota', lat_accel_factor=OFFLINE_LAF, friction=OFFLINE_FRICTION))
    assert est.min_vel == MIN_VEL == 15


class TestPartialFit:
  def test_filters_track_fit_past_threshold_before_validity(self, fake_params):
    est = TorqueEstimator(make_cp(brand='mazda', lat_accel_factor=OFFLINE_LAF, friction=OFFLINE_FRICTION))
    est._pm = FakePubMaster()
    seed_buckets(est, COUNTS_PARTIAL)
    assert not est.filtered_points.is_valid()
    assert est.filtered_points.get_valid_percent() >= PARTIAL_FIT_MIN_CALPERC

    msg = est.get_msg().lateralTorqueParameters
    assert msg.valid is False
    assert msg.calPerc >= PARTIAL_FIT_MIN_CALPERC
    assert msg.latAccelFactorRaw == pytest.approx(TRUE_LAF, abs=0.01)
    # update_params runs, so the slow EMA steps toward the fit instead of freezing at the offline tune
    assert est.decay > 50
    assert OFFLINE_LAF < msg.latAccelFactorFiltered < TRUE_LAF
    # noiseless points fit ~zero spread, so friction slides off the offline value but stays sane
    assert msg.frictionCoefficientFiltered < OFFLINE_FRICTION

    for _ in range(100):
      msg = est.get_msg().lateralTorqueParameters
    assert msg.latAccelFactorFiltered > OFFLINE_LAF + 0.5 * (TRUE_LAF - OFFLINE_LAF) * 0.1

  def test_below_threshold_stays_frozen(self, fake_params):
    est = TorqueEstimator(make_cp(brand='mazda', lat_accel_factor=OFFLINE_LAF, friction=OFFLINE_FRICTION))
    est._pm = FakePubMaster()
    seed_buckets(est, COUNTS_SPARSE)
    assert est.filtered_points.get_valid_percent() < PARTIAL_FIT_MIN_CALPERC

    msg = est.get_msg().lateralTorqueParameters
    assert msg.valid is False
    assert msg.latAccelFactorFiltered == pytest.approx(OFFLINE_LAF, abs=0.01)
    assert msg.frictionCoefficientFiltered == pytest.approx(OFFLINE_FRICTION, abs=0.01)
    assert est.decay == 50

  def test_partial_fit_off_when_self_tune_off(self, fake_params_off):
    est = TorqueEstimator(make_cp(brand='mazda', lat_accel_factor=OFFLINE_LAF, friction=OFFLINE_FRICTION))
    est._pm = FakePubMaster()
    seed_buckets(est, COUNTS_PARTIAL)

    msg = est.get_msg().lateralTorqueParameters
    assert msg.valid is False
    assert msg.latAccelFactorFiltered == pytest.approx(OFFLINE_LAF, abs=0.01)
    assert est.decay == 50

  def test_partial_fit_off_on_upstream_brand_self_tune(self, fake_params_off):
    """Brand-based self-tune (stock Toyota/Hyundai path) is not the fork's toggle opt-in:
    partial-fit must stay off there and the velocity floor stays upstream's."""
    est = TorqueEstimator(make_cp(brand='toyota', lat_accel_factor=OFFLINE_LAF, friction=OFFLINE_FRICTION))
    est._pm = FakePubMaster()
    assert est.speed_binned  # brand self-tune is on
    seed_buckets(est, COUNTS_PARTIAL)

    msg = est.get_msg().lateralTorqueParameters
    assert msg.valid is False
    assert msg.latAccelFactorFiltered == pytest.approx(OFFLINE_LAF, abs=0.01)
    assert est.decay == 50
    assert est.min_vel == MIN_VEL
