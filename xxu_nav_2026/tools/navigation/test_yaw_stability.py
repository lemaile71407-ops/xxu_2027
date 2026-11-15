"""Protect the acceptance check from missing evidence and false stable readings."""
import copy
import math
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from navigation_evidence import localization_stability_evidence, stationary_velocity_evidence
from check_yaw_stability import apply_sample_noise_profile


class StabilityEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.rows = [{'clock': 10 + i * .1, 'map_stamp': 10 + i * .1 - .02,
                      'truth_stamp': 10 + i * .1, 'trusted': True,
                      'map': [2., 2., math.pi - .001], 'truth': [2., 2., math.pi]}
                     for i in range(101)]

    def check(self, rows):
        return localization_stability_evidence(rows, 10., 20.)['passed']

    def test_stationary_and_wrapped_yaw_pass(self):
        self.rows[50]['map'][2] = -math.pi + .001
        self.assertTrue(self.check(self.rows))

    def test_missing_stale_frozen_and_nonfinite_tf_fail(self):
        for kind in ('missing', 'stale', 'frozen', 'backwards', 'nan'):
            rows = copy.deepcopy(self.rows)
            if kind == 'missing':
                rows[20]['map'] = None
            elif kind == 'stale':
                rows[20]['map_stamp'] -= 1
            elif kind == 'frozen':
                for row in rows:
                    row['map_stamp'] = 9.99
            elif kind == 'backwards':
                rows[20]['map_stamp'] = rows[19]['map_stamp'] - .01
            else:
                rows[20]['map'][0] = math.nan
            with self.subTest(kind=kind):
                self.assertFalse(self.check(rows))

    def test_short_or_gapped_observation_fails(self):
        for rows in ([], self.rows[:70], self.rows[:30] + self.rows[45:]):
            self.assertFalse(self.check(rows))

    def test_untrusted_localization_with_advancing_frozen_pose_fails(self):
        self.rows[50]['trusted'] = False
        self.assertFalse(self.check(self.rows))

    def test_frozen_truth_cannot_certify_stationarity(self):
        for row in self.rows:
            row['truth_stamp'] = 10.
        self.assertFalse(self.check(self.rows))

    def test_drift_and_true_motion_fail(self):
        for field, index, amount in [('map', 0, .121), ('map', 2, .201),
                                      ('truth', 0, .006), ('truth', 2, .011)]:
            rows = copy.deepcopy(self.rows)
            rows[50][field][index] += amount
            self.assertFalse(self.check(rows))

    def test_stable_but_wrong_map_pose_fails(self):
        for row in self.rows:
            row['map'][0] += .13
        self.assertFalse(self.check(self.rows))

    def test_accepted_centimeter_difference_is_not_new_precision_tuning(self):
        self.rows[50]['map'][0] += .041
        self.assertTrue(self.check(self.rows))


class StationaryVelocityEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.history = {name: [(100 + i * .01, 10 + i * .01, [.001, 0., .0002])
                               for i in range(1001)] for name in ('source_odom', 'odom')}

    def check(self):
        return stationary_velocity_evidence(self.history, 10., 20.)['passed']

    def test_continuously_fresh_quiet_source_and_adapter_pass(self):
        self.assertTrue(self.check())

    def test_midwindow_source_spike_fails_even_when_final_parking_is_quiet(self):
        self.history['source_odom'][500][2][1] = .02077
        self.assertFalse(self.check())

    def test_old_parking_speed_limit_is_unchanged(self):
        self.history['odom'][500][2][:2] = [.02, 0.]
        self.assertTrue(self.check())
        self.history['odom'][500][2][0] = .02001
        self.assertFalse(self.check())

    def test_missing_frozen_gapped_and_nonfinite_evidence_fails(self):
        for mode in ('missing', 'frozen', 'gap', 'nan'):
            self.setUp()
            if mode == 'missing':
                self.history['source_odom'] = []
            elif mode == 'frozen':
                self.history['source_odom'] = [(100., 10., [.001, 0., 0.])] * 100
            elif mode == 'gap':
                del self.history['odom'][400:600]
            else:
                self.history['odom'][500][2][0] = math.nan
            with self.subTest(mode=mode):
                self.assertFalse(self.check())


class SampleNoiseProfileTest(unittest.TestCase):
    def test_only_two_sample_variances_change_without_mutating_capture_config(self):
        config = {'filter': {'gyro_variance': .25, 'accel_variance': .5,
                             'gyro_bias_psd': 1.33e-6, 'convergence_tolerance': .001},
                  'map': {'voxel_size': .5}}
        original = copy.deepcopy(config)
        changed = apply_sample_noise_profile(config, {'gyro_variance': 4e-8, 'accel_variance': 2.89e-4})
        self.assertEqual(config, original)
        self.assertEqual(changed['map'], original['map'])
        self.assertEqual(changed['filter']['gyro_bias_psd'], original['filter']['gyro_bias_psd'])
        self.assertEqual(changed['filter']['convergence_tolerance'], .001)
        self.assertEqual(changed['filter']['gyro_variance'], 4e-8)
        self.assertEqual(changed['filter']['accel_variance'], 2.89e-4)

    def test_invalid_values_and_unrelated_threshold_overrides_are_rejected(self):
        config = {'filter': {'gyro_variance': .25, 'accel_variance': .5}}
        for profile in ({'gyro_variance': .1}, {'gyro_variance': .1, 'accel_variance': .1, 'convergence_tolerance': .1},
                        *({'gyro_variance': value, 'accel_variance': .1} for value in [0, -1, math.nan, math.inf, True, '0.1'])):
            with self.subTest(profile=profile), self.assertRaises(ValueError):
                apply_sample_noise_profile(config, profile)

    def test_profile_matches_declared_simulator_sample_noise(self):
        directory = Path(__file__).resolve().parent
        profile = yaml.safe_load((directory / 'ega_imu_sample_noise.yaml').read_text())
        model = ET.parse(directory.parents[1] / 'src/xxu_sim_bringup/models/ega_folded/model.sdf')
        imu = model.find(".//sensor[@name='livox_lidar_housing_imu']/imu")
        self.assertIsNotNone(imu)
        for kind, key in [('angular_velocity', 'gyro_variance'), ('linear_acceleration', 'accel_variance')]:
            for axis in 'xyz':
                std = float(imu.find(f'{kind}/{axis}/noise/stddev').text)
                self.assertAlmostEqual(profile[key], std * std, places=14)


if __name__ == '__main__':
    unittest.main()
