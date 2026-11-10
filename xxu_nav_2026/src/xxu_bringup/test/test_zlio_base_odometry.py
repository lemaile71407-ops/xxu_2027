import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import Mock

import numpy as np
import rclpy
from geometry_msgs.msg import Transform
from nav_msgs.msg import Odometry

path = Path(__file__).resolve().parents[1] / 'scripts' / 'zlio_base_odometry.py'
spec = importlib.util.spec_from_file_location('zlio_base_odometry', path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class ConversionTest(unittest.TestCase):
    def test_offset_rotation_velocity_and_covariance(self):
        m = Odometry()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'imu'
        m.pose.pose.position.x = 10.
        m.pose.pose.position.y = 20.
        m.pose.pose.orientation.z = math.sqrt(.5)
        m.pose.pose.orientation.w = math.sqrt(.5)
        m.twist.twist.linear.x = 1.
        m.twist.twist.angular.z = 2.
        m.pose.covariance = np.eye(6).reshape(-1).tolist()
        m.twist.covariance = np.eye(6).reshape(-1).tolist()
        t = Transform()
        t.translation.x = 1.
        t.rotation.z = math.sqrt(.5)
        t.rotation.w = math.sqrt(.5)
        out = mod.convert(m, t, 'base_footprint')
        np.testing.assert_allclose([out.pose.pose.position.x, out.pose.pose.position.y], [10., 21.], atol=1e-12)
        np.testing.assert_allclose([out.pose.pose.orientation.z, out.pose.pose.orientation.w], [1., 0.], atol=1e-12)
        np.testing.assert_allclose([out.twist.twist.linear.x, out.twist.twist.linear.y], [2., -1.], atol=1e-12)
        self.assertAlmostEqual(out.twist.twist.angular.z, 2.)
        self.assertEqual(out.child_frame_id, 'base_footprint')
        pose_cov = np.array(out.pose.covariance).reshape(6, 6)
        twist_cov = np.array(out.twist.covariance).reshape(6, 6)
        self.assertAlmostEqual(pose_cov[0, 0], 2.)
        self.assertAlmostEqual(pose_cov[0, 5], -1.)
        self.assertAlmostEqual(twist_cov[0, 0], 2.)
        self.assertAlmostEqual(twist_cov[0, 5], 1.)

    def test_invalid_data_is_rejected(self):
        m, t = Odometry(), Transform()
        t.rotation.w = 1.
        m.pose.pose.orientation.w = 0.
        with self.assertRaises(ValueError):
            mod.convert(m, t, 'base_footprint')
        m.pose.pose.orientation.w = 1.
        m.twist.twist.angular.z = float('nan')
        with self.assertRaises(ValueError):
            mod.convert(m, t, 'base_footprint')


class AdapterFailureTest(unittest.TestCase):
    def test_rejects_wrong_frames_and_missing_tf(self):
        rclpy.init()
        node = mod.BaseOdometry()
        node.publisher = Mock()
        node.broadcaster = Mock()
        try:
            self.assertIs(node.buffer.clock, node.get_clock())
            m = Odometry()
            m.header.frame_id = 'odom'
            m.child_frame_id = 'wrong_sensor'
            m.header.stamp.sec = 1
            node.on_odom(m)
            m.child_frame_id = 'imu'
            node.on_odom(m)  # No sensor-to-base transform in this buffer.
            node.publisher.publish.assert_not_called()
            node.broadcaster.sendTransform.assert_not_called()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    unittest.main()
