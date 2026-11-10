#!/usr/bin/env python3
"""Adapt ZLIO's rigid virtual sensor odometry to the navigation base frame."""

import copy
import signal
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def quaternion(q):
    a = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(a)
    if not np.isfinite(norm) or norm < 1.e-12:
        raise ValueError('invalid rotation quaternion')
    return a / norm


def rotation(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def multiply(a, b):
    av, aw, bv, bw = a[:3], a[3], b[:3], b[3]
    return np.r_[aw*bv + bw*av + np.cross(av, bv), aw*bw - av.dot(bv)]


def vector(v):
    return np.array([v.x, v.y, v.z], dtype=float)


def assign(v, values):
    v.x, v.y, v.z = (float(x) for x in values)


def convert(msg, sensor_from_base, base_frame):
    """Convert pose, rigid-body twist and their covariances at one timestamp.

    sensor_from_base must be a rigid transform: moving gimbal frames require
    relative joint velocity and must first be compensated into a fixed frame.
    Pose covariance is ZLIO's world-frame position and left rotation error;
    twist covariance is expressed at the sensor origin in its local frame.
    """
    pose = msg.pose.pose
    q_os = quaternion(pose.orientation)
    q_sb = quaternion(sensor_from_base.rotation)
    r_os, r_sb = rotation(q_os), rotation(q_sb)
    t_sb = vector(sensor_from_base.translation)
    p_os = vector(pose.position)
    v_s, w_s = vector(msg.twist.twist.linear), vector(msg.twist.twist.angular)
    pose_cov = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
    twist_cov = np.asarray(msg.twist.covariance, dtype=float).reshape(6, 6)
    if not all(np.isfinite(x).all() for x in (t_sb, p_os, v_s, w_s, pose_cov, twist_cov)):
        raise ValueError('non-finite odometry or transform')

    lever_o = r_os @ t_sb
    q_ob = multiply(q_os, q_sb)
    out = copy.deepcopy(msg)
    out.child_frame_id = base_frame
    assign(out.pose.pose.position, p_os + lever_o)
    out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = (float(x) for x in q_ob)
    r_bs = r_sb.T
    assign(out.twist.twist.linear, r_bs @ (v_s + np.cross(w_s, t_sb)))
    assign(out.twist.twist.angular, r_bs @ w_s)

    pose_j = np.eye(6)
    pose_j[:3, 3:] = -skew(lever_o)
    twist_j = np.zeros((6, 6))
    twist_j[:3, :3] = r_bs
    twist_j[:3, 3:] = -r_bs @ skew(t_sb)
    twist_j[3:, 3:] = r_bs
    out.pose.covariance = (pose_j @ pose_cov @ pose_j.T).reshape(-1).tolist()
    out.twist.covariance = (twist_j @ twist_cov @ twist_j.T).reshape(-1).tolist()
    return out


class FrameCloudBridge:
    """Expose local deskewed returns with their original stamp and payload.

    In this opt-in rigid-sensor contract, ZLIO's `imu` coordinates are the
    preprocessor's fixed virtual sensor (`lio_base_sensor`). Only that frame
    name changes; TF performs the subsequent sensor-to-base transformation.
    """

    def __init__(self, sensor_frame, source_child, odom_frame, timeout):
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError('cloud timeout must be finite and positive')
        self.sensor_frame, self.source_child, self.odom_frame = sensor_frame, source_child, odom_frame
        self.timeout = timeout
        self.identity, self.healthy = None, False
        self.last_sequence = self.last_stamp = 0

    def session(self, msg):
        identity = (msg.instance_id, msg.epoch)
        if self.identity and identity[0] == self.identity[0] and identity[1] < self.identity[1]:
            return
        if identity != self.identity:
            self.last_sequence = self.last_stamp = 0
        self.identity = identity
        self.healthy = bool(msg.instance_id and not msg.reset_required and not msg.failed)

    def convert(self, frame, now_ns):
        cloud, odom = frame.cloud, frame.odometry
        if (not self.healthy or self.identity != (frame.instance_id, frame.epoch)
                or frame.sequence <= self.last_sequence or not frame.lidar_updated):
            raise ValueError('inactive frontend identity, repeated sequence or propagation-only frame')
        stamp = odom.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if (odom.header.frame_id != self.odom_frame or odom.child_frame_id != self.source_child
                or cloud.header.frame_id != self.source_child or cloud.header.stamp != stamp
                or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000
                or stamp_ns <= self.last_stamp or not -50_000_000 <= now_ns-stamp_ns <= self.timeout*1.e9):
            raise ValueError('invalid frame contract or stale/nonadvancing cloud timestamp')
        quaternion(odom.pose.pose.orientation)
        if not np.isfinite(vector(odom.pose.pose.position)).all():
            raise ValueError('nonfinite frontend pose')
        if (not cloud.width or not cloud.height or cloud.point_step <= 0
                or cloud.row_step < cloud.width*cloud.point_step
                or len(cloud.data) != cloud.row_step*cloud.height):
            raise ValueError('invalid point cloud layout')
        for name in ('x', 'y', 'z'):
            matches = [f for f in cloud.fields if f.name == name]
            if (len(matches) != 1 or matches[0].datatype != PointField.FLOAT32
                    or matches[0].count != 1 or matches[0].offset+4 > cloud.point_step):
                raise ValueError('invalid XYZ point field')
            values = np.ndarray((cloud.height, cloud.width), dtype='>f4' if cloud.is_bigendian else '<f4',
                                buffer=cloud.data, offset=matches[0].offset,
                                strides=(cloud.row_step, cloud.point_step))
            if not np.isfinite(values).all():
                raise ValueError('nonfinite point coordinates')
        result = copy.deepcopy(cloud)
        result.header.frame_id = self.sensor_frame
        self.last_sequence, self.last_stamp = frame.sequence, stamp_ns
        return result


class BaseOdometry(Node):
    def __init__(self):
        super().__init__('zlio_base_odometry')
        self.sensor_frame = self.declare_parameter('sensor_frame', 'lio_base_sensor').value
        self.source_child = self.declare_parameter('source_child_frame', 'imu').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        input_topic = self.declare_parameter('input_topic', '/odometry').value
        output_topic = self.declare_parameter('output_topic', '/odom').value
        if not all((self.sensor_frame, self.source_child, self.odom_frame, self.base_frame)):
            raise ValueError('frame names must be nonempty')
        if input_topic == output_topic:
            raise ValueError('input and output topics must differ')
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(Odometry, output_topic, 50)
        self.subscription = self.create_subscription(Odometry, input_topic, self.on_odom, 100)
        publish_cloud = self.declare_parameter('publish_deskewed_cloud', False).value
        frame_topic = self.declare_parameter('frontend_frame_topic', '/frontend/frame').value
        session_topic = self.declare_parameter('frontend_session_topic', '/frontend/session').value
        cloud_topic = self.declare_parameter('deskewed_cloud_topic', '/cloud_deskewed').value
        cloud_timeout = self.declare_parameter('cloud_timeout', .5).value
        if publish_cloud:
            from localization_interfaces.msg import FrontendFrame, FrontendSession
            self.cloud_bridge = FrameCloudBridge(self.sensor_frame, self.source_child, self.odom_frame, cloud_timeout)
            self.cloud_publisher = self.create_publisher(PointCloud2, cloud_topic, 10)
            self.session_subscription = self.create_subscription(
                FrontendSession, session_topic, self.cloud_bridge.session,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.frame_subscription = self.create_subscription(FrontendFrame, frame_topic, self.on_frame, 10)

    def on_frame(self, msg):
        try:
            cloud = self.cloud_bridge.convert(msg, self.get_clock().now().nanoseconds)
        except ValueError as exc:
            self.get_logger().warning(f'Cannot expose ZLIO navigation cloud: {exc}', throttle_duration_sec=2.)
            return
        self.cloud_publisher.publish(cloud)

    def on_odom(self, msg):
        if msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.source_child:
            self.get_logger().warning('Rejecting unexpected ZLIO odometry frames', throttle_duration_sec=2.)
            return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.get_logger().warning('Rejecting zero odometry timestamp', throttle_duration_sec=2.)
            return
        try:
            tf = self.buffer.lookup_transform(self.sensor_frame, self.base_frame,
                                              Time.from_msg(msg.header.stamp))
            out = convert(msg, tf.transform, self.base_frame)
        except (TransformException, ValueError) as exc:
            self.get_logger().warning(f'Cannot convert sensor odometry to base: {exc}',
                                      throttle_duration_sec=2.)
            return
        t = TransformStamped()
        t.header = out.header
        t.child_frame_id = out.child_frame_id
        t.transform.translation.x = out.pose.pose.position.x
        t.transform.translation.y = out.pose.pose.position.y
        t.transform.translation.z = out.pose.pose.position.z
        t.transform.rotation = out.pose.pose.orientation
        self.broadcaster.sendTransform(t)
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = BaseOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
