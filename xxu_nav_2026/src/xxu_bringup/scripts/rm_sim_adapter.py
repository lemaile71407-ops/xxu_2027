#!/usr/bin/env python3
"""Forward safe velocity and stabilize the upstream simulator gimbal."""

import math
import time
from xxu_bringup.interfaces import interfaces

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def velocity_is_valid(message, now, received_at, monotonic_now):
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    values = (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
              message.twist.angular.x, message.twist.angular.y, message.twist.angular.z)
    return (0 <= now-stamp <= 0.3 and 0 <= monotonic_now-received_at <= 0.3
            and all(math.isfinite(value) for value in values))


class RmSimAdapter(Node):
    def __init__(self):
        super().__init__('rm_sim_adapter')
        contract = interfaces()
        self.publisher = self.create_publisher(Float64, contract.topic('simulation.gimbal_big_yaw'), 10)
        self.velocity_publisher = self.create_publisher(Twist, contract.topic('navigation.chassis_input'), 10)
        self.create_subscription(TwistStamped, contract.topic('simulation.safe_velocity'),
                                 self.velocity, 10)
        self.command = None
        self.received_at = 0.0
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.01, self.publish_velocity, clock=self.steady_clock)
        self.position = None
        self.last_yaw = None
        self.target = None
        self.create_subscription(JointState, contract.topic('gimbal.joint_states'), self.joints, 20)
        self.create_subscription(Odometry, contract.topic('localization.odometry'), self.odometry, 20)

    def velocity(self, message):
        self.command = message
        self.received_at = time.monotonic()

    def publish_velocity(self):
        output = Twist()
        if self.command is not None and velocity_is_valid(
                self.command, self.get_clock().now().nanoseconds*1e-9,
                self.received_at, time.monotonic()):
            output = self.command.twist
        self.velocity_publisher.publish(output)

    def joints(self, message):
        if 'big_yaw_joint' in message.name:
            index = message.name.index('big_yaw_joint')
            if index < len(message.position) and math.isfinite(message.position[index]):
                self.position = message.position[index]

    def odometry(self, message):
        if self.position is None:
            return
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        if not math.isfinite(yaw):
            return
        if self.last_yaw is None:
            self.target = self.position
        else:
            delta = yaw - self.last_yaw
            self.target -= math.atan2(math.sin(delta), math.cos(delta))
        self.last_yaw = yaw
        self.publisher.publish(Float64(data=self.target))


def main():
    rclpy.init()
    node = RmSimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
