#!/usr/bin/env python3
"""Run one low-rate mapping mission through Nav2 and the backend save service."""

import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger

from xxu_bringup.mapping_waypoints import load_waypoints_csv, yaw_to_quaternion


class WaypointMappingRunner(Node):
    def __init__(self):
        super().__init__('waypoint_mapping_runner')
        defaults = {
            'waypoints_file': '',
            'frame_id': 'map',
            'action_name': 'follow_waypoints',
            'save_map_service': '/backend_node/save_map',
            'auto_save_map': True,
            'save_on_partial': True,
            'initial_wait_sec': 3.0,
            'pre_save_wait_sec': 2.0,
            'server_wait_timeout_sec': 120.0,
            'save_timeout_sec': 30.0,
            'exit_on_complete': True,
        }
        self.config = {name: self.declare_parameter(name, value).value
                       for name, value in defaults.items()}
        self.current_waypoint = None

    def _spin_delay(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return True
            rclpy.spin_once(self, timeout_sec=min(remaining, 0.1))
        return False

    def _goal(self, waypoints):
        goal = FollowWaypoints.Goal()
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.config['frame_id']
            pose.header.stamp = Time().to_msg()
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            orientation = pose.pose.orientation
            orientation.x, orientation.y, orientation.z, orientation.w = yaw_to_quaternion(waypoint.yaw)
            goal.poses.append(pose)
        return goal

    def _feedback(self, message):
        index = message.feedback.current_waypoint
        if index != self.current_waypoint:
            self.current_waypoint = index
            self.get_logger().info(f'Current waypoint index {index}')

    def _save_map(self):
        service_name = self.config['save_map_service']
        client = self.create_client(Trigger, service_name)
        timeout = self.config['save_timeout_sec']
        deadline = time.monotonic() + timeout
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Save map service unavailable: {service_name}')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(0.0, deadline - time.monotonic()))
        if not rclpy.ok():
            return
        if not future.done():
            raise RuntimeError(f'Timed out waiting for {service_name}')
        response = future.result()
        message = f'Save map success={response.success}, message={response.message}'
        if not response.success:
            raise RuntimeError(message)
        self.get_logger().info(message)

    def run(self):
        waypoint_file = self.config['waypoints_file']
        if not waypoint_file or not Path(waypoint_file).is_file():
            raise ValueError('waypoints_file must name an existing CSV file')
        waypoints = load_waypoints_csv(waypoint_file)
        for name in ('initial_wait_sec', 'pre_save_wait_sec', 'server_wait_timeout_sec', 'save_timeout_sec'):
            value = self.config[name]
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and non-negative')

        action_name = self.config['action_name']
        action = ActionClient(self, FollowWaypoints, action_name)
        timeout = self.config['server_wait_timeout_sec']
        if not action.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f'Waypoint action server unavailable: {action_name}')
        if not self._spin_delay(self.config['initial_wait_sec']):
            return

        self.get_logger().info(f'Sending {len(waypoints)} waypoints to {action_name}')
        future = action.send_goal_async(self._goal(waypoints), feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not rclpy.ok():
            return
        if not future.done():
            raise RuntimeError(f'Timed out waiting for {action_name} goal acceptance')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(f'Waypoint goal rejected by {action_name}')

        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, future)
        if not rclpy.ok():
            return
        outcome = future.result()
        result = outcome.result
        missed = [f'{item.index} (error {item.error_code})' for item in result.missed_waypoints]
        self.get_logger().info(f'Missed waypoints: {", ".join(missed) if missed else "none"}')
        complete = (outcome.status == GoalStatus.STATUS_SUCCEEDED and not missed
                    and result.error_code == FollowWaypoints.Result.NONE)
        if not complete:
            self.get_logger().warning(
                f'Waypoint mission ended with status={outcome.status}, '
                f'error_code={result.error_code}, message={result.error_msg}')
        if (self.config['auto_save_map'] and (complete or self.config['save_on_partial'])
                and self._spin_delay(self.config['pre_save_wait_sec'])):
            self._save_map()


def main():
    rclpy.init()
    node = None
    try:
        node = WaypointMappingRunner()
        node.run()
        if not node.config['exit_on_complete'] and rclpy.ok():
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (OSError, ValueError, RuntimeError) as error:
        if node is None:
            raise
        node.get_logger().error(str(error))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
