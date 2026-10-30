"""Construct the shared velocity adapter and watchdog."""
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

from .interfaces import interfaces


def velocity_chain(*, use_sim_time, use_fake_frame, cmd_vel_topic, spin_speed,
                   velocity_window, joint_states_topic=None,
                   output_topic=None, watchdog_parameters=None):
    """Build the same velocity adapter and freshness gate for each platform."""
    contract = interfaces()
    joint_states_topic = joint_states_topic or contract.topic("gimbal.joint_states")
    output_topic = output_topic or contract.topic("navigation.chassis_input")
    fake = Node(
        package="fake_vel_transform", executable="fake_vel_transform_node",
        name="fake_vel_transform", output="screen", condition=IfCondition(use_fake_frame),
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_base_frame": contract.frame('navigation_base'), "fake_robot_base_frame": contract.frame('fake_navigation_base'),
            "odom_topic": contract.topic('localization.odometry'), "input_cmd_vel_topic": cmd_vel_topic,
            "output_cmd_vel_topic": contract.topic('navigation.transformed_output'), "spin_speed": spin_speed,
            "gyro_linear_threshold": 0.01, "odom_timeout": 0.5,
            "nav_velocity_smoothing_window": velocity_window,
        }],
    )
    parameters = {
        "use_sim_time": use_sim_time,
        "input_topic": PythonExpression([
            "'", contract.topic("navigation.transformed_output"), "' if '", use_fake_frame,
            "' == 'true' else '", cmd_vel_topic, "'",
        ]),
        "output_topic": output_topic, "output_frame_id": contract.frame('navigation_base'),
        "timeout": 0.3, "publish_rate": 100.0,
        "require_odom": True, "odom_topic": contract.topic('localization.odometry'), "odom_timeout": 0.5,
        "require_scan": True, "scan_topic": contract.topic('localization.scan'), "scan_timeout": 0.5,
        "require_joint_states": True, "joint_states_topic": joint_states_topic,
        "joint_states_timeout": 0.5, "require_tf": True,
        "tf_target_frame": contract.frame('odom'), "tf_source_frame": contract.frame('navigation_base'), "tf_timeout": 0.5,
        "required_tf_pairs": [f"{contract.frame('odom')}->{contract.frame('ground_base')}",
                              f"{contract.frame('navigation_base')}->{contract.frame('gimbal')}"],
    }
    parameters.update(watchdog_parameters or {})
    watchdog = Node(package="fake_vel_transform", executable="cmd_vel_watchdog.py",
                    name="cmd_vel_watchdog", output="screen", parameters=[parameters])
    return fake, watchdog
