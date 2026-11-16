"""Launch the STM32 chassis lower-controller serial/USB CDC bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    string_defaults = {
        "port": "/dev/ttyACM0",
        "cmd_vel_topic": "/cmd_vel",
        "expected_frame_id": "base_link",
        "status_topic": "/chassis/status",
        "wheel_vel_topic": "/chassis/wheel_vel",
        "wheel_odom_topic": "/wheel_odom",
        "wheel_odom_frame": "wheel_odom",
        "base_frame": "base_link",
    }
    int_defaults = {
        "baudrate": "115200",
    }
    float_defaults = {
        "send_rate": "50.0",
        "cmd_timeout": "0.3",
        "status_timeout": "0.15",
        "ack_timeout": "0.25",
        "reconnect_interval": "0.5",
    }
    bool_defaults = {
        "use_sim_time": "false",
        "accept_empty_frame_id": "true",
        "require_fresh_stamp": "true",
        "require_run_enable": "false",
        "publish_wheel_vel": "true",
        "publish_wheel_odom": "true",
    }

    params = {name: LaunchConfiguration(name) for name in string_defaults}
    params.update(
        {
            name: ParameterValue(LaunchConfiguration(name), value_type=int)
            for name in int_defaults
        }
    )
    params.update(
        {
            name: ParameterValue(LaunchConfiguration(name), value_type=float)
            for name in float_defaults
        }
    )
    params.update(
        {
            name: ParameterValue(LaunchConfiguration(name), value_type=bool)
            for name in bool_defaults
        }
    )

    return LaunchDescription(
        [
            *(
                DeclareLaunchArgument(name, default_value=value)
                for name, value in {**string_defaults, **int_defaults, **float_defaults}.items()
            ),
            *(
                DeclareLaunchArgument(
                    name, default_value=value, choices=["true", "false"]
                )
                for name, value in bool_defaults.items()
            ),
            Node(
                package="xxu_hardware",
                executable="chassis_lower_bridge",
                name="chassis_lower_bridge",
                parameters=[params],
                output="screen",
            ),
        ]
    )
