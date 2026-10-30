"""Run the real MID360 gimbal compensation chain and Small Point-LIO."""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from xxu_bringup.interfaces import interfaces
from xxu_bringup.sensor_launch import lio_preprocessing


def generate_launch_description():
    contract = interfaces()
    use_sim_time = LaunchConfiguration("use_sim_time")
    model = LaunchConfiguration("model")
    config_file = LaunchConfiguration("config_file")
    lidar_topic = LaunchConfiguration("lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    joint_position_offset = LaunchConfiguration("joint_position_offset")
    input_acceleration_scale = LaunchConfiguration("input_acceleration_scale")
    start_robot_state_publisher = LaunchConfiguration(
        "start_robot_state_publisher"
    )

    default_model = PathJoinSubstitution(
        [FindPackageShare("xxu_description"), "urdf", "xxu.urdf.xacro"]
    )
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("small_point_lio"),
            "config",
            "xxu_mid360_gimbal_compensated.yaml",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(start_robot_state_publisher),
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", model]), value_type=str
            ),
            "use_sim_time": use_sim_time,
        }],
        remappings=[("joint_states", joint_states_topic)],
    )

    lio_nodes = lio_preprocessing(
        "real", use_sim_time=use_sim_time, config_file=config_file,
        pointcloud_overrides={"joint_position_offset": ParameterValue(joint_position_offset, value_type=float)},
        imu_overrides={
            "joint_position_offset": ParameterValue(joint_position_offset, value_type=float),
            "input_acceleration_scale": ParameterValue(input_acceleration_scale, value_type=float),
        },
        pointcloud_remappings={"points_in": lidar_topic, "joint_states": joint_states_topic},
        imu_remappings={"imu_in": imu_topic, "joint_states": joint_states_topic},
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the ROS simulation clock when true",
        ),
        DeclareLaunchArgument(
            "model",
            default_value=default_model,
            description="Real robot URDF/Xacro path",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Small Point-LIO compensated parameter file",
        ),
        DeclareLaunchArgument(
            "lidar_topic",
            default_value=contract.topic('radar.pointcloud'),
            description="Raw livox_ros_driver2 PointCloud2 topic",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value=contract.topic('radar.imu'),
            description="Raw livox_ros_driver2 IMU topic (acceleration in g)",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value=contract.topic('gimbal.joint_states'),
            description="Timestamped gimbal joint-state topic",
        ),
        DeclareLaunchArgument(
            "joint_position_offset",
            default_value="0.0",
            description="Encoder-to-mechanical gimbal zero offset in radians",
        ),
        DeclareLaunchArgument(
            "input_acceleration_scale",
            default_value="9.81",
            description="Raw IMU acceleration multiplier; official Livox uses g",
        ),
        DeclareLaunchArgument(
            "start_robot_state_publisher",
            default_value="true",
            description="Publish the real robot TF tree from the included Xacro",
        ),
        robot_state_publisher,
        *lio_nodes,
    ])
