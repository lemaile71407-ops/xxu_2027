from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_model_path = PathJoinSubstitution(
        [FindPackageShare("xxu_description"), "urdf", "xxu.urdf.xacro"]
    )
    default_rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("xxu_description"), "rviz", "display.rviz"]
    )

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # Mechanical model inspection has a private TF graph. It may run beside
    # navigation without reintroducing wheel/gimbal frames into global /tf.
    model_tf = [("/tf", "/model_display/tf"), ("/tf_static", "/model_display/tf_static")]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model_path,
                description="Absolute path to robot URDF or Xacro file",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config_path,
                description="Absolute path to RViz config file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                remappings=[*model_tf, ("/joint_states", "/model_display/joint_states")],
                parameters=[
                    {
                        "robot_description": Command(["xacro ", model]),
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                remappings=[("/joint_states", "/model_display/joint_states")],
                parameters=[
                    {
                        "robot_description": Command(["xacro ", model]),
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                remappings=model_tf,
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
