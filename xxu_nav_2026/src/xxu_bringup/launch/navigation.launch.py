"""Start Nav2 localization and navigation for the XXU robot."""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from xxu_bringup.interfaces import interfaces
from xxu_bringup.navigation_parameters import nav2_parameters


def map_server_node(configured_params, map_file, use_sim_time):
    """Construct the navigation map server."""
    return Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[configured_params, {"yaml_filename": map_file, "use_sim_time": use_sim_time}],
    )


def localization_nodes(configured_params, use_sim_time, enable_gicp,
                       default_gicp_params, gicp_pcd_map, nav2_common_remaps):
    """Construct the existing AMCL and optional GICP localization nodes."""
    return [
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="both",
            # AMCL remains the coarse initializer/relocalizer, but must not
            # publish map->odom while GICP owns that transform.
            parameters=[configured_params, {
                "tf_broadcast": ParameterValue(PythonExpression([
                    "'false' if '", enable_gicp, "' == 'true' else 'true'",
                ]), value_type=bool),
            }],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="xxu_gicp_localization",
            executable="gicp_localization_node",
            name="gicp_localization",
            output="both",
            condition=IfCondition(enable_gicp),
            parameters=[default_gicp_params, {
                "use_sim_time": use_sim_time,
                "pcd_map": gicp_pcd_map,
            }],
        ),
    ]


def navigation_server_nodes(selected_nav_params, nav2_common_remaps, contract):
    """Construct the Nav2 navigation servers without selecting localization."""
    return [
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[selected_nav_params],
            remappings=nav2_common_remaps + [("cmd_vel", contract.topic("navigation.controller_output", relative=True))],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[selected_nav_params],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[selected_nav_params],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[selected_nav_params],
            remappings=nav2_common_remaps + [("cmd_vel", contract.topic("navigation.controller_output", relative=True))],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[selected_nav_params],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[selected_nav_params],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[selected_nav_params],
            remappings=nav2_common_remaps + [("cmd_vel", contract.topic("navigation.controller_output", relative=True))],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[selected_nav_params],
            remappings=nav2_common_remaps,
        ),
    ]


def localization_lifecycle_manager(use_sim_time, autostart, localization_lifecycle_nodes):
    """Construct the existing map/localization lifecycle manager."""
    return Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": localization_lifecycle_nodes,
        }],
    )


def generate_launch_description():
    contract = interfaces()
    bringup_share = FindPackageShare("xxu_bringup")
    gicp_share = FindPackageShare("xxu_gicp_localization")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    autostart = LaunchConfiguration("autostart")
    use_fake_frame = LaunchConfiguration("use_fake_frame")
    enable_localization = LaunchConfiguration("enable_localization")
    localization_mode = LaunchConfiguration("localization_mode")
    enable_gicp = LaunchConfiguration("enable_gicp")
    gicp_pcd_map = LaunchConfiguration("gicp_pcd_map")
    cmd_vel_in_topic = LaunchConfiguration("cmd_vel_in_topic")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")
    planner_tolerance = LaunchConfiguration("planner_tolerance")

    # Keep standalone Nav2 aligned with the complex_mapping Gazebo world.
    default_map = PathJoinSubstitution([bringup_share, "maps", "complex_map.yaml"])
    default_params = PathJoinSubstitution([bringup_share, "config", "nav2_navigation.yaml"])
    default_rviz_config = PathJoinSubstitution([bringup_share, "rviz", "navigation.rviz"])
    default_gicp_params = PathJoinSubstitution(
        [gicp_share, "config", "gicp_localization.yaml"]
    )

    localization_lifecycle_nodes = [
        "map_server",
        "amcl",
    ]
    navigation_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor",
    ]

    nav2_common_remaps = [(contract.topic('system.tf'), "tf"), (contract.topic('system.tf_static'), "tf_static")]
    configured_params, selected_nav_params, _ = nav2_parameters(
        params_file, use_sim_time, use_fake_frame,
        get_package_share_directory("xxu_bringup"), mode="navigation",
        robot_geometry_file=LaunchConfiguration('robot_geometry_file'),
        extra_rewrites={
            "collision_monitor.ros__parameters.cmd_vel_in_topic": cmd_vel_in_topic,
            "collision_monitor.ros__parameters.cmd_vel_out_topic": cmd_vel_out_topic,
            "planner_server.ros__parameters.GridBased.tolerance": planner_tolerance,
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_geometry_file', default_value=''),
        DeclareLaunchArgument('localization_mode', default_value='amcl',
                             choices=['amcl', 'external']),
        DeclareLaunchArgument(
            "enable_localization",
            default_value="false",
            description="Re-enable paused AMCL/GICP localization and map navigation",
        ),
        LogInfo(msg="Map navigation requires enable_localization:=true; external mode uses the supplied localization state and TF."),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to the map yaml file",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Full path to the Nav2 parameters file",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Full path to the RViz navigation config file",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically configure and activate lifecycle nodes",
        ),
        DeclareLaunchArgument(
            "use_fake_frame",
            default_value="true",
            description="Use gimbal_yaw_fake as the Nav2 robot base frame",
        ),
        DeclareLaunchArgument(
            "enable_gicp",
            default_value="false",
            description="Use AMCL only for initialization and small_gicp for map->odom",
        ),
        DeclareLaunchArgument(
            "gicp_pcd_map",
            default_value="",
            description="Map-frame PCD consumed by xxu_gicp_localization",
        ),
        DeclareLaunchArgument(
            "cmd_vel_in_topic",
            default_value=contract.topic("navigation.smoothed_output", relative=True),
            description="Velocity input consumed by collision_monitor",
        ),
        DeclareLaunchArgument(
            "cmd_vel_out_topic",
            default_value=contract.topic('navigation.collision_output'),
            description="Collision-monitored velocity output",
        ),
        DeclareLaunchArgument(
            "planner_tolerance",
            default_value="1.0",
            description="Navfn endpoint tolerance in metres",
        ),
        GroupAction(
            condition=IfCondition(enable_localization),
            actions=[
                map_server_node(configured_params, map_file, use_sim_time),
                GroupAction(condition=IfCondition(PythonExpression([
                    "'", localization_mode, "' == 'amcl'",
                ])), actions=[
                    *localization_nodes(configured_params, use_sim_time, enable_gicp,
                                        default_gicp_params, gicp_pcd_map, nav2_common_remaps),
                    localization_lifecycle_manager(use_sim_time, autostart, localization_lifecycle_nodes),
                ]),
                GroupAction(condition=IfCondition(PythonExpression([
                    "'", localization_mode, "' == 'external'",
                ])), actions=[localization_lifecycle_manager(use_sim_time, autostart, ['map_server'])]),
                *navigation_server_nodes(selected_nav_params, nav2_common_remaps, contract),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        # Activation waits for localization in the selected mode.
                        # nav2_navigation_startup starts this manager once it exists.
                        "autostart": False,
                        "node_names": navigation_lifecycle_nodes,
                    }],
                ),
                Node(
                    package="xxu_bringup",
                    executable="nav2_navigation_startup.py",
                    name="nav2_navigation_startup",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time,
                                 "require_localization": ParameterValue(PythonExpression([
                                     "'", localization_mode, "' == 'external'",
                                 ]), value_type=bool)}],
                ),
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2_navigation",
                    output="screen",
                    condition=IfCondition(rviz),
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
            ],
        ),
    ])
