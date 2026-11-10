"""启动 Gazebo Harmonic 并加载 XXU 机器人模型."""


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from xxu_bringup.interfaces import interfaces
from xxu_bringup.sensor_launch import sensor_chain_defaults, sensor_preprocessing, small_point_lio
from xxu_bringup.velocity_launch import velocity_chain


def generate_launch_description():
    contract = interfaces()
    pkg_share = FindPackageShare("xxu_description")

    model_path = PathJoinSubstitution(
        [pkg_share, "urdf", "xxu_gazebo.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [pkg_share, "config", "xxu_ros2_control.yaml"]
    )
    nav2_tb3_models_path = PathJoinSubstitution(
        [FindPackageShare("nav2_minimal_tb3_sim"), "models"]
    )
    xxu_description_share_parent = PathJoinSubstitution(
        [FindPackagePrefix("xxu_description"), "share"]
    )
    compensate_gimbal_for_lio = LaunchConfiguration("compensate_gimbal_for_lio")
    small_point_lio_config = PathJoinSubstitution([
        FindPackageShare("small_point_lio"),
        "config",
        PythonExpression([
            "'xxu_gazebo_mid360_gimbal_compensated.yaml' if '",
            compensate_gimbal_for_lio,
            "' == 'true' else 'xxu_gazebo_mid360.yaml'",
        ]),
    ])
    mid360_scan_mode_csv = PathJoinSubstitution(
        [FindPackageShare("xxu_livox_sim"), "config", "mid360.csv"]
    )
    livox_plugin_path = PathJoinSubstitution(
        [FindPackagePrefix("xxu_livox_sim"), "lib", "xxu_livox_sim"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_lio = LaunchConfiguration("enable_lio")
    enable_cmd_vel_odom = LaunchConfiguration("enable_cmd_vel_odom")
    use_livox_native = LaunchConfiguration("use_livox_native")
    use_fake_frame = LaunchConfiguration("use_fake_frame")
    gimbal_mode = LaunchConfiguration("gimbal_mode")
    gimbal_spin_rate = LaunchConfiguration("gimbal_spin_rate")
    gimbal_spin_start_delay = LaunchConfiguration("gimbal_spin_start_delay")
    gyro_spin_rate = LaunchConfiguration("gyro_spin_rate")
    nav_velocity_smoothing_window = LaunchConfiguration("nav_velocity_smoothing_window")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    scan_self_filter_radius = LaunchConfiguration("scan_self_filter_radius")
    scan_min_height = LaunchConfiguration("scan_min_height")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")

    odom_only_warning = LogInfo(
        msg=(
            "enable_lio=false with enable_cmd_vel_odom=true is odom-only: "
            "the safety scan uses gimbal-compensated points without LIO body-motion deskew. "
            "This diagnostic mode does not support map navigation."
        ),
        condition=IfCondition(
            PythonExpression([
                "'",
                enable_lio,
                "' == 'false' and '",
                enable_cmd_vel_odom,
                "' == 'true'",
            ])
        ),
    )

    gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        [
            xxu_description_share_parent,
            ":",
            nav2_tb3_models_path,
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH"),
        ],
    )
    gz_system_plugin_path = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        [
            livox_plugin_path,
            ":",
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )
    rmw_implementation = SetEnvironmentVariable(
        "RMW_IMPLEMENTATION",
        "rmw_cyclonedds_cpp",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    declare_lio_motion_diagnostics = DeclareLaunchArgument(
        "lio_motion_diagnostics", default_value="false",
        description="Log LIO motion propagation and matching diagnostics once per simulation second",
    )
    declare_nav_velocity_smoothing_window = DeclareLaunchArgument(
        "nav_velocity_smoothing_window", default_value="0.3",
        description="Navigation velocity average window in seconds; 0 disables filtering",
    )
    declare_enable_lio = DeclareLaunchArgument(
        "enable_lio",
        default_value="true",
        description="Start Small Point-LIO and publish /odom",
    )
    declare_compensate_gimbal_for_lio = DeclareLaunchArgument(
        "compensate_gimbal_for_lio",
        default_value="true",
        description=(
            "Remove known gimbal motion from point cloud and IMU before LIO; "
            "false restores the raw mid360_link estimator baseline"
        ),
    )
    declare_enable_cmd_vel_odom = DeclareLaunchArgument(
        "enable_cmd_vel_odom",
        default_value="false",
        description=(
            "Publish odom by integrating /cmd_vel when LIO is disabled; "
            "this fallback has no /scan and is not a navigation source"
        ),
    )
    declare_use_livox_native = DeclareLaunchArgument(
        "use_livox_native",
        default_value="true",
        description="Use batch-raycast MID360 simulation with per-point timestamps",
    )
    declare_use_fake_frame = DeclareLaunchArgument(
        "use_fake_frame",
        default_value="true",
        description="Use fake_vel_transform and gimbal_yaw_fake for Nav2 velocity commands",
    )
    declare_gimbal_mode = DeclareLaunchArgument(
        "gimbal_mode",
        default_value="spin",
        description="Radar gimbal mode: spin or hold",
    )
    declare_gimbal_spin_rate = DeclareLaunchArgument(
        "gimbal_spin_rate",
        default_value="6.283185307179586",
        description="Continuous radar gimbal speed in rad/s when gimbal_mode=spin",
    )
    declare_gimbal_spin_start_delay = DeclareLaunchArgument(
        "gimbal_spin_start_delay",
        default_value="1.0",
        description="Seconds to hold the MID360 after the first LIO odometry output",
    )
    declare_gyro_spin_rate = DeclareLaunchArgument(
        "gyro_spin_rate",
        default_value="1.5",
        description="Chassis gyro angular speed while translating (rad/s); 0 disables it",
    )
    declare_gazebo_gui = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        description="Start the Gazebo Sim GUI; disable it to reduce CPU load during mapping",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_share, "worlds", "complex_mapping.sdf"]),
        description="Gazebo world SDF path",
    )
    declare_spawn_x = DeclareLaunchArgument(
        "spawn_x", default_value="1.75",
        description="Gazebo robot spawn x in world coordinates",
    )
    declare_spawn_y = DeclareLaunchArgument(
        "spawn_y", default_value="0.0",
        description="Gazebo robot spawn y in world coordinates",
    )
    declare_spawn_yaw = DeclareLaunchArgument(
        "spawn_yaw", default_value="3.14159",
        description="Gazebo robot spawn yaw in world coordinates",
    )
    declare_scan_self_filter_radius = DeclareLaunchArgument(
        "scan_self_filter_radius", default_value="0.42",
        description="Base-frame radius that deterministically removes chassis self-returns from /scan",
    )
    declare_scan_min_height = DeclareLaunchArgument(
        "scan_min_height", default_value="0.30",
        description="Minimum base-frame point height retained in the shared navigation scan",
    )
    declare_cmd_vel_out_topic = DeclareLaunchArgument(
        "cmd_vel_out_topic", default_value=contract.topic('navigation.collision_output'),
        description="Collision-monitor output consumed by the downstream safety chain",
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command([
                    "xacro ",
                    model_path,
                    " controllers_file:=",
                    controllers_file,
                    " use_livox_native:=",
                    use_livox_native,
                    " scan_mode_csv:=",
                    mid360_scan_mode_csv,
                ]),
                value_type=str,
            ),
            "use_sim_time": use_sim_time,
            # Joint states are already available at ~166 Hz.  Publish the
            # moving gimbal TF at 100 Hz so MID-360 point-time lookups do not
            # routinely extrapolate into the future.
            "publish_frequency": 100.0,
        }],
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            # Server-only mode keeps the physics/sensors running while leaving
            # CPU for LIO, SLAM and Nav2. RViz remains available separately.
            "gz_args": [
                PythonExpression([
                    "'-r ' if '", gazebo_gui,
                    "' == 'true' else '-s -r '",
                ]),
                world,
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # 生成机器人
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description",
                   "-name", "xxu",
                   "-x", spawn_x,
                   "-y", spawn_y,
                   "-z", "0.05",
                   "-Y", spawn_yaw,
                   "-allow_renaming", "false"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    gimbal_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gimbal_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                wheel_velocity_controller_spawner,
                gimbal_velocity_controller_spawner,
            ],
        )
    )

    # 默认模拟实车搜索目标时的连续旋转，也可切回世界航向保持模式。
    # 动态关节状态由 robot_state_publisher 转换成 base_link -> gimbal_link TF，
    # 点云与 IMU 共用这条真实运动链。
    gimbal_stabilizer = Node(
        package="xxu_bringup",
        executable="gimbal_stabilizer.py",
        name="gimbal_stabilizer",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "mode": gimbal_mode,
            "spin_rate": gimbal_spin_rate,
            "spin_start_delay": gimbal_spin_start_delay,
            "command_topic": contract.topic('simulation.gimbal_velocity'),
            "command_rate": 200.0,
            "wait_for_odom": enable_lio,
            "odom_topic": contract.topic('localization.odometry'),
        }],
    )

    chassis_controller = Node(
        package="xxu_chassis_controller",
        executable="chassis_controller",
        name="chassis_controller",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "wheel_radius": 0.0762,
            "wheel_command_sign": -1.0,
            "wheel_x": [0.223275, 0.223275, -0.223275, -0.223275],
            "wheel_y": [0.223275, -0.223275, 0.223275, -0.223275],
            "drive_direction_angle": [-0.785398, -2.356194, 0.785398, 2.356194],
            "max_wheel_speed": 100.0,
            "timeout": 0.3,
            "publish_rate": 50.0,
            "cmd_vel_topic": contract.topic('navigation.chassis_input'),
            "wheel_command_topic": contract.topic('chassis.wheel_command'),
            "expected_frame_id": contract.frame('navigation_base'),
            "accept_empty_frame_id": True,
        }],
    )

    fake_vel_transform, cmd_vel_watchdog = velocity_chain(
        use_sim_time=use_sim_time, use_fake_frame=use_fake_frame,
        cmd_vel_topic=cmd_vel_out_topic, spin_speed=gyro_spin_rate,
        velocity_window=ParameterValue(nav_velocity_smoothing_window, value_type=float),
        watchdog_parameters={"status_topic": contract.topic('system.healthy'), "tf_topic": contract.topic('system.tf')},
    )
    # IMU bridge (GZ -> ROS 单向)
    bridge_imu = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_imu",
        arguments=[
            contract.topic("simulation.imu") + "@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
        remappings=[
            (contract.topic('simulation.imu'), contract.topic('simulation.imu_raw')),
        ],
    )

    # Transform each acquisition-time group exactly once. In production the
    # result is expressed at the rigid gimbal-axis frame and shared by LIO and
    # navigation. The false branch keeps the raw-LIO rollback while navigation
    # still receives a deskewed cloud in base_footprint.
    lio_nodes = sensor_preprocessing(
        "gazebo", use_sim_time=use_sim_time,
        pointcloud_overrides={
            "target_frame": PythonExpression([
                "'", contract.frame("lidar_compensated"), "' if '", compensate_gimbal_for_lio,
                "' == 'true' else '", contract.frame("ground_base"), "'",
            ]),
            "use_joint_interpolation": ParameterValue(compensate_gimbal_for_lio, value_type=bool),
        },
        conditions={
            "imu": IfCondition(PythonExpression([
                "'", enable_lio, "' == 'true' and '",
                compensate_gimbal_for_lio, "' == 'true'",
            ])),
            "raw_pointcloud": UnlessCondition(use_livox_native),
        },
    )
    lio_nodes.append(small_point_lio(
        "gazebo", use_sim_time=use_sim_time, config_file=small_point_lio_config,
        lio_overrides={"motion_diagnostics_en": ParameterValue(
            LaunchConfiguration("lio_motion_diagnostics"), value_type=bool)},
        condition=IfCondition(enable_lio),
    ))

    # Fallback GPU lidar bridge. The native Livox plugin publishes its own
    # PointCloud2 and does not need this extra sensor stream.
    bridge_lidar = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_lidar",
        arguments=[
            contract.topic("simulation.lidar_gpu") + "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
        condition=UnlessCondition(use_livox_native),
        remappings=[
            (contract.topic('simulation.lidar_gpu'), contract.topic('simulation.lidar_raw')),
        ],
    )

    bridge_livox_native = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_livox_native",
        arguments=[
            contract.topic("simulation.lidar_native") + "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
        condition=IfCondition(use_livox_native),
        remappings=[
            # The native Gazebo plugin already publishes mid360_link and
            # per-point timestamps; avoid a frame-id-only republisher hop.
            (contract.topic('simulation.lidar_native'), contract.topic('simulation.lidar_shaped')),
        ],
    )

    # Gazebo Harmonic names a sensor attached to gimbal_link as
    # xxu/gimbal_link/mid360_lidar and does not preserve the SDF frame_id.
    # This is the real sensor mount transform from the URDF, exposed in TF so
    # pointcloud_frame_republisher can transform by message timestamp.
    sensor_defaults = sensor_chain_defaults("gazebo")["pointcloud"]["parameters"]
    gpu_lidar_sensor_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gpu_lidar_sensor_tf",
        output="screen",
        condition=UnlessCondition(use_livox_native),
        arguments=[
            "--x", str(sensor_defaults["sensor_offset"][0]),
            "--y", str(sensor_defaults["sensor_offset"][1]),
            "--z", str(sensor_defaults["sensor_offset"][2]),
            "--roll", str(sensor_defaults["sensor_rpy"][0]),
            "--pitch", str(sensor_defaults["sensor_rpy"][1]),
            "--yaw", str(sensor_defaults["sensor_rpy"][2]),
            "--frame-id", contract.frame('gimbal'),
            "--child-frame-id", contract.frame('gpu_lidar'),
        ],
    )

    livox_pointcloud_shaper = Node(
        package="xxu_livox_sim",
        executable="livox_pointcloud_shaper",
        name="livox_pointcloud_shaper",
        output="screen",
        condition=UnlessCondition(use_livox_native),
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": contract.frame('gazebo_lidar'),
            # Gazebo gpu_lidar publishes one instantaneous point-cloud frame.
            # Do not fabricate per-point times across 0.1 s: a fast gimbal would
            # make LIO deskew an already coherent frame and bend it by tens of degrees.
            "scan_period": 0.0,
            "max_points": 12000,
            "scan_mode_csv": mid360_scan_mode_csv,
        }],
        remappings=[
            ("points_in", contract.topic('simulation.lidar_lio')),
            ("points_out", contract.topic('simulation.lidar_shaped')),
        ],
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": contract.frame('ground_base'),
            "transform_tolerance": 0.05,
            # The chassis top is z=0.26949 m in base_footprint.  Retaining
            # only points above it removes deterministic deskewed body echoes
            # before the shared Nav2/TTC/Collision Monitor scan is formed.
            "min_height": ParameterValue(scan_min_height, value_type=float),
            "max_height": 0.50,
            "angle_min": -3.1415926,
            "angle_max": 3.1415926,
            "angle_increment": 0.0087266,
            "scan_time": 0.1,
            "range_min": 0.30,
            "range_max": 40.0,
            "robot_self_filter_radius": scan_self_filter_radius,
            "use_inf": True,
            "inf_epsilon": 1.0,
            "scan_qos_reliability": "reliable",
            "publish_processed_cloud": True,
            "processed_cloud_frame": contract.frame('ground_base'),
            "processed_cloud_project_to_2d": True,
            "processed_cloud_z_value": 0.0,
            "crop_min_x": -40.0,
            "crop_max_x": 40.0,
            "crop_min_y": -40.0,
            "crop_max_y": 40.0,
        }],
        remappings=[
            # Navigation uses LIO body-motion deskew. Without LIO, retain a
            # gimbal-compensated scan for the diagnostic safety chain.
            ("cloud_in", PythonExpression([
                "'", contract.topic("localization.deskewed_cloud"), "' if '", enable_lio,
                "' == 'true' else '", contract.topic("simulation.lidar_compensated"), "'",
            ])),
            ("scan", contract.topic('localization.scan')),
            ("processed_cloud", contract.topic('localization.navigation_cloud')),
        ],
    )


    # Clock bridge
    bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_clock",
        arguments=[
            contract.topic("system.clock") + "@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )


    cmd_vel_odometry = Node(
        package="xxu_bringup",
        executable="cmd_vel_odometry.py",
        name="cmd_vel_odometry",
        output="screen",
        condition=IfCondition(enable_cmd_vel_odom),
        parameters=[{
            "use_sim_time": use_sim_time,
            "cmd_vel_topic": contract.topic('navigation.chassis_input'),
            "odom_topic": contract.topic('localization.odometry'),
            "odom_frame": contract.frame('odom'),
            "base_frame": contract.frame('ground_base'),
            "publish_rate": 50.0,
            "cmd_timeout": 0.3,
        }],
    )

    return LaunchDescription([
        rmw_implementation,
        gz_resource_path,
        gz_system_plugin_path,
        declare_use_sim_time,
        declare_enable_lio,
        declare_lio_motion_diagnostics,
        declare_nav_velocity_smoothing_window,
        declare_compensate_gimbal_for_lio,
        declare_enable_cmd_vel_odom,
        declare_use_livox_native,
        declare_use_fake_frame,
        declare_gimbal_mode,
        declare_gimbal_spin_rate,
        declare_gimbal_spin_start_delay,
        declare_gyro_spin_rate,
        declare_gazebo_gui,
        declare_world,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_yaw,
        declare_scan_self_filter_radius,
        declare_scan_min_height,
        declare_cmd_vel_out_topic,
        odom_only_warning,
        robot_state_publisher,
        gz_sim,
        # Gazebo advertises /clock only after the world is running. Starting
        # the bridge immediately can leave it connected but receiving no clock
        # samples, so defer it briefly until the world transport is ready.
        TimerAction(period=2.0, actions=[bridge_clock]),
        spawn_robot,
        spawn_controllers,
        gimbal_stabilizer,
        chassis_controller,
        fake_vel_transform,
        bridge_imu,
        *lio_nodes,
        bridge_lidar,
        bridge_livox_native,
        gpu_lidar_sensor_tf,
        # These processing nodes only subscribe and wait until sensor data is
        # available, so start them immediately instead of adding fixed delays.
        livox_pointcloud_shaper,
        pointcloud_to_scan,
        cmd_vel_watchdog,
        cmd_vel_odometry,
    ])
