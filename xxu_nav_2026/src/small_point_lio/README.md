# Small Point-LIO

Small Point-LIO is an advanced implementation of the [Point-LIO algorithm](https://github.com/hku-mars/Point-LIO), delivering a 2-3x speed improvement over the original.

The default branch is for ROS2. If you want to run it without ros, please checkout the `main` branch.

If you want to know why it so fast, please read [this](https://bbs.robomaster.com/article/813022).

<img src="./img/ACE.jpg" width="200px">

Robot-specific preprocessing is installed by `xxu_bringup`; see the [LIO integration guide](../../tools/lio/README.md#预处理工具归属).

## Contact

QQ group: 1070252119

Email: 1709185482@qq.com

## Param

here are some parameters you can set in config file:

```yaml
small_point_lio:
    ros__parameters:
        lidar_topic: /livox/lidar # LiDAR topic name
        imu_topic: /livox/imu # IMU topic name
        lidar_type: livox # Lidar type
        lidar_frame: livox_frame # Lidar frame
        save_pcd: false # Whether to save point cloud

        # Point Cloud Filtering
        point_filter_num: 1 # keep one point every N points
        min_distance: 0.5 # Minimum point cloud radius; points closer than this will be filtered
        max_distance: 1000 # Maximum point cloud radius; points farther than this will be filtered
        space_downsample: true # Whether to enable point cloud downsampling
        space_downsample_leaf_size: 0.5 # Voxel size used for point cloud downsampling

        # IMU Processing
        gravity: [0.0, 0.0, -9.810] # Gravity vector
        fix_gravity_direction: true # Default initializer uses a recent stationary window of 200 IMU samples
        check_satu: true # Whether to enable IMU data saturation check
        satu_acc: 3.0 # IMU acceleration saturation threshold
        satu_gyro: 35.0 # IMU angular velocity saturation threshold
        acc_norm: 1.0 # IMU acceleration norm

        # Map
        map_resolution: 0.5 # Map resolution
        init_map_size: 10 # Number of points required to initialize the map

        # LiDAR-IMU Extrinsic Calibration
        extrinsic_est_en: false # Whether to estimate LiDAR-IMU extrinsic transformation online
        extrinsic_T: [-0.011, -0.02329, 0.04412]
        extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Kalman Filter Parameters
        # R
        laser_point_cov: 0.01 # Laser point covariance
        imu_meas_acc_cov: 0.01 # IMU measured acceleration covariance
        imu_meas_omg_cov: 0.01 # IMU measured angular velocity covariance
        # Q
        velocity_cov: 20.0 # Velocity covariance
        acceleration_cov: 500.0 # Acceleration covariance
        omg_cov: 1000.0 # Angular velocity covariance
        ba_cov: 0.0001 # Acceleration bias covariance
        bg_cov: 0.0001 # Gyroscope bias covariance
        plane_thr: 0.1 # Plane matching threshold (smaller value = stricter)
        match_s: 81.0 # Point-to-plane association threshold (smaller value = stricter)

        # Data Publishing
        publish_odometry_without_downsample: false # Whether to publish high-frequency odometry. Note that this does not enhance the real-time nature of the odometry and but degrades performance. It is recommended to increase the point cloud publishing rate to achieve highly real-time odometry.
```

## 运动中初始化（可选候选）

默认仍使用静止窗口初始化。需要验证运动中重启时，在所用的**完整 LIO 参数文件**内加入：

```yaml
small_point_lio:
    ros__parameters:
        dynamic_init_en: true
        dynamic_init_duration: 2.0
        planar_constraint_en: false
```

这段只展示新增参数，不能替代完整参数文件。`dynamic_init_en` 默认 `false`；
开启后自动选择初始化方式：IMU 平稳且多帧点云支持静止时，采用静止模型快速启动；
其余情况积累运动窗口并尝试联合优化。`dynamic_init_duration` 仅约束运动分支关键帧末时刻的最小跨度，
允许 1–5 秒，默认 2 秒；它不强制静止分支等待两秒，也不保证运动启动耗时。
任一分支成功后均退出初始化选择，进入原有里程计；ROS 节点重启或已有 `reset()` 会清除选择与候选结果。

实现借鉴 [TDT 的运动初始化设计思路](https://github.com/mozihe/z-localization/blob/main/docs/design.md#22-初始化)，
在本包内独立实现，不移植对方代码，也不改变现有 30 维 ESKF 的误差状态模型。
原仓库的冷启动统一执行窗口联合优化；下述静止快启加运动补充，是本包的适配策略。

静止分支先检查最近 200 条 IMU：最大角速度不超过 0.05 rad/s，比力均方根波动不超过 0.15 m/s²。
这只是预筛，随后用跨度至少 0.6 秒的 3–5 帧点云检查相对运动和几何支撑：
位移不超过 2 cm、转角不超过 0.01 rad、点到面残差 RMS 不超过 2 cm，且配准信息最弱方向受约束。
通过后采用静止模型，速度和零偏初值置零、平均比力给出重力方向；这不代表静止时能够独立标定加速度计零偏。
这些门限只能提供有限窗口内的静止证据，真实动态物体和配准错误仍需录包验证。

运动分支流程：

1. 在有界缓存中选取 5–12 帧、每帧 40–160 个经过滤的点，以 IMU 加速度均值仅提供重力**初猜**。
2. IMU 传播、逐点运动补偿与帧间点到面粗配准产生初步轨迹。
3. 固定首帧位姿消除坐标系自由度，在同一窗口内联合优化后续位姿、各帧速度、共享陀螺仪/加速度计零偏和重力。
   约束包括帧间共面残差、IMU 旋转/速度/位置预积分残差及重力模长；不以零偏先验掩盖不可观测方向。
4. 使用优化轨迹重新去畸变、更新对应关系，迭代至收敛。
5. 检查匹配数量、信息矩阵秩、残差、零偏大小及估计不确定性，再对齐重力方向，
   用末帧状态、传播后的协方差和累积点云初始化滤波器与地图。

两种检查在单个后台任务中进行，任务内部 OpenMP 限 1 线程，前台线程预算不变。
传感器回调继续缓存数据；只有主线程验收通过才安装结果。
交接保留末帧之后的点云/IMU，并再次检查这段 IMU 的时间连续性。结果过旧或交接期间断流时拒绝安装。
缓存保留窗口长度加 0.75 秒的数据，上限为 10 万稀疏点、30 万稠密点和 2 万条 IMU。

输入要求与限制：

- 点云与 IMU 必须描述同一刚体，外参和逐点时间有效，IMU 加速度单位由既有 `acc_norm` 换算。
  云台补偿到虚拟帧时必须同时补偿点云和 IMU；本功能不替代云台运动补偿。
- “不要求静止”仍需要足够的运动激励和可约束位姿的平面几何。
  合成测试中的持续 yaw 旋转可以走运动分支；固定姿态运动、单一平面等不足以可靠求解时继续等待。
  只有 IMU 与点云静止检查均通过才采用静止模型；关闭 `dynamic_init_en` 则使用原有静止路径。
- `INIT_OK path=static|motion` 表示初始化通过并标明所选分支；`INIT_WAIT reason=...` 表示窗口尚未通过。
  交接保护或后台异常也可能记录 `DYNAMIC_INIT_WAIT reason=...`。
  常见原因为 `insufficient_excitation`、`insufficient_plane_geometry`、`initial_state_uncertain` 或 IMU 时间不连续。
- 当前不兼容 `planar_constraint_en=true` 且 `planar_preserve_initial_tilt=false` 的强制零倾斜投影，
  这个组合会在加载参数时拒绝，防止覆盖恢复姿态却破坏重力、速度和地图的一致性。
  上面的候选配置关闭平面约束，不修改正式默认文件；若保留固定初始倾角的平面约束，则地图同步平移至配置高度。
- 当前预积分使用中点积分及对角噪声近似，噪声密度与验收门限见
  [Options](src/small_point_lio/dynamic_initializer.h)；末帧角速度/比力不确定性沿用配置中的 IMU 测量协方差。
  动态物体、较大初始配准误差、真实传感器噪声及长时间等待的恢复效果仍待录包验证。

### 核验记录：2026-10-27

本次核验自动选择、运动初始化与直接关联的滤波器交接；未执行完整导航或实车回归，未修改正式默认配置。
在 ROS 工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select small_point_lio --symlink-install --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/small_point_lio --output-on-failure
```

构建通过，两个测试程序通过（外部未设置 `OMP_NUM_THREADS`）。证据入口：

- [求解器测试](test/test_dynamic_initializer.cpp)：运动联合优化、点云静止检查、固定姿态匀加速/匀速拒绝、
  单平面退化、IMU 连续性、数值与窗口有效性。传感器生成见 [fixture](test/dynamic_initialization_fixture.h)。
- [前端交接测试](test/test_dynamic_initialization_frontend.cpp)：静止/运动自动选择、成功后任务退出、重置清除选择、
  末状态交接、缓存有界与传感器前沿、断流锁存、不兼容平面投影拒绝，以及原有静止路径。
- 当前构建日志：构建目录 `Testing/Temporary/LastTest.log`，后续测试可能覆盖它。
- [自动选择对照报告](../../../docs/validation/automatic_initialization_20261027.md)：同输入 42 次主对照、默认线程数补测、
  原始 CSV 与源码/二进制指纹。[此前运动分支独用的报告](../../../docs/validation/dynamic_initialization_20261027.md)保留为历史证据。

合成结果不代表实机精度。正式启用前仍需按 [LIO 录包流程](../../tools/lio/README.md#录制)采集静止启动与运动重启数据，
比较启动成功率、重力倾角、初始速度/零偏、启动耗时和交接后漂移，再完成相关导航回归。

## 地图独立对照

2026-10-28，对当前 SmallIVox 与外部 z-localization 累计平面地图完成 108 次合成微基准及上游地图 6 项测试。
没有替换正式地图或修改默认参数。结果与适用范围见[对照报告](../../../docs/validation/voxel_map_comparison_20261028.md)，
复现见[独立测试入口](test/map_benchmark/README.md)。

## 累计平面地图候选

已接入文章所用 z-localization 地图的实际源码，固定提交 `76e126328b670adc92981d9b4900cc059a6f2343`。
[上游子集](3rdparty/zlio_map/README.md)保留原 GPL-2.0 许可证、原文与逐文件指纹；本包原有 MIT 文件保留原版权声明。
该组件需要显式开启构建选项，默认构建与正式平台配置仍使用 SmallIVox。

```bash
colcon build --packages-select small_point_lio --cmake-args \
  -DBUILD_TESTING=ON -DSMALL_POINT_LIO_ZLIO_MAP=ON
ctest --test-dir build/small_point_lio --output-on-failure
```

在所用的完整 LIO 参数文件中指定 `map_backend: moment`、`moment_map_resolution: 0.5`，
或在完整参数之后叠加 [候选覆盖文件](config/moment_map.candidate.yaml)。覆盖文件不能独立启动。
当前要求 `extrinsic_est_en: false` 和有限正值的 `laser_point_cov`；未编译该组件却请求 `moment` 时明确报错。
`map_backend: ivox` 可切回原地图，原有 `map_resolution` 仅供 SmallIVox 使用。

候选包含哈希根体素、八叉树平面、累计点矩/噪声统计和冻结；没有用简化点矩地图替代上游实现。
地图接管静止/运动初始化建图、逐点匹配、即时/按扫描延迟入图与重置。原有 30 维 ESKF 保留，
只将查询到的平面残差和法向接到激光观测，并在观测方差中加入平面不确定性与表面散布。

适配器将本包 `[世界位置误差, 机体系右旋转误差]` 协方差变换为上游
`[世界系左平移误差, 世界系左旋转误差]`，保留交叉项；查询门限使用点的预测边缘协方差，
ESKF 观测噪声不重复叠加当前状态先验。入图采用现有各向同性激光点方差和转换后的位姿协方差，
不替换现有平台的雷达标定模型。延迟入图保存支持点采集时的协方差。

上游最大八叉树深度 4、单节点更新上限 1000 和平面判据保持原样；根体素上限 100 万。
达到根上限时上游抛出资源限制异常，本候选没有新增局部地图淘汰；长距离、复杂几何及实机行为仍待回归。
接入与同输入前端测试的实际结果见[核验报告](../../../docs/validation/moment_map_integration_20261028.md)。
2026-10-28 的 [Gazebo 对照](../../../docs/validation/moment_map_gazebo_20261028.md)中，
两种地图的完整导航回归均未通过；静止速度噪声和重复运行的停滞仍需排查。

## IMU time continuity

The estimator checks IMU sensor-time continuity after initialization.
`max_imu_sample_gap` defaults to 0.25 seconds and must be finite and positive.
If a larger gap occurs, it latches `~/imu_time_fault=true` and stops odometry,
TF, and deskewed navigation-cloud output before propagating across the gap.
Restart localization and reinitialize the map pose to recover; fresh IMU
messages alone do not clear the fault. The fault topic uses reliable,
transient-local QoS. Existing downstream freshness gates remain enabled.


Interval IMU, independent propagation and alternate velocity-source candidate code was removed on 2026-10-17. Historical experiment reports and data have also been removed.

## Published base twist and fallback

The normal base twist uses consecutive LIO source poses after time-matched
base TF conversion, not consecutive predicted publications. For source-pose
intervals `1e-6 < dt <= 1.0` seconds, linear velocity is the position difference
divided by `dt`, rotated into the current child frame; angular velocity contains
only the relative yaw rate (X/Y components are zero). This is an interval
estimate, not a full instantaneous three-dimensional twist.

The first sample or an unusable interval falls back to transformed LIO velocity.
That fallback assumes a fixed lidar-to-base transform and one rigid body, as in
the compensated `lio_base_sensor` path. It does not subtract relative joint
motion, so raw dynamic-gimbal input is outside its complete instantaneous-twist
conversion scope. Public TF and odometry use the corrected source timestamp; repeated or out-of-order outputs are rejected.
Implementation: [odometry callback](src/small_point_lio_node.cpp).

## Source-time navigation output

The optional navigation pose prediction (B) was removed on 2026-10-17. The node publishes corrected source poses directly, without a prediction timer, extrapolation state, prediction parameters or prediction diagnostic topic. IMU gap latching, deskew history, base-frame velocity conversion and conservative covariance remain.

## Save map

**Step 1**: set `save_pcd` to `true` in config file.

**Step 2**: run small point lio until the map is finished.

**Step 3**: save map by calling service:

```cpp
ros2 service call /map_save std_srvs/srv/Trigger
```

> Note: Please make sure you have enough memery to save map. Don't forget to set `save_pcd` to `false` after saving.

## Third-party

Small Point-LIO is built in on and with the aid of the following open source projects. Credits are given to these projects.

|                            project                             |                          description                           |                                            license                                            |
| :------------------------------------------------------------: | :------------------------------------------------------------: | :-------------------------------------------------------------------------------------------: |
|           [Eigen](https://gitlab.com/libeigen/eigen)           |           A C++ template library for linear algebra            | [Mozilla Public License Version 2.0](https://gitlab.com/libeigen/eigen/-/blob/master/LICENSE) |
| [unordered_dense](https://github.com/martinus/unordered_dense) |          A fast & densely stored hashmap and hashset           |         [MIT License](https://github.com/martinus/unordered_dense/blob/main/LICENSE)          |
|       [small_gicp](https://github.com/koide3/small_gicp)       | Efficient and parallel algorithms for point cloud registration |            [MIT License](https://github.com/koide3/small_gicp/blob/master/LICENSE)            |
|          [Open3D](https://github.com/isl-org/Open3D)           |            A Modern Library for 3D Data Processing             |                  [MIT License](https://github.com/isl-org/Open3D/blob/main/LICENSE)                   |

## License

Copyright (C) 2025 Yingjie Huang

Licensed under the MIT License. See License.txt in the project root for license information.
