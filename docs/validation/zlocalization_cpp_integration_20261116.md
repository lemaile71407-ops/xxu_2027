# Z-Localization C++ 链路接入核验（2026-11-16）

核验对象：远程 `xxu_2027` 对应的本地 `xxu_2026` 工作区，以 `xxu_nav_2026/src/z-localization` 为实际源码。根目录旧独立副本仅提供当前本机已有的 GTSAM/Livox SDK2 依赖，不作为本次业务源码。本文区分源码接入、当前构建、合成/回环测试与尚未完成的实车验收。

## Phase 1：修改前的只读结论

只读报告已先于代码修改输出。修改前的接口如下；旧 Python 源码现已由 C++ 替代，下表是迁移基线，不代表当前运行接口。

| 原节点 | 订阅 | 发布 | 原 QoS |
| --- | --- | --- | --- |
| `zlio_base_odometry.py` | `/odometry`，`nav_msgs/msg/Odometry` | `/odom`，同类型；`/tf` | 里程计 Reliable/Keep Last 100 → 50 |
| 同节点的可选云桥 | `/frontend/frame`，`FrontendFrame`；`/frontend/session`，`FrontendSession` | `/cloud_deskewed`，`sensor_msgs/msg/PointCloud2` | 帧 Reliable/10，云 Reliable/10；会话 Reliable/Transient Local/1 |
| `cmd_vel_watchdog.py` | `/cmd_vel_collision`，`TwistStamped`；`/odom`，`Odometry`；`/scan`，`LaserScan`；`/joint_states`，`JointState`；`/tf`，`TFMessage` | 默认 `/cmd_vel`，`TwistStamped`；`/cmd_vel_watchdog/healthy`，`Bool` | 命令/传感器/显式 TF 订阅 Sensor Data；命令和状态输出 Reliable/10；另有常驻 TF Listener |
| 同节点的定位保护 | `/backend_node/localization_state`，`String`；`/localization/odometry`，`Odometry` | — | 原订阅 Reliable/100 |

节点安装原由 [CMakeLists](../../xxu_nav_2026/src/xxu_bringup/CMakeLists.txt) 管理；启动入口为 [底盘适配](../../xxu_nav_2026/src/xxu_bringup/launch/zlio_base_adapter.launch.py)、[速度链构造](../../xxu_nav_2026/src/xxu_bringup/xxu_bringup/velocity_launch.py)。[接口 YAML](../../xxu_nav_2026/src/xxu_bringup/config/robot_interfaces.yaml) 是话题/坐标名来源，[传感器 YAML](../../xxu_nav_2026/src/xxu_bringup/config/sensor_chain.yaml) 是预处理参数来源，未发现独立的旧看门狗参数 YAML。

### 坐标与外参基线

- 前端默认父帧 `odom`、子帧 `imu`，并非 `imu_link`。原适配器明确把 `imu` 视为固定虚拟传感器 `lio_base_sensor`，每条里程计查询 `lio_base_sensor ← base_footprint`，输出 `child_frame_id=base_footprint`。
- 旧转换使用 `T_ob = T_os T_sb`；速度为 `v_b = R_sbᵀ(v_s + ω_s × t_sb)`、`ω_b = R_sbᵀ ω_s`。位姿协方差使用世界系左旋转误差的杆臂 Jacobian，速度协方差使用局部刚体伴随 Jacobian，两者不能混用。
- EGA [URDF](../../xxu_nav_2026/src/xxu_sim_bringup/models/ega_folded/robot.urdf) 的原地面到车体高度为 `0.06973823713479495 m`；`base_link ← lio_base_sensor` 为单位变换。雷达壳体相对车体平移约 `[0.132428, -0.080466, 0.140657] m`，RPY 约 `[-0.383972, 0, -2.116797] rad`；仿真帧还包括 `livox_lidar`、`livox_lidar_imu`。这些模型值不是实车标定。
- 实车使用 `livox_frame`；[待测安装模板](../../xxu_nav_2026/src/xxu_bringup/config/rigid_mount.pending.yaml) 仍含 `null`，且 `calibration_confirmed=false`。没有代填实车外参。

### 保护基线

命令超时 `0.3 s`，里程计、扫描、关节、所需动态 TF 均为 `0.5 s`。源码输出默认 `20 Hz`，共享启动覆盖为 `100 Hz`。命令同时检查稳态接收时间和 ROS 源时间戳，不能用重发输出更新时间来续期。墙钟定时器保证 `/clock` 暂停时继续停车。

开启定位保护后，状态必须为 `NORMAL` 或 `OBS_DEGRADED`、`pose_trusted=true`、`tf_frozen=false`；还要检查状态与位姿的接收时间、源时间持续推进及 ROS 时间年龄。原定位超时 `0.5 s`，恢复保持 `1 s`。坐标 epoch 变化不能复用旧位姿，正常可信 generation 变化不能为旧源时间续期。关闭期间和恢复开门边沿清空命令，须收到新命令才放行。

### 扫描与其他 Python 路径

扫描由仓库内 C++ [pointcloud_to_laserscan](../../xxu_nav_2026/src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp) 生成。原 RMUL 输入已是 `/cloud_deskewed`，但该云来自 Python FrameCloudBridge；扫描原目标为 `base_footprint`，扫描/processed cloud 为 Reliable，且转换器常驻 TF Listener。单改输入 remapping 不能消除原 Python 解码路径。

生产入口还涉及点云变换、IMU 补偿、里程计启动门、仿真适配/云台、实车串口等 Python 节点。导航启动/空闲补扫脚本原本订阅定位里程计并监听 TF。忽略的外部 `rm_sim_26` 副本还有 `omni_drive.py`；本仓库启动已改用自有 C++ 对应程序，没有修改外部副本。

离线/诊断 Python 工具仍包括 `inspect_mid360_bag.py`、`check_cloud_truth_alignment.py`、`record_benchmark_bag.py`、`check_ega_navigation.py`、`observe_mppi_manual.py`、`record_localization_jumps.py`、`monitor_localization_integrity.py`，以及合成测试。这些工具不在部署的传感器/TF/控制数据路径中；它们的存在不应被描述为“仓库完全没有 Python”。未发现自建生产 Python 节点订阅 `CustomMsg`。

### 前后端接口基线

- 前端 `/odometry` 是随 IMU 更新的高频预测，`/lidar_odometry` 是后验；`/frontend/frame` 封装同步后验和 IMU 局部去畸变云，`/frontend/odometry` 为带会话身份的预测，`/frontend/session` 为保留会话通知。
- 原独立可视化云 `/frontend/cloud_registered` 是配准到世界系的显示结果，不能直接当作局部导航云。
- 原 `publish_tf` 默认开启，已有 RMUL 组合显式关闭。后端发布 `/localization/odometry` 和 `map → odom`。
- `/backend_node/relocalize` 是 `std_srvs/srv/Trigger`，当前仅 LOST 时接受手动重定位；公开协调重置为 `/localization_reset`，前端内部协议为 `/zlio/reset_internal`（`localization_interfaces/srv/ResetFrontend`）。服务语义未改。
- 当前前端以 `window.minimum_interval/maximum_interval` 自适应调度，没有“分段更新数”参数。按本次更新后的任务范围，没有修改体素、噪声、采样点数或时间调度 YAML。

## Phase 2—4：当前实现与边界

### C++ 组件和 TF

[里程计组件](../../xxu_nav_2026/src/xxu_bringup/src/zlio_base_odometry.cpp) 与 [转换函数](../../xxu_nav_2026/src/xxu_bringup/include/xxu_bringup/odometry_conversion.hpp) 缓存 `Eigen::Isometry3d T_base_sensor`，计算 `T_odom_base = T_odom_sensor * inverse(T_base_sensor)`，保留原始时间，输出 `/odom`、`child_frame_id=base_link` 和相同位姿的 TF。支持两种外参来源：

- `extrinsics_source=parameters`：必须提供三维 `base_from_sensor_xyz` 和 `base_from_sensor_rpy`，启动时缓存，无 TF 订阅。
- `extrinsics_source=tf_static`：只订阅保留的静态 TF，成功取得完整刚性链后立即释放订阅和 Buffer。动态云台变换不会被误缓存；缺外参不发布，不使用零外参兜底。

[C++ 看门狗](../../xxu_nav_2026/src/xxu_bringup/src/cmd_vel_watchdog.cpp) 使用墙钟 Timer，保留上述命令、源时间、传感器、TF、定位失信和恢复保持行为。它直接接收两个受控发布者的数据流，并向标准 `/tf` 转发有效动态变换：

| 发布者 | 专用 TF 输出 |
| --- | --- |
| 底盘桥接 | `/sentry/tf/odometry` |
| 定位后端 | `/sentry/tf/localization` |

看门狗无需 `TransformListener`，也不订阅全局 `/tf`；仍校验连接和动态边接收新鲜度，并拒绝不同父节点冲突。它只向低频监督发布 `~/localization_ready` 与 `~/tf_ready`：可信定位就绪只在启用定位保护且状态合格时发布；仅 TF 就绪不替代可信定位。剩余 Python 导航监督不再订阅传感器、里程计或 TF，保留动作取消/归属处理。

当前 TF 实现以补充任务的 [TF 瘦身核验](tf_tree_audit_20261116.md) 为准：机械模型发布者已移出导航，安装外参直接连接 `base_link`，`base_footprint` 已移除；前端拒绝开启独立 TF，ZLIO 后端不广播兼容 `map → body`。底盘桥默认 30 Hz，专用流经看门狗只转发核心动态边。初轮 C++ 迁移和后续瘦身验证分开记录。

### 点云、IMU 与扫描

[前端](../../xxu_nav_2026/src/z-localization/src/frontend/apps/node.cpp) 直接发布独立 `/cloud_deskewed`（`PointCloud2`），只发有效激光更新，保留原时间、局部坐标与有限点验证、单调性及 `0.5 s` 源年龄保护。后端原子帧仍使用原 `imu` 标签；已补偿刚性传感器的组合入口显式设置 `deskewed_cloud_frame=lio_base_sensor`（实车取安装文件的 sensor），这只是原有同坐标系契约的明确命名，不添加可能冲突的 `imu` TF 边，也不进行二次几何变换。

[点云预处理](../../xxu_nav_2026/src/xxu_bringup/src/pointcloud_frame_republisher.cpp)、[IMU 补偿](../../xxu_nav_2026/src/xxu_bringup/src/gimbal_imu_compensator.cpp) 均为 C++。保留逐点时间、组织行填充、字节序、非 XYZ 数据、车体自滤除、关节插值和 IMU 杆臂/协方差转换；固定外参仅启动期读取。动态云台使用关节插值，不再通过全局动态 TF 逐点查询。仿真十点平均仍要求固定传感器和精确连续 `1 ms` 采样；实车保持默认单样本模式。

扫描直接订阅 `/cloud_deskewed`，`target_frame=base_link`；静态坐标变换缓存后释放 TF 订阅。点云和高频里程计两端统一为 Best Effort/Keep Last 的 `SensorDataQoS`，包括 Livox 发布端、前端、后端与扫描。会话/状态的保留消息及小体积控制输出继续使用各自必要的可靠语义。扫描处理尊重行步长和字节序，避免把填充数据视为点。

原自建 Python 高频节点的源码、安装项和启动项已移除。仿真底盘/云台及实车串口桥也有 C++ 对应组件；串口协议的纯 Python 编解码/状态机保留为离线参考和测试工具，不是 ROS 运行节点。

### Jazzy 与构建依赖

替换了相关 TF、message_filters 等存在标准替代的弃用 `.h` 包含为 `.hpp`。PCL、GTSAM、yaml-cpp 和系统 C API 本身的有效 `.h` 不是弃用 ROS 包装头，没有盲目改成不存在的 `.hpp`。

前后端显式链接 `Eigen3::Eigen`，要求系统 Eigen 3.4。后端配置期编译检查要求 GTSAM 使用系统 Eigen 且版本宏相符，不匹配即停止配置。当前本机 GTSAM 4.2 使用 `GTSAM_USE_SYSTEM_EIGEN=ON`、`GTSAM_BUILD_WITH_MARCH_NATIVE=OFF`；本次复用该依赖，没有声称重新构建 GTSAM。现有 PCL/SDK2 依赖已满足构建，未安装依赖。

### 实车组合入口

[sentry_localization.launch.py](../../xxu_nav_2026/src/xxu_bringup/launch/sentry_localization.launch.py) 在同一个 `component_container_mt` 中装载底盘桥接与看门狗，启动 C++ 预处理、前后端和扫描，并在节点前设置 `OMP_NUM_THREADS=2`。安装文件须确认测量、以 `base_link` 为基准；前端临时配置只覆盖已预处理输入所需的单位雷达/IMU外参，不改原配置文件。驱动关闭时不会打开硬件；驱动开启须显式提供设备网络 JSON。

在 `xxu_nav_2026` 目录构建并加载环境后，使用真实文件：

```bash
ros2 launch xxu_bringup sentry_localization.launch.py \
  mount_file:=/path/to/measured_mount.yaml \
  frontend_config:=/path/to/lio.yaml \
  backend_config:=/path/to/backend.yaml \
  prior_map:=/path/to/matching_prior \
  driver:=true driver_config:=/path/to/MID360_config.json
```

该入口不启动底盘串口或自动发送目标。固定安装没有关节反馈时，须根据实际硬件契约显式选择 `require_joint_states:=false`；默认保留原关节新鲜度保护。已有静态发布者拥有直接安装 TF 时使用 `publish_mount_tf:=false`，并确保外参与安装文件一致。不要并行运行两个定位/里程计 TF 发布链。

## Phase 5：实际验证结果

核验日期：2026-11-16。构建主机为 Intel Core i7-14650HX；以下结果不是 i5-7500 实机性能数据。

| 核验范围 | 当前结果 | 证据 |
| --- | --- | --- |
| 完整工作区 Release 构建 | 19/19 包完成，无编译或链接失败 | [构建日志](../../xxu_nav_2026/log/cpp_integration_20261116/release_build.log) |
| 前端、后端、底盘桥与串口协议 CTest | 分别 10、18、2、1 个测试目标通过 | [隔离域回归](../../xxu_nav_2026/log/cpp_integration_20261116/colcon_tests_isolated.log)、[逐项结果](../../xxu_nav_2026/log/cpp_integration_20261116/test_results.json) |
| 前端独立去畸变云与组合/重置协议 | 最终相关目标复测通过 | [复测日志](../../xxu_nav_2026/log/cpp_integration_20261116/final_frontend_bringup_tests.log) |
| 已安装 C++ 进程的集成测试 | 9/9；覆盖参数/静态外参、双组件装载、关节插值、点云布局、TF 断流、定位恢复与暂停时钟停车 | [集成与保护回归](../../xxu_nav_2026/log/cpp_integration_20261116/acceptance_tests.log) |
| 合成转换吞吐 | 3 秒发送/接收 600 IMU、30 点云、30 scan，即 200/10/10 Hz；不含估计器或硬件 | 同上日志中的 JSON |
| 串口协议与 C++ PTY 回环 | 11/11，通过二进制协议与故障保护回归；未连接真实底盘 | 同上 |
| 启动就绪、刚性安装、车辆参数与空闲补扫 | 2、6、15、9 项通过；刚性安装还委托执行相关 C++ 运行测试 | 同上 |
| 最后一次四元数边界检查 | C++ 两个测试目标通过；安装后的里程计逆外参/零全局 TF 订阅用例通过 | [单元复测](../../xxu_nav_2026/log/cpp_integration_20261116/final_unit_tests.log)、[里程计复测](../../xxu_nav_2026/log/cpp_integration_20261116/final_odometry_test.log) |
| 初轮 EGA 与两个展开 Xacro 模型（后续瘦身见专项核验） | 均只有 `base_link` 根、单父、无环，`base_footprint` 的父帧为 `base_link` | [模型检查](../../xxu_nav_2026/log/cpp_integration_20261116/tf_model_checks.json) |
| 改动范围静态检查 | 26 个 Python 文件语法、4 个 package.xml、158 个直接文档链接通过；启动/安装项不引用已删除节点，`git diff --check` 通过 | [静态检查](../../xxu_nav_2026/log/cpp_integration_20261116/static_checks.json) |

构建实际命令为 `colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release`。进程先加载 Jazzy，并将现有 `../z-localization/deps` 加入 CMake/动态库搜索路径；仅当前构建进程清除原 `LD_LIBRARY_PATH`，避免本机 MVS 自带 libusb 干扰，不改全局环境。为降低构建峰值内存设置 `MAKEFLAGS=-j1` 和 `CMAKE_BUILD_PARALLEL_LEVEL=1`。测试域使用 183 及从 186 开始的隔离域，不重启现有业务会话。

上述证据目录属于忽略的本机构建产物；报告和源码可进入版本管理，日志不会自动进入 Git。逐项目 XML/CTest 原始结果一并保存在该目录。生产入口静态检查还确认未引用已删除的 Python 可执行文件；保留的反向 `laserscan_to_pointcloud` 通用组件与离线标定工具不是本次部署链路，它们仍有 TF Listener，不能将本结论扩大为所有可选工具或 Nav2 均不读取 TF。

首轮 ROS 回归发现：可视化观察器请求 Reliable，与新 Best Effort 发布端不兼容；既有域分配器默认从 1 选择，而当前已有前端运行在域 1，导致会话身份串扰。修正观察器 QoS，将测试域分配改为从 186 起保留高位隔离域后复测。没有为使测试通过而放宽业务保护阈值或修改定位算法。[首次失败摘要](../../xxu_nav_2026/log/cpp_integration_20261116/initial_colcon_tests.log) 与最终结果分别保留；早期详细输出仍在工作区原 colcon 日志中。

## 实车核验命令与未验证项

重新启动所选新入口并加载同一安装环境后，在使用相同 ROS 域的独立终端检查：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /odom --once --field child_frame_id --qos-reliability best_effort
ros2 topic info /scan --verbose
ros2 topic hz /livox/imu --window 400 --wall-time
ros2 topic hz /livox/lidar --window 100 --wall-time
ros2 topic hz /cloud_deskewed --window 100 --wall-time
ros2 topic hz /scan --window 100 --wall-time
```

该 Jazzy 安装的 `ros2 topic hz` 已使用 Sensor Data QoS，CLI 不接受 `--qos-reliability`；上面的命令按实际帮助接口核对。实车目标是 IMU 约 `200 Hz`、原始点云约 `10 Hz`、扫描约 `10 Hz`。前端去畸变云按实际观测更新输出，不通过重复旧帧补足频率。`view_frames` 应显示单父无环的 `map → odom → base_link` 及真实安装的传感器子树，不再包含 `base_footprint`、轮子或云台的导航 TF。

后续已完成限定四目标 Gazebo 导航闭环，当前结果与告警见 [TF 瘦身核验的导航实测](tf_tree_audit_20261116.md#后续-gazebo-四目标导航实测2026-11-16)。尚未执行实车标定、i5-7500 压测、真实地图定位/重定位、长路线完整验收或真实传感器测频。未获得能复现和归因完整链路 `0.54 Hz` 的原始证据，因此本次确认删除 Python 解码/变换路径，不声称已实测解决该具体频率问题。没有重启或验收用户原有运行会话，没有 Git 提交、推送或发布。

## 修改与新增文件清单

本次新增 17、修改 57、删除 9 个文件（删除项为已替换的 Python 节点/测试）。以下仅列本任务文件；不包含用户原有文档修改、归档重命名和未跟踪资产。`A/M/D` 分别表示新增/修改/删除。

| 状态 | 文件 |
| --- | --- |
| A | [docs/validation/zlocalization_cpp_integration_20261116.md](zlocalization_cpp_integration_20261116.md) |
| M | [xxu_nav_2026/src/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp](../../xxu_nav_2026/src/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp) |
| M | [xxu_nav_2026/src/pb_omni_pid_pursuit_controller/include/pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp](../../xxu_nav_2026/src/pb_omni_pid_pursuit_controller/include/pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp) |
| M | [xxu_nav_2026/src/pb_omni_pid_pursuit_controller/src/goal_approach_controller.cpp](../../xxu_nav_2026/src/pb_omni_pid_pursuit_controller/src/goal_approach_controller.cpp) |
| M | [xxu_nav_2026/src/pointcloud_to_laserscan/CMakeLists.txt](../../xxu_nav_2026/src/pointcloud_to_laserscan/CMakeLists.txt) |
| M | [xxu_nav_2026/src/pointcloud_to_laserscan/include/pointcloud_to_laserscan/laserscan_to_pointcloud_node.hpp](../../xxu_nav_2026/src/pointcloud_to_laserscan/include/pointcloud_to_laserscan/laserscan_to_pointcloud_node.hpp) |
| M | [xxu_nav_2026/src/pointcloud_to_laserscan/include/pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp](../../xxu_nav_2026/src/pointcloud_to_laserscan/include/pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp) |
| M | [xxu_nav_2026/src/pointcloud_to_laserscan/package.xml](../../xxu_nav_2026/src/pointcloud_to_laserscan/package.xml) |
| M | [xxu_nav_2026/src/pointcloud_to_laserscan/src/laserscan_to_pointcloud_node.cpp](../../xxu_nav_2026/src/pointcloud_to_laserscan/src/laserscan_to_pointcloud_node.cpp) |
| M | [xxu_nav_2026/src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp](../../xxu_nav_2026/src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp) |
| M | [xxu_nav_2026/src/xxu_bringup/CMakeLists.txt](../../xxu_nav_2026/src/xxu_bringup/CMakeLists.txt) |
| M | [xxu_nav_2026/src/xxu_bringup/config/rigid_mount.pending.yaml](../../xxu_nav_2026/src/xxu_bringup/config/rigid_mount.pending.yaml) |
| M | [xxu_nav_2026/src/xxu_bringup/config/robot_interfaces.yaml](../../xxu_nav_2026/src/xxu_bringup/config/robot_interfaces.yaml) |
| M | [xxu_nav_2026/src/xxu_bringup/config/sensor_chain.yaml](../../xxu_nav_2026/src/xxu_bringup/config/sensor_chain.yaml) |
| A | [xxu_nav_2026/src/xxu_bringup/include/xxu_bringup/cloud_layout.hpp](../../xxu_nav_2026/src/xxu_bringup/include/xxu_bringup/cloud_layout.hpp) |
| A | [xxu_nav_2026/src/xxu_bringup/include/xxu_bringup/localization_health.hpp](../../xxu_nav_2026/src/xxu_bringup/include/xxu_bringup/localization_health.hpp) |
| A | [xxu_nav_2026/src/xxu_bringup/include/xxu_bringup/odometry_conversion.hpp](../../xxu_nav_2026/src/xxu_bringup/include/xxu_bringup/odometry_conversion.hpp) |
| A | [xxu_nav_2026/src/xxu_bringup/include/xxu_bringup/sensor_math.hpp](../../xxu_nav_2026/src/xxu_bringup/include/xxu_bringup/sensor_math.hpp) |
| M | [xxu_nav_2026/src/xxu_bringup/launch/rigid_sensor.launch.py](../../xxu_nav_2026/src/xxu_bringup/launch/rigid_sensor.launch.py) |
| A | [xxu_nav_2026/src/xxu_bringup/launch/sentry_localization.launch.py](../../xxu_nav_2026/src/xxu_bringup/launch/sentry_localization.launch.py) |
| M | [xxu_nav_2026/src/xxu_bringup/launch/zlio_base_adapter.launch.py](../../xxu_nav_2026/src/xxu_bringup/launch/zlio_base_adapter.launch.py) |
| M | [xxu_nav_2026/src/xxu_bringup/package.xml](../../xxu_nav_2026/src/xxu_bringup/package.xml) |
| D | `xxu_nav_2026/src/xxu_bringup/scripts/cmd_vel_watchdog.py` |
| D | `xxu_nav_2026/src/xxu_bringup/scripts/gimbal_imu_compensator.py` |
| M | [xxu_nav_2026/src/xxu_bringup/scripts/nav2_navigation_startup.py](../../xxu_nav_2026/src/xxu_bringup/scripts/nav2_navigation_startup.py) |
| D | `xxu_nav_2026/src/xxu_bringup/scripts/pointcloud_frame_republisher.py` |
| D | `xxu_nav_2026/src/xxu_bringup/scripts/wait_for_odom.py` |
| D | `xxu_nav_2026/src/xxu_bringup/scripts/zlio_base_odometry.py` |
| A | [xxu_nav_2026/src/xxu_bringup/src/cmd_vel_watchdog.cpp](../../xxu_nav_2026/src/xxu_bringup/src/cmd_vel_watchdog.cpp) |
| A | [xxu_nav_2026/src/xxu_bringup/src/gimbal_imu_compensator.cpp](../../xxu_nav_2026/src/xxu_bringup/src/gimbal_imu_compensator.cpp) |
| A | [xxu_nav_2026/src/xxu_bringup/src/pointcloud_frame_republisher.cpp](../../xxu_nav_2026/src/xxu_bringup/src/pointcloud_frame_republisher.cpp) |
| A | [xxu_nav_2026/src/xxu_bringup/src/wait_for_odom.cpp](../../xxu_nav_2026/src/xxu_bringup/src/wait_for_odom.cpp) |
| A | [xxu_nav_2026/src/xxu_bringup/src/zlio_base_odometry.cpp](../../xxu_nav_2026/src/xxu_bringup/src/zlio_base_odometry.cpp) |
| A | [xxu_nav_2026/src/xxu_bringup/test/sentry_bridge_test.cpp](../../xxu_nav_2026/src/xxu_bringup/test/sentry_bridge_test.cpp) |
| D | `xxu_nav_2026/src/xxu_bringup/test/test_zlio_base_odometry.py` |
| M | [xxu_nav_2026/src/xxu_bringup/xxu_bringup/sensor_launch.py](../../xxu_nav_2026/src/xxu_bringup/xxu_bringup/sensor_launch.py) |
| M | [xxu_nav_2026/src/xxu_bringup/xxu_bringup/velocity_launch.py](../../xxu_nav_2026/src/xxu_bringup/xxu_bringup/velocity_launch.py) |
| M | [xxu_nav_2026/src/xxu_description/urdf/xxu.urdf.xacro](../../xxu_nav_2026/src/xxu_description/urdf/xxu.urdf.xacro) |
| M | [xxu_nav_2026/src/xxu_description/urdf/xxu_gazebo.urdf.xacro](../../xxu_nav_2026/src/xxu_description/urdf/xxu_gazebo.urdf.xacro) |
| M | [xxu_nav_2026/src/xxu_hardware/CMakeLists.txt](../../xxu_nav_2026/src/xxu_hardware/CMakeLists.txt) |
| A | [xxu_nav_2026/src/xxu_hardware/include/xxu_hardware/chassis_protocol.hpp](../../xxu_nav_2026/src/xxu_hardware/include/xxu_hardware/chassis_protocol.hpp) |
| M | [xxu_nav_2026/src/xxu_hardware/launch/chassis_lower_bridge.launch.py](../../xxu_nav_2026/src/xxu_hardware/launch/chassis_lower_bridge.launch.py) |
| M | [xxu_nav_2026/src/xxu_hardware/package.xml](../../xxu_nav_2026/src/xxu_hardware/package.xml) |
| D | `xxu_nav_2026/src/xxu_hardware/scripts/chassis_lower_bridge.py` |
| A | [xxu_nav_2026/src/xxu_hardware/src/chassis_lower_bridge.cpp](../../xxu_nav_2026/src/xxu_hardware/src/chassis_lower_bridge.cpp) |
| A | [xxu_nav_2026/src/xxu_hardware/test/chassis_protocol_test.cpp](../../xxu_nav_2026/src/xxu_hardware/test/chassis_protocol_test.cpp) |
| M | [xxu_nav_2026/src/xxu_sim_bringup/CMakeLists.txt](../../xxu_nav_2026/src/xxu_sim_bringup/CMakeLists.txt) |
| M | [xxu_nav_2026/src/xxu_sim_bringup/launch/rm_sim_gazebo.launch.py](../../xxu_nav_2026/src/xxu_sim_bringup/launch/rm_sim_gazebo.launch.py) |
| M | [xxu_nav_2026/src/xxu_sim_bringup/launch/rmul_navigation.launch.py](../../xxu_nav_2026/src/xxu_sim_bringup/launch/rmul_navigation.launch.py) |
| M | [xxu_nav_2026/src/xxu_sim_bringup/models/ega_folded/robot.urdf](../../xxu_nav_2026/src/xxu_sim_bringup/models/ega_folded/robot.urdf) |
| M | [xxu_nav_2026/src/xxu_sim_bringup/package.xml](../../xxu_nav_2026/src/xxu_sim_bringup/package.xml) |
| D | `xxu_nav_2026/src/xxu_sim_bringup/scripts/gimbal_stabilizer.py` |
| D | `xxu_nav_2026/src/xxu_sim_bringup/scripts/rm_sim_adapter.py` |
| A | [xxu_nav_2026/src/xxu_sim_bringup/src/simulation_adapters.cpp](../../xxu_nav_2026/src/xxu_sim_bringup/src/simulation_adapters.cpp) |
| M | [xxu_nav_2026/src/z-localization/src/backend/CMakeLists.txt](../../xxu_nav_2026/src/z-localization/src/backend/CMakeLists.txt) |
| M | [xxu_nav_2026/src/z-localization/src/backend/apps/node.hpp](../../xxu_nav_2026/src/z-localization/src/backend/apps/node.hpp) |
| M | [xxu_nav_2026/src/z-localization/src/backend/apps/node_input.cpp](../../xxu_nav_2026/src/z-localization/src/backend/apps/node_input.cpp) |
| M | [xxu_nav_2026/src/z-localization/src/backend/apps/publisher.cpp](../../xxu_nav_2026/src/z-localization/src/backend/apps/publisher.cpp) |
| M | [xxu_nav_2026/src/z-localization/src/backend/launch/backend.launch.py](../../xxu_nav_2026/src/z-localization/src/backend/launch/backend.launch.py) |
| M | [xxu_nav_2026/src/z-localization/src/backend/test/ros/test_calibration_launch.py](../../xxu_nav_2026/src/z-localization/src/backend/test/ros/test_calibration_launch.py) |
| M | [xxu_nav_2026/src/z-localization/src/backend/test/ros/test_map_service.py](../../xxu_nav_2026/src/z-localization/src/backend/test/ros/test_map_service.py) |
| M | [xxu_nav_2026/src/z-localization/src/backend/tools/calibrate.cpp](../../xxu_nav_2026/src/z-localization/src/backend/tools/calibrate.cpp) |
| M | [xxu_nav_2026/src/z-localization/src/bringup/launch/bringup.launch.py](../../xxu_nav_2026/src/z-localization/src/bringup/launch/bringup.launch.py) |
| M | [xxu_nav_2026/src/z-localization/src/bringup/test/test_composition.py](../../xxu_nav_2026/src/z-localization/src/bringup/test/test_composition.py) |
| M | [xxu_nav_2026/src/z-localization/src/bringup/test/test_reset_protocol.py](../../xxu_nav_2026/src/z-localization/src/bringup/test/test_reset_protocol.py) |
| M | [xxu_nav_2026/src/z-localization/src/frontend/CMakeLists.txt](../../xxu_nav_2026/src/z-localization/src/frontend/CMakeLists.txt) |
| M | [xxu_nav_2026/src/z-localization/src/frontend/apps/node.cpp](../../xxu_nav_2026/src/z-localization/src/frontend/apps/node.cpp) |
| M | [xxu_nav_2026/src/z-localization/src/frontend/apps/node.hpp](../../xxu_nav_2026/src/z-localization/src/frontend/apps/node.hpp) |
| M | [xxu_nav_2026/src/z-localization/src/frontend/apps/visualization.cpp](../../xxu_nav_2026/src/z-localization/src/frontend/apps/visualization.cpp) |
| M | [xxu_nav_2026/src/z-localization/src/frontend/launch/lio.launch.py](../../xxu_nav_2026/src/z-localization/src/frontend/launch/lio.launch.py) |
| M | [xxu_nav_2026/src/z-localization/src/frontend/test/ros/test_lifecycle.py](../../xxu_nav_2026/src/z-localization/src/frontend/test/ros/test_lifecycle.py) |
| M | [xxu_nav_2026/src/z-localization/src/frontend/test/ros/test_sensor_pipeline.py](../../xxu_nav_2026/src/z-localization/src/frontend/test/ros/test_sensor_pipeline.py) |
| M | [xxu_nav_2026/src/z-localization/src/frontend/test/ros/test_visualization.py](../../xxu_nav_2026/src/z-localization/src/frontend/test/ros/test_visualization.py) |
| M | [xxu_nav_2026/src/z-localization/src/livox_ros_driver2/src/lddc.cpp](../../xxu_nav_2026/src/z-localization/src/livox_ros_driver2/src/lddc.cpp) |
| M | [xxu_nav_2026/tools/lio/README.md](../../xxu_nav_2026/tools/lio/README.md) |
| M | [xxu_nav_2026/tools/navigation/README.md](../../xxu_nav_2026/tools/navigation/README.md) |
| M | [xxu_nav_2026/tools/navigation/cad/prepare_folded_robot.py](../../xxu_nav_2026/tools/navigation/cad/prepare_folded_robot.py) |
| M | [xxu_nav_2026/tools/navigation/check_ega_navigation.py](../../xxu_nav_2026/tools/navigation/check_ega_navigation.py) |
| M | [xxu_nav_2026/tools/navigation/test_chassis_lower_bridge.py](../../xxu_nav_2026/tools/navigation/test_chassis_lower_bridge.py) |
| A | [xxu_nav_2026/tools/navigation/test_cpp_bridge_pipeline.py](../../xxu_nav_2026/tools/navigation/test_cpp_bridge_pipeline.py) |
| M | [xxu_nav_2026/tools/navigation/test_review_repairs.py](../../xxu_nav_2026/tools/navigation/test_review_repairs.py) |
| M | [xxu_nav_2026/tools/navigation/test_rigid_mount.py](../../xxu_nav_2026/tools/navigation/test_rigid_mount.py) |
| M | [xxu_nav_2026/tools/navigation/test_zlio_navigation.py](../../xxu_nav_2026/tools/navigation/test_zlio_navigation.py) |

本地忽略的 [PROJECT_INDEX.md](../../PROJECT_INDEX.md) 仅新增本报告路由；证据目录及构建/安装产物不计入上表。
