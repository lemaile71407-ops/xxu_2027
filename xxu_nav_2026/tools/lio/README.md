# ZLIO 输入、预处理与录包

当前全项目只保留 ZLIO，Small Point-LIO 及其旧仿真/实车启动链已于 2026-11-11 撤除。当前入口及测试边界见[导航主线](../navigation/README.md#当前主线与收敛范围)。旧速度噪声分析和旧故障话题记录器随旧链归档；历史结果仍以对应验证报告为准。

## 预处理工具归属

点云、IMU 补偿和坐标重发布保留在 `xxu_bringup`。[sensor_launch.py](../../src/xxu_bringup/xxu_bringup/sensor_launch.py) 仅构造预处理与安装 TF，不选择或启动 LIO。原平台校准值来自 [sensor_chain.yaml](../../src/xxu_bringup/config/sensor_chain.yaml)，默认话题/坐标系来自 [robot_interfaces.yaml](../../src/xxu_bringup/config/robot_interfaces.yaml)；实车刚性安装独立使用下节的安装配置。

固定折叠 EGA 仍使用既有外参和杆臂补偿；ZLIO/TDT 由统一 bringup 启动，`zlio_base_adapter` 输出底盘里程计和动态 TF。保留实车预处理函数不代表已完成实车 ZLIO 接入。

## 实车刚性安装与多底盘配置

2026-11-12 用户确认当前雷达刚性安装在底盘上，安装距离待测；不沿用旧云台尺寸。新增 [rigid_sensor.launch.py](../../src/xxu_bringup/launch/rigid_sensor.launch.py) 复用现有点云变换、固定 IMU 处理和底盘适配器，不依赖 EGA CAD、轮子数量、运动学或 `/joint_states`，不启动硬件驱动、定位后端、Nav2 或底盘控制。原云台与 RMUL 仿真路径保留。

每种底盘/安装位置复制一份 [rigid_mount.pending.yaml](../../src/xxu_bringup/config/rigid_mount.pending.yaml)，作为该安装外参的唯一来源；填入测量值后设置 `calibration_confirmed: true`。模板中的 `null` 是待测值；未经确认、缺少有限测量值、重复 TF 帧或输入输出话题环路时，入口在创建节点前报错。

| 配置 | 含义 |
| --- | --- |
| `frames.base` | 输出里程计的底盘坐标系，与底盘导航配置一致，默认 `base_footprint` |
| `frames.sensor` | 刚性虚拟传感器，原点必须与物理 IMU 原点重合，方向自行选定，默认 `lio_base_sensor` |
| `frames.lidar` / `frames.imu_source` | 驱动点云 / IMU 消息帧名；帧名不代表已确认其测量轴约定 |
| `base_to_sensor` | 底盘坐标系中虚拟传感器的位姿，`base ← sensor` |
| `sensor_to_lidar` | 虚拟传感器中点云坐标系的位姿，`sensor ← lidar`，包含雷达与 IMU 的真实内外参 |
| `sensor_from_imu_rpy` | 原始 IMU 测量轴转到虚拟传感器轴的旋转；两者原点重合，不估计杆臂加速度 |
| `imu_acceleration_scale` | 转为 m/s² 的倍率，输入为 g 时填 `9.81`，m/s² 时填 `1.0`，按真实驱动核验 |
| `topics` | 输入及输出的点云/IMU 话题，各底盘可独立指定 |

`xyz` 单位为米，`rpy` 为固定轴 roll/pitch/yaw，单位弧度。入口发布静态链 `base → sensor → lidar`；点云转换到 `sensor`，保留逐点时间和其他字段。固定外参无需逐点查询动态 TF。IMU 只旋转测量和协方差、换算单位，不订阅关节。换底盘只选择另一份安装文件；底盘轮廓、速度限制与驱动协议仍由对应底盘配置负责，本次不扩展底盘运动学支持。

构建并加载含本次改动的 `xxu_bringup` 安装环境后，选择已经测量的文件：

```bash
ros2 launch xxu_bringup rigid_sensor.launch.py mount_file:=/path/to/measured_mount.yaml
```

默认同时启动现有底盘适配器，向它传递配置的 `sensor` 和 `base`，发布 `/odom` 与 `odom → base`。`start_base_adapter:=false` 只启 TF 与预处理；`input_topic`、`output_topic`、`odom_frame`、`source_child_frame` 沿用底盘适配器参数。若底盘 robot_state_publisher 已发布这两段 TF，使用 `publish_mount_tf:=false`，确保模型 TF 与安装配置一致且相对关系恒定。每条 TF 只允许一个发布者。本入口不发布车轮、云台或 `base_footprint → base_link`，这些由选定底盘负责。

ZLIO 应订阅本配置的两个输出话题，将 `sensor.body_from_lidar` 设为**单位旋转、零平移**（输入已统一到同一虚拟传感器），并设 `imu_acceleration_scale:=1.0`、`publish_tf:=false`，避免重复外参变换或单位换算。点内时间字段/尺度须按真实驱动选择；后端负责 `map → odom`。更换安装后须核验先验地图、初始化位姿和后端外参约定，不能套用 EGA 仿真地图参数。

验证方法：加载 ROS Jazzy 后运行 `python3 tools/navigation/test_rigid_mount.py`。测试使用合成外参及隔离域 194 的 IMU 节点构造，不连接硬件。实际验证范围见[刚性安装核验](../../../docs/validation/rigid_sensor_mount_20261112.md)，待测模板不是实车标定文件。

### 实车与仿真 IMU 采样边界

- **仿真为什么是 `1000 Hz → 100 Hz` 十点平均（仅 `ega_folded`）**：Gazebo 物理步长为 `1 ms`，原 `100 Hz` IMU 每 10 步只做一次无滤波瞬时点采样，会漏掉轮地碰撞或转向期间的高频加速度变化（见[Stage 3 验证](../../../docs/validation/nav2_stage3_20261108.md)同段无噪声闭合对照：直接 `100 Hz` 抽样水平速度闭合误差最高约 `0.547 m/s`，连续 `1000 Hz` 与十点平均均 `< 1e-6 m/s`）。使用 `imu_sampling:=interval_mean` 时，`rm_sim_gazebo.launch.py` 仅允许 `robot_model:=ega_folded`，将临时 SDF 采样率升至 `1000 Hz` 并把单样本噪声标准差放大 `√10` 倍；随后 `gimbal_imu_compensator.py`（`imu_interval_samples:=10`）对连续 10 条 `1 ms` 样本求平均、取时间中点并把协方差缩为 `1/10`，输出仍保持 `100 Hz`。这样既消除仿真漏采，又保持与下游 `100 Hz` 接口、回调负载及 `ega_imu_sample_noise.yaml`（`gyro_variance: 4.0e-8`、`accel_variance: 2.89e-4`）同一噪声强度，避免把“高频降噪”混算进对照组收益。
- **实车不需要且禁止开启 `interval_mean` / `imu_interval_samples:=10`**：
  - 代码与启动边界：`rigid_sensor.launch.py` 默认使用 `imu_interval_samples=1` 直通模式；`gimbal_imu_compensator.py` 在 `imu_interval_samples=10` 时强制校验相邻消息时间差严格等于 `1000000 ns`（`1 ms`）且 `frame_id` 一致，否则清空缓存，而真实 IMU 驱动无法保证 `1000 Hz` 且逐帧精确 `1.000000 ms` 步进，强行开启会反复清缓存丢数据。
  - 物理采样差异：一般工业 MEMS IMU 模块通常在硬件内部以更高频率采样并经抗混叠低通/增量滤波后再按驱动频率输出（属一般硬件工作机理说明），不同于 Gazebo 的无滤波瞬时点采样。
- **实车整链仍待现场核验的事项**：实车部署前须基于真实驱动与录包确认驱动输出频率、时间戳单调性与抖动、加速度单位（`m/s²` 或 `g`）及对应 `imu_acceleration_scale`，并实测静止/运动噪声方差以标定实车 `lio.yaml`（勿直接照搬仿真 `ega_imu_sample_noise.yaml`）；若实车存在强烈底盘共振且怀疑混叠，也应先录包检查驱动频率与速度积分闭合后再评估处理方案。

## ZLIO 底盘里程计候选适配

独立候选入口 `ros2 launch xxu_bringup zlio_base_adapter.launch.py` 订阅 ZLIO `/odometry`，查找消息时刻的 `lio_base_sensor ← base_footprint`，转换位姿、刚体速度及协方差，发布 `/odom` 和 `odom → base_footprint`。默认将 ZLIO 输出的 `imu` 标签对应到已补偿的 `lio_base_sensor`；只有两者确实表示同一刚性坐标系时才能这样配置。参数 `source_child_frame`、`sensor_frame`、`base_frame`、`odom_frame`、`input_topic`、`output_topic` 和 `use_sim_time` 可覆盖。

运行前必须保证虚拟传感器相对底盘刚性固定，TF 由机器人模型或上述刚性安装入口唯一发布；移动云台原始帧不满足刚体速度转换条件。输入帧名不符、时间为零或缺少对应时刻的 TF 时适配器不发布。候选运行时 ZLIO 应关闭自身 TF 发布，并停止其他 `/odom` 和 `odom → base_footprint` 发布者；后端负责 `map → odom`。该入口不启动 ZLIO、后端或导航，也不处理定位降级时的 Nav2 保护。[回放验证及未完成项](../../../docs/validation/zlio_base_tf_20261031.md)仍限制正式使用。

### 固定传感器的统一定位入口

现有 `bringup/bringup.launch.py` 和 `frontend/lio.launch.py` 支持显式传入 `point_time_field`、`point_time_scale`、`point_time_absolute` 和 `publish_tf`，默认值保持原行为。重新构建安装 `frontend`、`backend`、`bringup`、`xxu_bringup` 和 `xxu_livox_sim` 并加载工作空间后，可复用统一入口启动前后端；底盘适配仍用上面的已有入口。仿真底座不再内嵌估计器，ZLIO 只由统一定位入口启动一次。

在 `xxu_nav_2026` 工作空间目录中，以下参数仅适用于本次自建图、固定虚拟传感器和已知出生点 `(2,2,0)`；传感器预处理及仿真时钟须另行启动。它只启动定位，不启动 Nav2 或发送指令：

```bash
ros2 launch bringup bringup.launch.py \
  frontend_config:="$PWD/maps/rmul_self_20261109/lio.yaml" \
  backend_config:="$PWD/maps/rmul_self_20261109/backend.yaml" \
  prior_map:="$PWD/maps/rmul_self_20261109/prior" \
  mode:=localization use_sim_time:=true input_type:=pointcloud2 \
  lidar_topic:=/mid360/livox_points_compensated imu_topic:=/imu_lio_compensated \
  imu_acceleration_scale:=1.0 point_time_field:=timestamp \
  point_time_scale:=1e-9 point_time_absolute:=true publish_tf:=false
```

仿真逐点时间版本沿用现存补丁，可用 `tools/navigation/build_rgl_timed_candidate.sh --output-dir "$PWD/build/rgl_timed_runtime"` 首次构建到新目录，再通过 `rgl_plugin_dir` 显式选择其 `install/RGLServerPlugin`；目录已存在时脚本拒绝覆盖。默认 RGL 插件不含这项扩展，不能只打开 `rgl_point_timing:=uniform` 就视为输入时序已成立。该模式仍是均匀扫描时间模型，未成为真实 Mid360 时序标定。

当前安装与静止检查结果、GTSAM 路径及相机 SDK 动态库冲突的处理范围见[第三阶段续测记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-09-续测预检查)。`record_localization_jumps.py --tdt` 可额外只读记录 TDT 状态与全局位姿；`--safe-velocity-topic`、`--truth-topic` 可记录安全输出和仿真真值，均不发布数据或控制命令。

## 录制

先准备实际传感器、所需关节/TF，以及 ZLIO 和底盘适配输出，再加载 ROS 2 与工作空间。实车整套 ZLIO 启动尚待验证，此工具只负责录制，不启动估计器。
每个工况单独录一个包：

```bash
tools/lio/record_mid360_regression.sh locked_stationary --duration 20
tools/lio/record_mid360_regression.sh rotating_stationary --duration 20
tools/lio/record_mid360_regression.sh locked_motion
tools/lio/record_mid360_regression.sh rotating_motion
```

运动组应走同一条可测量的路线，旋转组使用云台正常工作速度；不要为了复现仿真
而强行把实机设为 4 rad/s。如果底盘或动捕提供独立里程计，增加例如：

```bash
tools/lio/record_mid360_regression.sh rotating_motion \
  --reference-odom /wheel_odom
```

为避免录包进程与 LIO 抢占 CPU，默认 MCAP 使用低开销 `fastwrite`，且不录
`/cloud_deskewed` 等重型派生点云。如确实需要检查派生云，再加
`--include-derived-clouds`；存储空间受限时可另选 `--storage-profile zstd_fast`。

默认关键话题为 `/livox/lidar`、`/livox/imu`、`/joint_states` 和 `/odom`，
均可通过脚本参数覆盖。缺少任一关键话题时脚本会拒绝开始，避免得到无法回放的包。
每个包还包含采集时的 Git commit、话题清单和 ZLIO 参数快照。

## 检查

```bash
tools/lio/inspect_mid360_bag.py bags/lio_regression/<bag-directory>
```

检查器输出：

- 点云频率、每帧点数、有限坐标比例；
- 点内时间字段、单帧时间跨度、时间组数量，以及相邻帧的间隙、重叠和共享时间组；
- IMU 与云台关节频率；底盘静止时还可比较两者角速度；
- `/odom` 的路径长度、净位移和相对起点最大漂移；
- 可选独立里程计的同类统计。

静止旋转包首先看 `max_xy_from_start`；两组运动包再比较 LIO 与实测路线或独立里程计
的净位移。这样可将“旋转时漂移”拆分为输入时序/格式问题、静止旋转漂移和运动尺度
误差三个层次。
