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
| `frames.base` | 输出里程计的底盘坐标系，与底盘导航配置一致，默认 `base_link` |
| `frames.sensor` | 刚性虚拟传感器，原点必须与物理 IMU 原点重合，方向自行选定，默认 `lio_base_sensor` |
| `frames.lidar` / `frames.imu_source` | 驱动点云 / IMU 消息帧名；帧名不代表已确认其测量轴约定 |
| `base_to_sensor` | 底盘坐标系中虚拟传感器的位姿，`base ← sensor` |
| `sensor_to_lidar` | 虚拟传感器中点云坐标系的位姿，`sensor ← lidar`，包含雷达与 IMU 的真实内外参 |
| `sensor_from_imu_rpy` | 原始 IMU 测量轴转到虚拟传感器轴的旋转；两者原点重合，不估计杆臂加速度 |
| `imu_acceleration_scale` | 转为 m/s² 的倍率，输入为 g 时填 `9.81`，m/s² 时填 `1.0`，按真实驱动核验 |
| `topics` | 输入及输出的点云/IMU 话题，各底盘可独立指定 |

`xyz` 单位为米，`rpy` 为固定轴 roll/pitch/yaw，单位弧度。入口将安装链合成为 `base → sensor` 与 `base → lidar` 两条直接静态外参；点云转换到 `sensor`，保留逐点时间和其他字段。固定外参无需逐点查询动态 TF。IMU 只旋转测量和协方差、换算单位，不订阅关节。换底盘只选择另一份安装文件；底盘轮廓、速度限制与驱动协议仍由对应底盘配置负责，本次不扩展底盘运动学支持。

构建并加载含本次改动的 `xxu_bringup` 安装环境后，选择已经测量的文件：

```bash
ros2 launch xxu_bringup rigid_sensor.launch.py mount_file:=/path/to/measured_mount.yaml
```

默认同时启动现有底盘适配器，向它传递配置的 `sensor` 和 `base`，发布 `/odom` 与 `odom → base`。`start_base_adapter:=false` 只启 TF 与预处理；`input_topic`、`output_topic`、`odom_frame`、`source_child_frame` 沿用底盘适配器参数。若其他静态发布者已拥有这些直接外参，使用 `publish_mount_tf:=false`，确保 TF 与安装配置一致。每条 TF 只允许一个发布者；导航入口不运行机械模型 TF，不保留 `base_footprint`。

ZLIO 应订阅本配置的两个输出话题，将 `sensor.body_from_lidar` 设为**单位旋转、零平移**（输入已统一到同一虚拟传感器），并设 `imu_acceleration_scale:=1.0`、`publish_tf:=false`，避免重复外参变换或单位换算。点内时间字段/尺度须按真实驱动选择；后端负责 `map → odom`。更换安装后须核验先验地图、初始化位姿和后端外参约定，不能套用 EGA 仿真地图参数。

验证方法：加载 ROS Jazzy 后运行 `python3 tools/navigation/test_rigid_mount.py`。测试使用合成外参、隔离域中的已安装 C++ 节点，不连接硬件。C++ 迁移后的验证范围见[本次核验](../../../docs/validation/zlocalization_cpp_integration_20261116.md)；[原刚性安装核验](../../../docs/validation/rigid_sensor_mount_20261112.md)为历史证据，待测模板不是实车标定文件。

### 实车与仿真 IMU 采样边界

- **仿真为什么是 `1000 Hz → 100 Hz` 十点平均（仅 `ega_folded`）**：Gazebo 物理步长为 `1 ms`，原 `100 Hz` IMU 每 10 步只做一次无滤波瞬时点采样，会漏掉轮地碰撞或转向期间的高频加速度变化（见[Stage 3 验证](../../../docs/validation/nav2_stage3_20261108.md)同段无噪声闭合对照：直接 `100 Hz` 抽样水平速度闭合误差最高约 `0.547 m/s`，连续 `1000 Hz` 与十点平均均 `< 1e-6 m/s`）。使用 `imu_sampling:=interval_mean` 时，`rm_sim_gazebo.launch.py` 仅允许 `robot_model:=ega_folded`，将临时 SDF 采样率升至 `1000 Hz` 并把单样本噪声标准差放大 `√10` 倍；随后 `gimbal_imu_compensator`（C++）（`imu_interval_samples:=10`）对连续 10 条 `1 ms` 样本求平均、取时间中点并把协方差缩为 `1/10`，输出仍保持 `100 Hz`。这样既消除仿真漏采，又保持与下游 `100 Hz` 接口、回调负载及 `ega_imu_sample_noise.yaml`（`gyro_variance: 4.0e-8`、`accel_variance: 2.89e-4`）同一噪声强度，避免把“高频降噪”混算进对照组收益。
- **实车不需要且禁止开启 `interval_mean` / `imu_interval_samples:=10`**：
  - 代码与启动边界：`rigid_sensor.launch.py` 默认使用 `imu_interval_samples=1` 直通模式；`gimbal_imu_compensator`（C++） 在 `imu_interval_samples=10` 时强制校验相邻消息时间差严格等于 `1000000 ns`（`1 ms`）且 `frame_id` 一致，否则清空缓存，而真实 IMU 驱动无法保证 `1000 Hz` 且逐帧精确 `1.000000 ms` 步进，强行开启会反复清缓存丢数据。
  - 物理采样差异：一般工业 MEMS IMU 模块通常在硬件内部以更高频率采样并经抗混叠低通/增量滤波后再按驱动频率输出（属一般硬件工作机理说明），不同于 Gazebo 的无滤波瞬时点采样。
- **实车整链仍待现场核验的事项**：实车部署前须基于真实驱动与录包确认驱动输出频率、时间戳单调性与抖动、加速度单位（`m/s²` 或 `g`）及对应 `imu_acceleration_scale`，并实测静止/运动噪声方差以标定实车 `lio.yaml`（勿直接照搬仿真 `ega_imu_sample_noise.yaml`）；若实车存在强烈底盘共振且怀疑混叠，也应先录包检查驱动频率与速度积分闭合后再评估处理方案。

## C++ 定位与底盘适配

实车刚性安装组合入口为 `ros2 launch xxu_bringup sentry_localization.launch.py`，必须显式传入 `mount_file`、`frontend_config`、`backend_config`；`mode:=localization`（默认）还需传入匹配的 `prior_map`，`mode:=mapping` 可不传 `prior_map`。`mount_file` 要求已测量确认且 `frames.base=base_link`；待测模板不能启动。`driver:=true` 时另需设备网络配置 `driver_config`，驱动输出 PointCloud2 并同时接收 `frame_id`（`frames.lidar`）与 `imu_frame_id`（`frames.imu_source`），预处理、前端、底盘桥接、看门狗和扫描均为 C++。组合入口不启动底盘硬件或下发目标。默认参数适配刚性实车：`require_joint_states:=false`（不要求 `/joint_states`，接入关节发布器时可显式传 `true`），`enable_watchdog:=auto`（仅在 `mode:=localization` 启动 `cmd_vel_watchdog`；`mode:=mapping` 或 `enable_watchdog:=false` 时不启看门狗避免 100 Hz 零速抢占 `/cmd_vel`，同时里程计与后端直接向 `/tf` 发布动态变换）。`self_filter_geometry_file`（`rigid_sensor.launch.py` 与 `sentry_localization.launch.py` 均支持）和 `scan_self_filter_radius`（`sentry_localization.launch.py` 支持，默认 `0.0`）默认关闭，按需显式传入即可。原始 CustomMsg 仍可由统一前端 `input_type:=livox` 直接接收；不要给 PointCloud2 入口发送 CustomMsg。

前端直接发布局部 `/cloud_deskewed`（PointCloud2、SensorDataQoS），扫描节点直接消费它并输出 `base_link` 下的 `/scan`。前端的 TF 默认关闭，C++ 桥接发布 `/odom`，`child_frame_id=base_link`。刚性安装入口会生成仅覆盖前端雷达/IMU外参为单位变换的临时算法配置，避免输入预处理后再次应用外参；不修改原 YAML 的体素、噪声或时间调度参数。

独立入口 `ros2 launch xxu_bringup zlio_base_adapter.launch.py` 仍可使用，订阅 `/odometry`，从启动期 `/tf_static` 获取 `base_link ← lio_base_sensor` 并缓存，成功后释放订阅；不会读取动态云台外参，也不会回退成单位变换。它按 `T_odom_base = T_odom_sensor * inverse(T_base_sensor)` 转换位姿、刚体速度和协方差。默认明确将前端 `imu` 对应到固定虚拟传感器 `lio_base_sensor`，不新增 `imu` TF 边。组合入口通过前端 `deskewed_cloud_frame` 参数明确将独立导航云标注为同一个固定虚拟传感器；后端原子帧仍使用 `imu`。只有两者确实代表同一坐标系时才可配置这个别名。组件也支持 YAML 参数外参模式，参见[实现与本次核验](../../../docs/validation/zlocalization_cpp_integration_20261116.md)。

独立底盘入口默认直接发布 `/tf`，不启动前后端、扫描、看门狗或 Nav2。完整组合和 RMUL 主线将里程计与后端 TF 分别输出到 `/sentry/tf/odometry`、`/sentry/tf/localization`，由 C++ 看门狗只转发两条导航核心边到标准 `/tf`；自建桥接和看门狗均不订阅全局 `/tf`。输入帧错误、时间非法或外参尚未取得时不发布里程计。底盘 TF 默认 30 Hz，参数限定为 20–50 Hz；看门狗全局转发上限为每条边 50 Hz。仅对新数据采样，停流后不重播旧位姿。安装外参直接从 `base_link` 走 `/tf_static`，审计与复测见 [TF 瘦身核验](../../../docs/validation/tf_tree_audit_20261116.md)。实车外参、地图匹配及现场频率仍须实际验证；旧 Python 回放结果不能替代本次 C++ 验证。

### 固定传感器的统一定位入口

现有 `bringup/bringup.launch.py` 和 `frontend/lio.launch.py` 支持显式传入 `point_time_field`、`point_time_scale`、`point_time_absolute` 、`publish_tf` 和 `deskewed_cloud_frame`；前端 `publish_tf` 仅接受 `false`，由底盘桥发布导航 TF；导航云坐标别名只在明确同一物理坐标系时设置。重新构建安装 `frontend`、`backend`、`bringup`、`xxu_bringup` 和 `xxu_livox_sim` 并加载工作空间后，可复用统一入口启动前后端；底盘适配仍用上面的已有入口。仿真底座不再内嵌估计器，ZLIO 只由统一定位入口启动一次。

在 `xxu_nav_2026` 工作空间目录中，以下参数仅适用于本次自建图、固定虚拟传感器和已知出生点 `(2,2,0)`；传感器预处理及仿真时钟须另行启动。它只启动定位，不启动 Nav2 或发送指令：

```bash
ros2 launch bringup bringup.launch.py \
  frontend_config:="$PWD/maps/rmul_rotating_20261112/lio.yaml" \
  backend_config:="$PWD/maps/rmul_rotating_20261112/backend.yaml" \
  prior_map:="$PWD/maps/rmul_rotating_20261112/prior" \
  mode:=localization use_sim_time:=true input_type:=pointcloud2 \
  lidar_topic:=/mid360/livox_points_compensated imu_topic:=/imu_lio_compensated \
  imu_acceleration_scale:=1.0 point_time_field:=timestamp \
  point_time_scale:=1e-9 point_time_absolute:=true publish_tf:=false
```

仿真逐点时间版本沿用现存补丁，可用 `tools/navigation/build_rgl_timed_candidate.sh --output-dir "$PWD/build/rgl_timed_runtime"` 首次构建到新目录，再通过 `rgl_plugin_dir` 显式选择其 `install/RGLServerPlugin`；目录已存在时脚本拒绝覆盖。默认 RGL 插件不含这项扩展，不能只打开 `rgl_point_timing:=uniform` 就视为输入时序已成立。该模式仍是均匀扫描时间模型，未成为真实 Mid360 时序标定。

当前安装与静止检查结果、GTSAM 路径及相机 SDK 动态库冲突的处理范围见[第三阶段续测记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-09-续测预检查)。`record_localization_jumps.py --tdt` 可额外只读记录 TDT 状态与全局位姿；`--safe-velocity-topic`、`--truth-topic` 可记录安全输出和仿真真值，均不发布数据或控制命令。

## 实车刚性外参标定流程（在线与离线录包）

先在平整地面上确认 `base_link` 原点和底盘正前方，量出该原点到地面的高度 `base_height_above_ground`。地面投影原点使用 `0.0` m。记录原始 IMU、原始雷达和**未做刚性预处理**的 ZLIO 前端里程计，其中位姿为 `T_odom_imu`。留出范围明确、无台阶的地板视野，等前端启动并连续输出后再做动作。

1. 原地静止至少 3 s。地板须在雷达视野中，手不要碰车或传感器。
2. 沿底盘 `+X` 连续直行或推行至少 1.5 m，保持航向，优先录约 3 m。若全程沿 `-X`，设置 `straight_direction:=-1` 或 CLI 的 `--straight-direction -1`。
3. 围绕同一个底盘原点原地转至少 180°，优先一整圈。避免平移、急停和明显侧倾。杆臂解的原点由旋转中心决定，因此这一动作的旋转中心必须与目标 `base_link` 原点重合。

### 在线采集

加载 ROS Jazzy 和工作空间后，启动隔离的采集入口。以下命令假定原始驱动已输出 PointCloud2。如果需要同时启动驱动，附加 `driver:=true driver_config:=/path/to/MID360_config.json`。原始驱动输出 CustomMsg 时使用 `lidar_input_type:=livox`，要求构建时检测到 `livox_ros_driver2`。若已有原始输入的 ZLIO 前端在运行，使用 `start_zlio:=false`。

`start_zlio:=true` 将标定用 ZLIO 前端的输出隔离到 `/calibration/...`。`start_zlio:=true` 和 `start_zlio:=false` 的标定节点都默认订阅 `/calibration/lidar_odometry`，可用 `odom_topic:=/指定话题` 覆盖。标定入口默认拒绝 `/lidar_odometry`、`/odometry`、`/frontend/odometry` 和 `/odom`，这些生产话题可能发布补偿后虚拟传感器的位姿。确认所选话题来自原始输入的 ZLIO 前端后，才可显式设置 `allow_unisolated_odom_topic:=true`。

物理 IMU 里程计的 `header.frame_id` 和 `child_frame_id` 默认应为 `odom` 和 `imu`。外部前端使用其他帧名时，设置 `odom_frame:=...` 和 `odom_child_frame:=...`。自启 ZLIO 读取 PointCloud2 时，可通过 `point_time_field`、`point_time_scale`、`point_time_absolute` 指定点时间字段、换算到秒的倍率和绝对时间模式，默认分别为 `timestamp`、`1e-9`、`true`。

```bash
calibration_dir="$(mktemp -d /tmp/xxu-mount.XXXXXX)"
base_height=0.0
ros2 launch xxu_bringup rigid_mount_calibration.launch.py \
  driver:=false start_zlio:=true lidar_input_type:=pointcloud2 \
  base_height_above_ground:="$base_height" confirm:=false \
  output_mount_file:="$calibration_dir/rigid_mount.yaml" \
  output_backend_body_file:="$calibration_dir/backend_body.yaml" \
  output_report_file:="$calibration_dir/calibration_report.yaml"
```

在另一终端执行采集服务。 `capture_stationary` 启动默认 3 s 的定时采集，等日志给出样本数和质量结果再继续。手动定长可用 `start_stationary` 和 `stop_stationary`。每次 `start_*` 替换该阶段上一次的采集；`reset` 清空全部阶段。`start_*`、`stop_*` 和 `reset` 会自动将 `confirm` 重置为 `false`，新采集或清空后的结果须重新审阅确认才能正式导出。转动或直行期间各阶段只采集对应的原始里程计。

```bash
cal() { ros2 service call "/rigid_mount_calibration/$1" std_srvs/srv/Trigger '{}'; }

cal capture_stationary
# 等待节点日志给出静止质量结果。
cal start_straight
# 完成直线动作。
cal stop_straight
cal start_spin
# 完成原地旋转。
cal stop_spin
cal solve_and_export
# 检查导出的报告后，显式请求确认并重新导出。
ros2 param set /rigid_mount_calibration confirm true
cal solve_and_export
```

采集节点独立识别 IMU 单位，`imu_acceleration_scale:=0.0` 表示自动。自启的 ZLIO 也需要正确的单位，标定倍率自动时它使用 `zlio_imu_acceleration_scale:=9.81`。已确认驱动输出 m/s² 时可设置 `imu_acceleration_scale:=1.0`，该值同时传给 ZLIO。配置中 `imu_from_lidar_xyz`、`imu_from_lidar_rpy` 定义 Mid-360 的物理 IMU ← 雷达变换。改用实际设备内参时同步传入与之匹配的 `frontend_config`。启动入口只启所选驱动、ZLIO 前端和标定节点。

### 离线录包与求解

以下命令录制三个连续动作，`rigid_mount_calibrate` 默认读取 `/calibration/lidar_odometry`。外部原始输入前端使用其他话题时，通过 `--odom-topic` 指定。CLI 同样默认拒绝 `/lidar_odometry`、`/odometry`、`/frontend/odometry` 和 `/odom`。确认所选话题来自原始输入的 ZLIO 前端后，才可显式加上 `--allow-unisolated-odom-topic`。也可分别录静止、直行、原地转动三个包，求解时传 `--stationary-bag`、`--straight-bag`、`--spin-bag`。工具根据录包元数据识别 PointCloud2 或 CustomMsg，支持 MCAP 和 SQLite3。

离线求解和独立复核均可用 `--odom-frame` 和 `--odom-child-frame` 指定物理 IMU 里程计的父帧和子帧，默认分别为 `odom` 和 `imu`。

```bash
calibration_dir="$(mktemp -d /tmp/xxu-mount.XXXXXX)"
base_height=0.0
odom_topic=/calibration/lidar_odometry  # 在线和离线标定的默认话题
ros2 bag record -s mcap -o "$calibration_dir/training" \
  /livox/imu /livox/lidar "$odom_topic"
# 完成三个动作后按 Ctrl+C 停止录包，再执行求解。
ros2 run xxu_bringup rigid_mount_calibrate \
  --bag "$calibration_dir/training" --auto-segment --confirm \
  --odom-topic "$odom_topic" \
  --base-height-above-ground "$base_height" \
  --output-mount "$calibration_dir/rigid_mount.yaml" \
  --output-backend-body "$calibration_dir/backend_body.yaml" \
  --output-report "$calibration_dir/calibration_report.yaml"
```

若自动分段报告缺段或窗口不合适，可用 `--stationary-window 0:5 --straight-window 8:15 --spin-window 20:32` 替换 `--auto-segment`，将示例窗口换成实际动作时间。窗口以录包首条消息的**接收时间**为零点，并包含两端。传 `--lidar-topic`、`--imu-topic`、`--odom-topic` 切换话题。驱动消息帧不同于 `livox_frame` 时，显式传 `--imu-source-frame` 和 `--lidar-frame`，并在报告内核对观测帧名。 `rigid_mount_calibrate --help` 列出完整阈值选项。IMU 噪声报告同时给出原始量和 SI 单位下的样本方差，可用于核对设备的 ZLIO 噪声设置。

有独立测量时，可使用 `--yaw-deg DEG`、`--base-to-sensor-x M`、`--base-to-sensor-y M`、`--base-to-sensor-z M` 单独覆盖各量。在线对应的字符串参数是 `manual_yaw_deg`、`manual_tx`、`manual_ty`、`manual_tz`，空字符串表示未设置，`'0'` 表示测量值为零。yaw 参数是调平坐标系上的偏航 `psi`，满足 `R_base_imu = Rz(psi) * R_level_imu`。确认要求显式请求、所有量齐全且采集的阶段均通过检查。只录静止时，缺测字段写 `null`，生产入口拒绝加载未确认文件。 `--confirm` 在量不全或质量检查失败时返回非零，并仍按已指定的路径导出诊断文件。`--output-backend-body` 仅在确认通过时写入，未确认或验证失败时不会新建或覆盖已有后端文件。

### 独立验证与接入

重新录一组动作到 `"$calibration_dir/validation"`。独立验证使用已有 YAML 外参计算底盘系的重力、地板法向/高度、直行航向误差和自旋中心漂移。`--verify-mount` 要求静止、直行和自旋三段均执行并通过。缺段时仍导出已执行阶段的诊断，报告写入 `verification_passed: false` 并返回 1。校验必须使用同一个基准高度和直行方向。自动分段沿用外参文件记录的 IMU 加速度倍率。

```bash
ros2 run xxu_bringup rigid_mount_calibrate \
  --verify-mount "$calibration_dir/rigid_mount.yaml" \
  --bag "$calibration_dir/validation" --auto-segment \
  --odom-topic "$odom_topic" \
  --base-height-above-ground "$base_height" \
  --output-report "$calibration_dir/verification_report.yaml"

ros2 launch xxu_bringup rigid_sensor.launch.py \
  mount_file:="$calibration_dir/rigid_mount.yaml" use_sim_time:=false \
  publish_mount_tf:=true start_base_adapter:=true source_child_frame:=imu
```

`rigid_sensor.launch.py` 检查时应给 `/odometry` 提供与虚拟传感器轴一致的前端里程计，查看 `/livox/imu_lio_compensated`、`/livox/lidar_compensated` 和 `/odom`。默认 `base_aligned` 的虚拟传感器轴与底盘对齐，`imu_aligned` 保留物理 IMU 轴。两者在 `base_link` 中给出同样的重力、点云与底盘轨迹。生产预处理的点云输入为 PointCloud2。完成上述检查后停止独立入口，把生成的 `backend_body.yaml` 中 `common.body` 合入对应底盘和地图的专用后端配置，再启动组合入口：

```bash
ros2 launch xxu_bringup sentry_localization.launch.py \
  mount_file:="$calibration_dir/rigid_mount.yaml" \
  frontend_config:=/path/to/vehicle_lio.yaml \
  backend_config:=/path/to/vehicle_backend.yaml \
  prior_map:=/path/to/matching_prior_map driver:=false
```

[2026-11-19 标定工具核验](../../../docs/validation/rigid_mount_calibration_20261119.md)记录合成数据闭环和真实历史录包的只读检查。实车动作采集和外参验收仍待现场完成。

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
