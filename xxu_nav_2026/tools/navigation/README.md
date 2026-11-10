# 小陀螺导航：职责与验证

新仿真入口为 `rm_sim_navigation.launch.py`，采用固定版本 rm_sim_26 哨兵及 RMUC 2025 场地，版本见 [simulation.repos](../../simulation.repos)。
默认使用 RGL Mid360；首次运行前执行 [build_rgl.sh](build_rgl.sh)，或显式设置 `lidar_mode:=gpu_lidar`。
缺少 RGL 库时启动会报错，需要完成构建或选择已有后端。

RMUC 地图由 [generate_rmuc_map.py](generate_rmuc_map.py) 根据固定场地碰撞几何生成。
用户 RMUL 场地入口为 `rmul_navigation.launch.py`，使用随包地图和 EGA 固定折叠车模，仅保留原导航位置的一台 MID360。
重新转换场地使用 [prepare_rmul_field.py](prepare_rmul_field.py)；车模、物理近似、雷达保护罩运行假设与验证见 [CAD 说明](cad/README.md)。
上游车模的 RGL 仍按 link 规则忽略云台；折叠车模仅忽略雷达自身壳体所在 link，其余车身参与遮挡，进入 LIO 前按模型过滤自身回波。

RMUL 场地外观使用 [recolor_rmul_field.py](recolor_rmul_field.py) 生成的 COLLADA 与纹理：
哑光 PVC 地面/平台、红蓝区域、白色边界及原有图案、中央黄色标记和深灰板件。
这是按现有几何分区的赛场风格参考配色，未使用官方印刷贴图；地面新增 5 cm 宽视觉边界线。
原 STL 继续用于碰撞，导航 PGM/YAML 保持原值，未标定材料的摩擦系数或激光回波响应。
重新转换场地会自动保留此材质流程；单独重生成外观可执行：

```bash
python3 tools/navigation/recolor_rmul_field.py src/xxu_bringup/models/rmul_2027
```

调色板在脚本 `PALETTE`；生成分组和原网格哈希见模型的 `materials/appearance.json`。

仿真、实车、独立导航和自主探索入口的 `use_fake_frame` 默认均为 `true`。
分开启动时若需要普通底盘坐标系，必须在两端均设置 `use_fake_frame:=false`。
仿真入口的 `cmd_vel_out_topic` 会同步传给碰撞监测器、速度变换和 watchdog。

`enable_lio:=false enable_cmd_vel_odom:=true` 仅用于诊断：扫描改用云台补偿点云，
不包含 LIO 底盘运动去畸变，地图导航仍不启动。watchdog 的扫描新鲜度检查保留。
测试与实验候选源码已于 2026-10-17 清理，当前保留链路见下文。

本项目的导航按二维平面全向底盘处理。底盘自旋、雷达云台旋转是两个独立运动：

| 模块 | 负责什么 |
|---|---|
| Small Point-LIO | 结合云台关节、点云/IMU 补偿，输出真实底盘 `/odom` |
| Nav2 MPPI | 在稳定参考系中规划 XY 平移和避障 |
| fake_vel_transform | 发布稳定参考系及 `/odom_nav`，实时转回底盘速度，叠加上位机小陀螺 |
| 底盘控制器/下位机 | 执行轮速，保留轮速限制和超时停车 |
| 云台驱动 | 执行独立的云台控制并发布带时间戳的 `gimbal_joint` 状态 |

## 当前主线与收敛范围

2026-11-09：后续集成以 **ZLIO → TDT → 底盘里程计适配 → Nav2 MPPI Omni → 速度保护 → 底盘** 为主线。`rmul_navigation.launch.py` 默认仍使用 Small Point-LIO / AMCL；现在可显式选择 `localization_backend:=zlio_tdt` 组合现有入口。新图近出生点四目标闭环结果见[阶段三当前记录](../../../docs/validation/nav2_stage3_20261108.md#新基线外部定位-nav2-四目标闭环)，不等于完整第三阶段或正式故障恢复验收。

2026-11-10：新图专用后端配置显式启用可信高频 `map → odom` 发布，近出生点四目标的 15 Hz 控制周期连续两轮无超时告警，目标和停车均通过；失信、长路线与实车范围仍待验收。参数默认关闭，具体边界及对照数字见[阶段三时序修正记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-10高频-tf-发布修正与四目标复测)。

| 范围 | 收敛处理 |
| --- | --- |
| 定位与导航代码 | 保留 ZLIO/TDT、底盘适配、逐点时间输入、Nav2、现有速度保护；不新增导航模块或替换算法 |
| 启动与配置 | 默认保留兼容入口；外部定位模式复用既有 launch 与节点构造函数，不复制阶段启动文件 |
| 候选与测试 | 停止扩展恢复矩阵、定位预测和轮地物理候选；保留随现存接口维护的离线检查，不重建一次性探针 |
| 实验资料 | `docs/validation/data/` 不存在，不恢复已删除的旧录包和构建；当前新基线结果与运行产物由阶段三当前记录路由 |

历史记录：[阶段一：接口与 Costmap](../../../docs/validation/nav2_stage1_20261108.md)、[阶段二：停车与恢复](../../../docs/validation/nav2_stage2_20261108.md)、[阶段三：正常导航](../../../docs/validation/nav2_stage3_20261108.md)。历史结果为六个正常导航目标成功，绕障受低薄板物理接触阻挡，通道未执行；实车可通过该区域由用户确认，仿真根因仍未确定。暂停物理候选后，后续正常导航可先选择不经过该薄板的路线，原失败结果保持未通过。

## 共享启动与参数来源

- [nav2_navigation.yaml](../../src/xxu_bringup/config/nav2_navigation.yaml) 与 [nav2_exploration.yaml](../../src/xxu_bringup/config/nav2_exploration.yaml)保存各自基础参数，non-fake 模式直接使用基础行为。
- [nav2_fake_profile.yaml](../../src/xxu_bringup/config/nav2_fake_profile.yaml)是 fake 模式覆盖值的唯一来源：`common` 为导航/探索共有的 15 项；`navigation` 为地图导航专用 PathFollow 三项；`exploration` 当前不增加覆盖，保留探索基础值。此文件是覆盖表，不能作为独立 Nav2 参数文件启动。调节 fake 模式时先确认目标值是否在此表中。
- 共享启动实现按职责划分： [navigation_parameters.py](../../src/xxu_bringup/xxu_bringup/navigation_parameters.py) 构造 Nav2 参数，[velocity_launch.py](../../src/xxu_bringup/xxu_bringup/velocity_launch.py) 构造速度链，[sensor_launch.py](../../src/xxu_bringup/xxu_bringup/sensor_launch.py) 分别构造传感器预处理与 Small Point-LIO，[robot_profiles.py](../../src/xxu_bringup/xxu_bringup/robot_profiles.py) 处理车型参数，[map_paths.py](../../src/xxu_bringup/xxu_bringup/map_paths.py) 解析建图保存位置。原 [launch_helpers.py](../../src/xxu_bringup/xxu_bringup/launch_helpers.py) 保留兼容导出；新调用按需导入对应模块。仿真时间、坐标系、命令话题和规划容差由启动参数决定；行为树文件名在覆盖表中保存，路径按当前安装位置展开。
- 实车、旧 Gazebo、rm_sim 共用速度转换器和 watchdog 的定义。调用处仅指定平台差异：输入/输出话题、关节话题、自旋值与滤波窗口。rm_sim 最终安全输出仍是 `/rm_sim/cmd_vel_safe`；CAD 车型交给 `xxu_chassis_controller` 发布逐轮指令，上游车型保留速度/云台适配器。

共享代码安装在既有 `xxu_bringup` 包内。[interfaces.py](../../src/xxu_bringup/xxu_bringup/interfaces.py) 单独提供接口数据读取，运行节点可直接使用，无需导入 Nav2 启动代码。`xxu_description` 只安装模型、显示和 ros2_control 资源；修改集成代码或新增 Python 模块后需重新构建并加载工作空间。

[navigation.launch.py](../../src/xxu_bringup/launch/navigation.launch.py) 默认按原方式组合 AMCL/GICP 与 Nav2。显式选择 `localization_mode:=external enable_localization:=true` 时，只管理地图服务和导航服务器，排除 AMCL/GICP；启动器复用现有 watchdog 的状态、源时间和恢复保持检查，并同时等待 TF。运行中的保护仍由原 watchdog 负责，启动门不等于任务恢复联锁。

EGA 仿真可通过原 `rmul_navigation.launch.py` 选择外部模式；须显式提供匹配的二维图、先验地图包、前后端配置和 Nav2 参数，缺失时拒绝启动。该模式固定关闭其他 LIO、自旋和 fake frame，使用已有逐点时间插件及底盘适配。加载重建的工作空间、并按[运行环境记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-09-续测预检查)处理本机动态库路径后，在 `xxu_nav_2026` 目录中使用：

```bash
ros2 launch xxu_bringup rmul_navigation.launch.py \
  localization_backend:=zlio_tdt gazebo_gui:=false rviz:=false \
  frontend_config:="$PWD/maps/rmul_self_20261109/lio.yaml" \
  backend_config:="$PWD/maps/rmul_self_20261109/backend.yaml" \
  prior_map:="$PWD/maps/rmul_self_20261109/prior" \
  external_map:="$PWD/maps/rmul_self_20261109/nav2.yaml" \
  external_nav2_params:="$PWD/maps/rmul_self_20261109/navigation.yaml" \
  rgl_plugin_dir:="$PWD/build/rgl_timed_runtime/install/RGLServerPlugin"
```

该新图配置仅适用于已知出生点 `(2,2,0)`；默认只启动系统，不自动下发目标。现有 `check_ega_navigation.py --external-localization` 在隔离域内监督本轮测试，定位失信时取消动作并持续向平滑前话题发送零速度，随后结束测试，不自动重发目标。这是测试客户端行为，未把暂停的新请求门或任务联锁候选接入正式链；本轮正常导航未注入故障验证此分支。

固定虚拟传感器的 ZLIO 适配入口为 [zlio_base_adapter.launch.py](../../src/xxu_bringup/launch/zlio_base_adapter.launch.py)，可显式启用 `publish_deskewed_cloud`；它要求前端 `imu` 与补偿后的 `lio_base_sensor` 表示同一个固定虚拟传感器坐标。底盘转换、点云转发及状态/时间校验由适配器负责。定位失信保护参数见 [watchdog 接口](../../src/fake_vel_transform/README.md)；这些部件不等于外部定位 Nav2 已正式接入。

[sensor_chain.yaml](../../src/xxu_bringup/config/sensor_chain.yaml) 保存 `gazebo`、`rm_sim`、`real` 的校准及补偿默认参数，话题/坐标系使用 [robot_interfaces.yaml](../../src/xxu_bringup/config/robot_interfaces.yaml) 的语义引用。CAD 车型按 `models/<robot_model>/geometry.json` 选择，模型目录须同时提供 `model.sdf` 和 `robot.urdf`；当前 CAD 接口约定为固定传感器、四个独立驱动轮和 RGL。

车型可在同目录提供 `navigation_profile.yaml`，按 `navigation` / `exploration` 分支配置避障缓冲、平移分量限速和更严格的到点阈值；只应用明确配置的分支。`rm_sim` 的 CAD 车型还可提供 `lio_profile.yaml`，叠加在基础 LIO YAML 之后、调用处显式参数之前。EGA 的参数、回归入口和限制见 [CAD 说明](cad/README.md#导航适配回归)。未提供车型配置时保留原有行为。

单独启动自主探索时，通过 `robot_geometry_file` 显式指定与仿真相同的车型几何；该入口默认值为空，不会从另一个 launch 进程继承车型选择。例如，在加载重建后的工作空间后使用 EGA：

```bash
ros2 launch xxu_slam_toolbox autonomous_mapping.launch.py \
  robot_geometry_file:=$(ros2 pkg prefix --share xxu_bringup)/models/ega_folded/geometry.json
```

该参数为探索的局部/全局 costmap 和停车检测区域应用车型几何；可选配置仅使用 `exploration` 分支。对应离线回归为 `python3 tools/navigation/test_vehicle_parameters.py`，覆盖真实探索启动描述的参数展开，不启动节点。

启动文件读取接口契约，Nav2、探索、LIO、RViz 和 CAD 车型 SDF/URDF 通过以下离线检查维护一致性（只需 Python 与 PyYAML）：

```bash
python3 tools/navigation/check_interfaces.py
```

检查会比较消费者的具体语义，例如 `/scan` 被误填为契约中已有的 `/odom` 也会报错；CAD 模型还检查四轮控制、RGL、IMU、关节话题和关键 URDF 坐标系。它不动态改写静态文件；调整接口默认值时必须同步相关静态消费者并重新检查。实车驱动边界的话题启动参数覆盖继续有效。

审查修复的回归检查保存在 [test_review_repairs.py](test_review_repairs.py)。加载 ROS Jazzy 与重建后的工作空间，执行 `python3 tools/navigation/test_review_repairs.py` 可检查命令时间戳、清理范围、模型契约和停车判定，不启动 ROS 节点。设置 `XXU_REVIEW_CHASSIS_EXECUTABLE` 为重建后的底盘可执行文件路径，还会在隔离域 193 和专用话题中验证异常输入停车及恢复；该选项启动测试节点，不接入机器人驱动。构建与实测范围见[审查修复核验](../../../docs/validation/review_repairs_20261027.md)。

建图的 `map_save_path` 仍可显式指定。默认先取绝对路径环境变量 `XXU_MAP_DIR`，否则通过已安装 `maps/auto_map.yaml` 的有效软链接定位可写源码目录；复制安装或无可写源码时回退到 `~/.ros/xxu_maps/auto_map` 并提示。地图路径不再依赖启动目录。

CAD 底盘共用原控制器：`wheel_radii` 可选逐轮半径，`wheel_command_topics` 可选四个 Float64 出口，`max_wheel_acceleration` 默认 0（不限制），`require_fresh_stamp` 默认 false。CAD 仿真显式设置 80 rad/s²、100 Hz 和双重超时检查；零指令及超时绕过斜坡直接停车。新参数取默认值时，实车和旧 Gazebo 仍使用原 Float64MultiArray 输出。

## 只有一个 fake 坐标系

```text
map → odom → base_footprint
                  ├─ base_link → gimbal_link → 雷达
                  └─ gimbal_yaw_fake
```

`gimbal_yaw_fake` 原点现在与 `base_footprint` 一致，方向固定在 odom 中。
TF 由 `fake_vel_transform` 发布：抵消真实底盘的姿态。它不接在真实云台下面。
`base_link` 与 `base_footprint` 在现有模型中仅有 Z 高度差，平面速度旋转相同。
此实现面向二维地面导航，不是跨坡度的完整三维运动模型。

- `/odom`：真实 `odom → base_footprint`，twist 表达在真实底盘坐标系，包含自旋。
- `/odom_nav`：保留最新位姿和时间戳，线速度先旋转到稳定坐标系，再作窗口平均；虚拟角速度为零。
- `/odom_nav` 不是另一个定位器，也不发布第二条 `odom → base_footprint` TF。
- AMCL/GICP、云台补偿和 watchdog 继续使用真实底盘链路。

仿真和实车入口默认使用 `nav_velocity_smoothing_window:=0.3`（秒），
设为 `0.0` 可关闭。该参数在节点启动时生效，已有实例需要重启。

速度链路：

```text
MPPI → /cmd_vel_nav → 平滑 → /cmd_vel_smoothed
 → 碰撞监测 → /cmd_vel_collision
 → fake_vel_transform → /cmd_vel_transformed
 → watchdog → /cmd_vel → 底盘轮速
```

开启 `use_fake_frame` 时，启动文件自动将 controller、BT navigator 和速度平滑器的
里程计输入切换到 `/odom_nav`。MPPI 的 `wz_max` 和 `az_max` 为零，保留非零角速度
采样方差，避免采样代价中的除零。禁用角度评分，采用 PositionGoalChecker；到点只
检查 XY，不要求真实底盘或云台转到目标姿态。恢复行为树移除 Spin，保留清图、等待
和后退。关闭 fake 模式时保留原 YAML 的普通导航设置。

常规 fake 模式的碰撞轮廓使用半径 0.42 m 的圆形旋转包络（包括模型中的突出车轮），
近场停车区改为各方向对称的八边形。实车外形改变时必须按实际旋转包络更新。
折叠 EGA 车型通过自身 `geometry.json` 在共享配置合并后覆盖包络和停车区，并显式保留原 `StoppedGoalChecker`、0.12 m XY 容差和 π 朝向容差（目标朝向不参与成功判定）。本次结构调整未改变这些值。

## 上位机控制小陀螺

仿真和实车入口均提供 `gyro_spin_rate`，默认 1.5 rad/s。正负值决定旋转方向，0 关闭。
该值由上位机速度变换节点叠加，下位机不应重复叠加一份。

```bash
# 仿真：同时启用地图导航与小陀螺；地图定位仍需先验证
ros2 launch xxu_bringup simulation.launch.py \
  start_navigation:=true enable_localization:=true \
  use_fake_frame:=true gyro_spin_rate:=1.5 rviz:=true

# 实车：需要真实雷达、云台 JointState、底盘接口已经就绪
ros2 launch xxu_bringup real_robot.launch.py \
  start_navigation:=true enable_localization:=true \
  use_fake_frame:=true gyro_spin_rate:=1.5
```

任务/决策节点可调用 `/fake_vel_transform/set_parameters` 自主切换，无需人工操控。
下列命令只是调试同一接口：

```bash
ros2 param set /fake_vel_transform spin_speed 0.0
ros2 param set /fake_vel_transform spin_speed 1.5
```

当前语义是**移动时自旋**：平移速度大于 0.01 m/s 时叠加；到点、零速度命令、
碰撞停车、指令或里程计超时均停止底盘自旋。雷达云台的持续旋转不受此开关控制。
若比赛策略需要到点后持续原地小陀螺，需要另外接入任务使能与安全状态，不能把
导航停车/碰撞停车的零速度直接覆盖掉。维护指令仍可发送 physical angular.z；正常 fake 导航输出的 angular.z 为零。

速度转换以 100 Hz 更新持有的平移指令，使用最新有效测量航向；航向外推与位姿预测 B 已于 2026-10-17 撤除。接口见[fake模块说明](../../src/fake_vel_transform/README.md)。
watchdog 也以 100 Hz 转发；底盘轮速发布仍为 50 Hz。旧指令不会因重发而无限续期，变换节点同时检查
原指令时间戳和接收时间，超过 0.3 s 输出零速度；里程计超时为 0.5 s。
输出频率不等于输入新鲜度，应同时检查来源时间戳与接收间隔。

默认 1.5 rad/s 来自已完成的组合运动验证。轮速不饱和边界不是导航速度上限，
仍需用定位质量、制动距离、传感器延迟和障碍物场景确定比赛速度。

## MPPI 路线基准与速度链分析

保留以下只读记录与离线分析工具：

| 文件 | 用途 |
|---|---|
| `analyze_mppi_benchmark.py` | 离线分析历史 CSV 与 NPY 数据 |
| `observe_mppi_manual.py` | 记录手动导航的速度链和参数 |
| `record_localization_jumps.py` | 记录定位跳变 |
| `monitor_localization_integrity.py` | 持续观察定位与输入完整性 |

地图导航的 fake 模式当前默认是 GoalCritic 权重 15/启用距离 1.2 m，PathFollow 权重 14/前瞻偏移 20/退出距离 0.6 m，PathAlign 阈值 0.5 m。
参数按下方“共享启动与参数来源”分工维护；实际运行值应核对模式合并后的结果。
生成数据可写入 `bags/`，该目录不纳入版本管理。

分析已有记录时，输出目录必须尚不存在：

```bash
python3 tools/navigation/analyze_mppi_benchmark.py \
  bags/mppi_benchmark/existing_run/run.json \
  --output docs/mppi_benchmark_existing_run
```

离线报告提供每段路径 RMS/P95、各级速度峰值、末端采样误差、终点 0.5 m 内停留时间、连续有效采样中的终点回弹、
平滑降速与碰撞停车时间占比、消息新鲜度和采样缺口。路径误差沿用旧数据的
“到最新路径采样点最近距离”，便于对比；末端误差是最后一个有效采样时刻的结果，
可能早于 action 完成时刻。新增记录在 action 完成后等待 1 墙钟秒保存 `settled` 位置、消息年龄及里程计速度，用来识别制动后位移；其中定位修正不能视为物理滑行。速度模跨级比较没有补偿底盘响应延迟，只能定位调查方向。
有效观测秒不包含无效采样间隔，不能直接等同整段 action 耗时。

默认排除非正向或超过 0.25 s 的时间间隔；速度/TF 消息年龄阈值为 0.3 s，
低频全局路径单独使用 2.0 s 阈值，可通过 `--max-gap`、`--max-age` 和
`--max-plan-age` 调整。新数据的完整路线必须全部目标成功且没有运行或取消错误。规划预检查会发布 `/plan`，必须与正在执行的记录器分开运行。
旧 JSON/NPY 数据也可分析，但缺少时间戳和目标总数，报告不会将其自动判定为完整路线。

比较配置时，固定地图、起点、路线、自旋速度及定位模式，保存参数快照，至少重复数圈。
未完成路线只比较对应公共路段；不要用其累计耗时宣称整圈更快。

`monitor_localization_integrity.py --output 全新目录` 可只读记录定位异常，
包括同一 odom 位姿在前后 map→odom 下的大修正、扫描接收中断和粒子扩散。
它也记录原始/补偿 IMU 时间缺口及 odom 异常位移；`--max-odom-speed`
默认 3.5 m/s，仅用于告警，不修改机器人速度上限。
它保存 `health.json` 与 `events.jsonl`，不会拦截 TF、发送目标或停车。
使用与现场一致的 ROS 域和中间件；仿真默认 `use_sim_time=true`。
