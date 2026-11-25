# 小陀螺导航：职责与验证

当前 RMUL 主线入口为 `rmul_navigation.launch.py`，只组合 ZLIO/TDT 定位。`rm_sim_gazebo.launch.py` 仅提供仿真和传感器，不内嵌 LIO。上游 rm_sim_26 的固定版本见 [simulation.repos](../../simulation.repos)。旧 Small Point-LIO 及关联启动链已于 2026-11-11 按用户决定归档撤除。

RMUC 地图由 [generate_rmuc_map.py](generate_rmuc_map.py) 根据固定场地碰撞几何生成。
用户 RMUL 场地入口为 `rmul_navigation.launch.py`，显式选择匹配的二维地图、先验包及定位配置，使用 EGA 固定折叠车模，仅保留原导航位置的一台 MID360。
重新转换场地使用 [prepare_rmul_field.py](prepare_rmul_field.py)；车模、物理近似、雷达保护罩运行假设与验证见 [CAD 说明](cad/README.md)。
上游车模的 RGL 仍按 link 规则忽略云台；折叠车模仅忽略雷达自身壳体所在 link，其余车身参与遮挡，进入 LIO 前按模型过滤自身回波。

RMUL 场地外观使用 [recolor_rmul_field.py](recolor_rmul_field.py) 生成的 COLLADA 与纹理：
哑光 PVC 地面/平台、红蓝区域、白色边界及原有图案、中央黄色标记和深灰板件。
这是按现有几何分区的赛场风格参考配色，未使用官方印刷贴图；地面新增 5 cm 宽视觉边界线。
原 STL 保留为几何来源；地面标记改为仅显示的贴地面，单独导出的 `rmul_2027_collision.stl` 将这些装饰面的碰撞高度归零，保留非地面结构坐标。导航 PGM/YAML 保持原值，未标定材料的摩擦系数或激光回波响应。
重新转换场地会自动保留此流程；单独重生成贴地外观及对应碰撞网格可执行：

```bash
python3 tools/navigation/recolor_rmul_field.py src/xxu_sim_bringup/models/rmul_2027
```

调色板在脚本 `PALETTE`；生成分组和原网格哈希见模型的 `materials/appearance.json`。

RMUL ZLIO 主线与独立导航统一使用真实底盘坐标（`base_link`），`fake_vel_transform` 与虚拟参考帧分支已撤除。
仿真入口的 `cmd_vel_out_topic`（默认 `/cmd_vel_collision`）由 `collision_monitor` 输出后直接送入 `cmd_vel_watchdog`（`xxu_bringup`），经门控后输出 `/rm_sim/cmd_vel_safe`。

`enable_lio:=false enable_cmd_vel_odom:=true` 仅用于诊断：扫描改用云台补偿点云，
不包含 LIO 底盘运动去畸变，地图导航仍不启动。watchdog 的扫描新鲜度检查保留。
测试与实验候选源码已于 2026-10-17 清理，当前保留链路见下文。

本项目的导航按二维平面全向底盘处理。底盘自旋、雷达云台旋转是两个独立运动：

| 模块 | 负责什么 |
|---|---|
| ZLIO 与底盘适配 | ZLIO 接收补偿后的点云/IMU，底盘适配输出 `/odom` 和动态 TF |
| Nav2 MPPI Omni | 同时预测 XY 平移与车体转动；显式 `--spin` 模式增加持续自旋评分 |
| 平滑器、碰撞监测与 watchdog | 平滑速度并执行碰撞、消息新鲜度与定位失信停车保护 |
| 底盘控制器/下位机 | 执行轮速，保留轮速限制和超时停车 |
| 云台驱动 | 执行独立的云台控制并发布带时间戳的 `gimbal_joint` 状态 |

## 当前主线与收敛范围

2026-11-14：按用户要求临时试验每轴 0.6 m/s、自旋 0.5 rad/s，两个目标及停车通过，返回段在规划失败后的恢复旋转中定位失信，整组未通过，已回退平移 0.5。回退后另一手动目标也失信，不能将故障直接归因于提速；默认配置未改。证据与当前限制见[平移提速试验](../../../docs/validation/nav2_stage3_20261108.md#2026-11-14-平移-06-ms-会话试验与回退)。

2026-11-13：持续自旋已作为显式 `--spin` 试验模式接入 MPPI；实测移动段平均角速度 0.491 rad/s，随后目标完成和 10 秒停车观察通过。自动往返被新目标抢占，完整回归及动态避障验收仍待完成，默认配置不变。范围与证据见[持续自旋接入](../../../docs/validation/nav2_stage3_20261108.md#2026-11-13-mppi-持续自旋显式接入)。

2026-11-13：十样本平均模式的 IMU 输入缓存已从 5 条扩至 4000 条，40 项相关检查及安装后两轮阻塞对照通过，普通直通模式不变；旧 LOST 会话随后退出，新出生点会话已恢复 NORMAL。范围与证据见[IMU 缓存修复](../../../docs/validation/nav2_stage3_20261108.md#2026-11-13-十样本-imu-输入缓存修复)。

2026-11-13：当前启动统一使用 [start_rmul.sh](start_rmul.sh) 显式选择 `tested_sim` 或 `baseline`，已脱离日志覆盖层；39 项离线检查、68 项核心回归和安装后的限定四目标检查通过。录帧专项的四项失败及长路线边界保留，验证范围与证据见[启动收敛记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-13-启动与实验实现收敛)。

2026-11-12：当前可视化加载[多位置旋转实测新图](../../maps/rmul_rotating_20261112/README.md)及按用户要求改成贴地标记的场地。原先约 `(5.45,2.53)` 的停滞路线在取消装饰凸起碰撞后，直接单点及朝向 0° 的往返检查通过；180° 后快速斜移仍触发定位保护，完整组合未通过。场地修改、安装核验和当前启动入口见[地面图案贴地记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-12-地面图案改为贴地显示)。旧图及原始场地 STL 保留，地图采集与修改场地前的失败见[重建记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-12-多位置旋转重建地图)；导航默认参数未改。

2026-11-12：定位联锁已修正同一坐标 epoch 内可信状态代次切换导致的误取消，保护检查及 180° 回归通过；动态避障仍未整体验收，横穿净距和持续挡路后的停滞待解决。修复、同流回放及最新闭环结果见[阶段三联锁修正记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-12-定位联锁代次语义修正)。此前短时挡路后移走两轮通过的范围仍见[动态障碍初测](../../../docs/validation/nav2_stage3_20261108.md#2026-11-12-动态障碍初测)。

2026-11-12：MPPI 平移每轴 0.5 m/s、转向维持 0.2 rad/s 的显式配置已在同一六目标仿真路线连续两轮通过，包含停车与每目标 10 秒静止检查。配置、复跑入口、基线对比及 0.6 rad/s 转向失败边界统一见[阶段三提速记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-12-mppi-平移提速对照)。原默认配置未改，绕障及实车未验收。

2026-11-11：后续集成以 **ZLIO → TDT → 底盘里程计适配 → Nav2 MPPI Omni → 速度保护 → 底盘** 为主线。`rmul_navigation.launch.py` 已收敛为唯一 ZLIO/TDT 组合，旧 Small Point-LIO / AMCL 分支已撤除；`localization_backend` 仅接受 `zlio_tdt`。新图近出生点四目标闭环结果见[阶段三当前记录](../../../docs/validation/nav2_stage3_20261108.md#新基线外部定位-nav2-四目标闭环)，不等于完整第三阶段或正式故障恢复验收。

2026-11-10：新图专用后端配置显式启用可信高频 `map → odom` 发布，近出生点四目标的 15 Hz 控制周期连续两轮无超时告警，目标和停车均通过；失信、长路线与实车范围仍待验收。参数默认关闭，具体边界及对照数字见[阶段三时序修正记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-10高频-tf-发布修正与四目标复测)。

| 范围 | 收敛处理 |
| --- | --- |
| 定位与导航代码 | 保留 ZLIO/TDT、底盘适配、逐点时间输入、Nav2、现有速度保护；不新增导航模块或替换算法 |
| 启动与配置 | RMUL 统一为 ZLIO/TDT；撤除旧 LIO 仿真/实车启动，仅保留传感器底座和通用 Nav2 构造 |
| 候选与测试 | 停止扩展恢复矩阵、定位预测和轮地物理候选；保留随现存接口维护的离线检查，不重建一次性探针 |
| 实验资料 | `docs/validation/data/` 不存在，不恢复已删除的旧录包和构建；当前新基线结果与运行产物由阶段三当前记录路由 |

历史记录：[阶段一：接口与 Costmap](../../../docs/validation/nav2_stage1_20261108.md)、[阶段二：停车与恢复](../../../docs/validation/nav2_stage2_20261108.md)、[阶段三：正常导航](../../../docs/validation/nav2_stage3_20261108.md)。历史结果为六个正常导航目标成功，绕障受低薄板物理接触阻挡，通道未执行；实车可通过该区域由用户确认，仿真根因仍未确定。暂停物理候选后，后续正常导航可先选择不经过该薄板的路线，原失败结果保持未通过。

## 共享启动与参数来源

- [nav2_navigation.yaml](../../src/xxu_bringup/config/nav2_navigation.yaml) 与 [nav2_exploration.yaml](../../src/xxu_bringup/config/nav2_exploration.yaml)保存各自基础参数；通用车型外包络与模式参数通过 `robot_geometry_file`（及同目录可选 `navigation_profile.yaml`）由 `RobotGeometryParameters` 叠加（`nav2_fake_profile.yaml` 已撤除）。
- 共享启动实现按职责划分： [navigation_parameters.py](../../src/xxu_bringup/xxu_bringup/navigation_parameters.py) 构造 Nav2 参数，[velocity_launch.py](../../src/xxu_bringup/xxu_bringup/velocity_launch.py) 构造速度保护门控（`cmd_vel_watchdog`），[sensor_launch.py](../../src/xxu_bringup/xxu_bringup/sensor_launch.py) 只构造传感器预处理，估计器统一由 ZLIO bringup 启动，[robot_profiles.py](../../src/xxu_bringup/xxu_bringup/robot_profiles.py) 处理车型参数，[map_paths.py](../../src/xxu_bringup/xxu_bringup/map_paths.py) 解析建图保存位置。原 [launch_helpers.py](../../src/xxu_bringup/xxu_bringup/launch_helpers.py) 保留兼容导出；新调用按需导入对应模块。
- 实车与 `rm_sim` 共用 `xxu_bringup` 中 `velocity_chain` / `cmd_vel_watchdog` 的接口定义；调用处按需指定输入/输出话题、关节话题及定位健康检查参数。`rm_sim` 最终安全输出为 `/rm_sim/cmd_vel_safe`；CAD 车型交给 `xxu_chassis_controller` 发布逐轮指令，实车通过 `xxu_hardware` 串口桥接下发。

共享代码安装在 `xxu_bringup` 包内；实车串口通信桥位于 `xxu_hardware`，Gazebo 仿真启动与 CAD/场地模型位于 `xxu_sim_bringup`。[interfaces.py](../../src/xxu_bringup/xxu_bringup/interfaces.py) 单独提供接口数据读取，运行节点可直接使用，无需导入 Nav2 启动代码。`xxu_description` 只安装模型、显示和 ros2_control 资源；修改集成代码或新增 Python 模块后需重新构建并加载工作空间。

[navigation.launch.py](../../src/xxu_bringup/launch/navigation.launch.py) 已收敛为仅使用外部定位（ZLIO/TDT）：`localization_mode` 仅保留兼容参数 `external`（旧 AMCL/GICP 已移除），启动 `map_server`、Nav2 导航服务器与 `nav2_navigation_startup` 等待定位就绪和 TF；运行中的速度安全门控由 `cmd_vel_watchdog` 负责，启动等待不等于任务恢复联锁。

EGA 仿真使用 [start_rmul.sh](start_rmul.sh) 统一准备安装环境和匹配输入。在 `xxu_nav_2026` 目录执行：

```bash
./tools/navigation/start_rmul.sh --profile tested_sim
```

动态避让人工测试可增加一辆外部速度话题控制的 EGA 车：

```bash
./tools/navigation/start_rmul.sh --profile tested_sim --dynamic-vehicle
```

第二辆车与原车使用同一物理车模，以场地中心 `(6.3, 4.3)` 对称出生并反向朝向；默认位置为 `(10.6, 6.6)`。该车订阅 `/dynamic_test/cmd_vel`（`geometry_msgs/msg/TwistStamped`），不发布传感器数据，独立的命令和轮速话题不会控制原车（`key_controll` 键盘遥控包现已撤除）。2026-11-14 原限定核验记录：隔离域无界面启动日志确认第二车在 `(10.6, 6.6)` 创建成功、四个独立轮速桥及底盘控制器启动；传感器观测及动态避让效果尚未实测，不能据此视为避让通过。

空闲原地补扫使用 `./tools/navigation/start_rmul.sh --profile tested_sim --dynamic-vehicle --idle-scan`。无导航任务或到点后空闲 2 秒，复用 Nav2 Spin 循环扫描一圈；按用户 2026-11-14 续调使用 0.5 rad/s 上限（平滑器同步为 ±0.5），收到新导航/行为任务即取消自己的补扫，保留碰撞检查、速度平滑和最终安全门。定位失信时取消，恢复可信并通过原恢复保持检查后再尝试。此选项与移动中的 `--spin` 独立，普通启动默认关闭。补扫需要新的有效射线才能清除旧障碍，不能保证清空全部遮挡区；长时间旋转中的定位仍待验证。2026-11-14 的复现、当前会话核验和限制见[空闲补扫记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-14-障碍残留机制复现与空闲补扫)。

显式试验边移动边自旋：`./tools/navigation/start_rmul.sh --profile tested_sim --spin`。该选项要求 MPPI Omni，目标角速度 +0.5 rad/s，在距目标 1.0→0.65 m 的范围逐渐释放自旋要求，之后由原目标朝向与停车检查器完成到点；无导航目标时不强制自旋。自旋直接参与 MPPI 预测和评分，关闭与其冲突的 PathAngle/Twirling 评分，保留碰撞、限速及定位保护和彩色轨迹显示。此模式只允许 `tested_sim`，需要重新构建安装 `xxu_bringup` 的 C++ 评分插件；尚未完成完整路线回归，不用于实车验收。

`tested_sim` 显式选择旋转实测新图、`imu_sampling:=interval_mean`、`backend_experimental.yaml` 与停车型目标检查器；`baseline` 使用同图的 100 Hz 直接 IMU 和默认后端。其中 `imu_sampling:=interval_mean`（1000→100 Hz 十点平均）仅用于补偿 `ega_folded` 仿真 `1 ms` 步长下的瞬时漏采，**实车保持默认直通且禁止开启 `imu_interval_samples:=10`**，原理与待测项见[实车与仿真 IMU 采样边界](../lio/README.md#实车与仿真-imu-采样边界)。后端候选参数 `registration/experimental_blended_adaptive` 默认关闭，尚有录帧专项诊断和 180° 后快速斜移失败。追加 `--headless` 关闭界面；`-- launch_arg:=value` 显式覆盖启动参数。脚本保留调用者的 ROS 域和 Gazebo 分区设置，未指定时使用域 90 和新分区。运行依赖正常安装目录、实测地图、依赖库与已构建 RGL 插件，不再加载日志中的程序或模型副本；历史覆盖层只保留为证据。

该新图配置仅适用于已知出生点 `(2,2,0)`；默认只启动系统，不自动下发目标。现有 `check_ega_navigation.py --external-localization` 在隔离域内监督本轮测试，定位失信时取消动作并持续向平滑前话题发送零速度，随后结束测试，不自动重发目标。这是测试客户端行为，未把暂停的新请求门或任务联锁候选接入正式链；本轮正常导航未注入故障验证此分支。

固定虚拟传感器的 C++ 适配入口为 [zlio_base_adapter.launch.py](../../src/xxu_bringup/launch/zlio_base_adapter.launch.py)，前端现在直接发布 `/cloud_deskewed`。它要求前端 `imu` 与补偿后的 `lio_base_sensor` 表示同一个固定虚拟传感器坐标；桥接输出 `base_link` 里程计与 TF。定位可信度、源时间、新鲜度和恢复保护由 [C++ 看门狗](../../src/xxu_bringup/src/cmd_vel_watchdog.cpp)统一执行，Python 启动/空闲补扫监督只接收低频就绪信号。专用 TF 数据流、实车入口和本次验证范围见 [C++ 集成核验](../../../docs/validation/zlocalization_cpp_integration_20261116.md)；代码接入不代表实车定位或性能验收通过。

[sensor_chain.yaml](../../src/xxu_bringup/config/sensor_chain.yaml) 保存 `gazebo`、`rm_sim`、`real` 的校准及补偿默认参数，话题/坐标系使用 [robot_interfaces.yaml](../../src/xxu_bringup/config/robot_interfaces.yaml) 的语义引用。仿真 CAD 车型按 `xxu_sim_bringup/models/<robot_model>/geometry.json` 选择，模型目录须同时提供 `model.sdf` 和 `robot.urdf`；当前 CAD 接口约定为固定传感器、四个独立驱动轮和 RGL。

车型可在同目录提供 `navigation_profile.yaml`，按 `navigation` / `exploration` 分支配置避障缓冲、平移分量限速和更严格的到点阈值；只应用明确配置的分支。可选的 `goal_yaw_tolerance` 只有显式设置才让停车型目标检查器验收航向；EGA 默认 profile 未设置，隔离转向试验及未通过的定位持续性结果见[阶段三记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-10保留停车检查器的航向候选及定位失信复现)。`rm_sim` 的 CAD 车型还可提供 `lio_profile.yaml`，叠加在基础 LIO YAML 之后、调用处显式参数之前。EGA 的参数、回归入口和限制见 [CAD 说明](cad/README.md#导航适配回归)。未提供车型配置时保留原有行为。

180° 转后定位稳定性回归使用 [check_yaw_stability.py](check_yaw_stability.py)，默认两种路线各三轮，每目标停车后继续观察 10 s 仿真时间，任一轮失信或缺少新鲜证据即失败。它同时监测静止全窗口的原始与适配里程计速度，保留窗口前后停车检查。可选 `--frontend-noise-profile tools/navigation/ega_imu_sample_noise.yaml` 仅在结果目录生成原 100 Hz 流的仿真噪声对照配置，单独校正噪声仍为 4/6。区间平均隔离候选在同输出噪声强度下完整 6/6 通过，候选模型与补偿副本尚未加入默认链；详见报告的最新源速度回归节。它只通过显式测试 profile 验收航向，未改默认车型；固定判据、执行命令及实际结果以[阶段三回归记录](../../../docs/validation/nav2_stage3_20261108.md#2026-11-11180-转后定位稳定性回归)为准。

单独启动自主探索时，通过 `robot_geometry_file` 显式指定与仿真相同的车型几何；该入口默认值为空，不会从另一个 launch 进程继承车型选择。例如，在加载重建后的工作空间后使用 EGA：

```bash
ros2 launch xxu_slam_toolbox autonomous_mapping.launch.py \
  robot_geometry_file:=$(ros2 pkg prefix --share xxu_sim_bringup)/models/ega_folded/geometry.json
```

该参数为探索的局部/全局 costmap 和停车检测区域应用车型几何；可选配置仅使用 `exploration` 分支。对应离线回归为 `python3 tools/navigation/test_vehicle_parameters.py`，覆盖真实探索启动描述的参数展开，不启动节点。

启动文件读取接口契约，Nav2、探索、LIO、RViz 和 CAD 车型 SDF/URDF 通过以下离线检查维护一致性（只需 Python 与 PyYAML）：

```bash
python3 tools/navigation/check_interfaces.py
```

检查会比较消费者的具体语义，例如 `/scan` 被误填为契约中已有的 `/odom` 也会报错；CAD 模型还检查四轮控制、RGL、IMU、关节话题和关键 URDF 坐标系。它不动态改写静态文件；调整接口默认值时必须同步相关静态消费者并重新检查。实车驱动边界的话题启动参数覆盖继续有效。

审查修复的回归检查保存在 [test_review_repairs.py](test_review_repairs.py)。加载 ROS Jazzy 与重建后的工作空间，执行 `python3 tools/navigation/test_review_repairs.py` 可检查命令时间戳、清理范围、模型契约和停车判定，不启动 ROS 节点。设置 `XXU_REVIEW_CHASSIS_EXECUTABLE` 为重建后的底盘可执行文件路径，还会在隔离域 193 和专用话题中验证异常输入停车及恢复；该选项启动测试节点，不接入机器人驱动。构建与实测范围见[审查修复核验](../../../docs/validation/review_repairs_20261027.md)。

建图的 `map_save_path` 仍可显式指定。默认先取绝对路径环境变量 `XXU_MAP_DIR`，否则通过已安装 `maps/auto_map.yaml` 的有效软链接定位可写源码目录；复制安装或无可写源码时回退到 `~/.ros/xxu_maps/auto_map` 并提示。地图路径不再依赖启动目录。`mapping.launch.py` 与 `autonomous_mapping.launch.py` 会将启动参数 `use_sim_time` 重写进 `slam_params_file`，实车传入 `use_sim_time:=false` 时不会被静态 YAML 中的默认值覆盖（由 `test_slam_toolbox_launch.py` 覆盖启动展开回归）。

若使用 ZLIO/TDT 建图模式（`mode:=mapping` + `/backend_node/save_map`）生成 3D `map.pcd`，可用离线工具按通行高度带在同一 `map` 坐标系下导出 Nav2 二维图（`.pgm` + `.yaml`，可选 `--keyframes` 沿关键帧局部清零自由区）：

```bash
python3 tools/navigation/pcd_to_nav2_map.py \
  --pcd /path/to/map.pcd --output /path/to/nav2.yaml
```

CAD 底盘共用原控制器：`wheel_radii` 可选逐轮半径，`wheel_command_topics` 可选四个 Float64 出口，`max_wheel_acceleration` 默认 0（不限制），`require_fresh_stamp` 默认 false。CAD 仿真显式设置 80 rad/s²、100 Hz 和双重超时检查；零指令及超时绕过斜坡直接停车。新参数取默认值时，实车和旧 Gazebo 仍使用原 Float64MultiArray 输出。

## 坐标链与速度保护

当前主线统一使用真实底盘坐标，不再维护 `gimbal_yaw_fake` 虚拟坐标系、`/odom_nav` 或 `fake_vel_transform` 节点：

```text
map --动态--> odom --动态 30 Hz--> base_link --静态--> 传感器（直接子帧）
```

- `/odom`：真实 `odom -> base_link`，twist 表达在真实底盘坐标系。导航不启动机械模型 TF，扫描只用一次静态传感器外参；完整盘点和当前验证见 [TF 瘦身核验](../../../docs/validation/tf_tree_audit_20261116.md)。
- 速度链路：`MPPI -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel_collision -> cmd_vel_watchdog (xxu_bringup) -> /cmd_vel（默认）或 /rm_sim/cmd_vel_safe（rm_sim 仿真）`。
- `cmd_vel_watchdog`（定义见 [velocity_launch.py](../../src/xxu_bringup/xxu_bringup/velocity_launch.py) 与 [cmd_vel_watchdog.cpp](../../src/xxu_bringup/src/cmd_vel_watchdog.cpp)）以 `100 Hz` 输出、基于单调稳态时钟检查指令超时（默认 `0.3 s`）、`/odom` / `/scan` / `joint_states` / 必需 TF 对超时（默认 `0.5 s`）以及可选外部定位健康状态（含 `localization_recovery_hold`），任一条件不满足即发布零速。TF 发布者使用专用输入流，保护节点不订阅全局 `/tf`。
- 车型碰撞与到点配置：常规配置叠加 `robot_geometry_file` 的圆形外包络与八边形近场急停多边形；折叠 EGA 车型通过自身 `geometry.json` 保留 `StoppedGoalChecker`。
- 小陀螺自旋策略已不再通过 `/fake_vel_transform/set_parameters` 动态叠加：普通导航默认不附加自旋，显式 `--spin` 试验模式由 MPPI 自身预测并输出角速度（见上文 `--spin` 说明）。

## MPPI 路线基准与速度链分析

保留以下只读记录与离线分析工具：

| 文件 | 用途 |
|---|---|
| `analyze_mppi_benchmark.py` | 离线分析历史 CSV 与 NPY 数据 |
| `observe_mppi_manual.py` | 记录手动导航的速度链和参数 |
| `record_localization_jumps.py` | 记录定位跳变 |
| `monitor_localization_integrity.py` | 持续观察定位与输入完整性 |

地图导航基础参数直接维护在 [nav2_navigation.yaml](../../src/xxu_bringup/config/nav2_navigation.yaml)，车型外包络与模式配置按 `robot_geometry_file` 叠加（见上文“共享启动与参数来源”）。
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

## 工具与回归脚本目录

| 类别 | 文件 | 用途 |
|---|---|---|
| 启动与场地构建 | [start_rmul.sh](start_rmul.sh) | RMUL 仿真导航统一启动入口（支持 `tested_sim` / `baseline`、`--idle-scan`、`--spin`、`--dynamic-vehicle`） |
| 启动与场地构建 | [generate_rmuc_map.py](generate_rmuc_map.py) | 根据固定场地碰撞几何生成 RMUC 二维地图 |
| 地图导出 | [pcd_to_nav2_map.py](pcd_to_nav2_map.py) | 从 ZLIO/TDT 3D `map.pcd` 按高度带离线导出同坐标系 Nav2 `.pgm` / `.yaml` |
| 启动与场地构建 | [prepare_rmul_field.py](prepare_rmul_field.py) | 重新转换 RMUL 场地模型与碰撞网格 |
| 启动与场地构建 | [recolor_rmul_field.py](recolor_rmul_field.py) | 生成 RMUL 场地贴地外观纹理与 `rmul_2027_collision.stl` |
| 构建与补丁 | [build_rgl.sh](build_rgl.sh) | 构建默认 RGL Gazebo 插件 |
| 构建与补丁 | [build_rgl_timed_candidate.sh](build_rgl_timed_candidate.sh) | 使用 [rgl_timed_scan.patch](rgl_timed_scan.patch) 构建逐点时间候选 RGL 插件 |
| 离线与隔离测试 | [check_interfaces.py](check_interfaces.py) | 静态检查启动、配置与 CAD 模型话题/坐标系接口契约 |
| 离线与隔离测试 | [test_cpp_bridge_pipeline.py](test_cpp_bridge_pipeline.py) | C++ 传感器预处理、里程计桥、看门狗 TF/保活与时钟限频隔离管线测试 |
| 离线与隔离测试 | [test_chassis_lower_bridge.py](test_chassis_lower_bridge.py) | STM32 串口协议编解码、状态机与 C++ PTY 环回测试 |
| 离线与隔离测试 | [test_rigid_mount.py](test_rigid_mount.py) | 实车刚性安装多底盘外参、预处理与帧传递回归 |
| 离线与隔离测试 | [test_rigid_mount_calibration.py](test_rigid_mount_calibration.py) | 刚性外参在线/离线标定工具与闭环校验回归 |
| 离线与隔离测试 | [test_vehicle_parameters.py](test_vehicle_parameters.py) | 车型几何与探索/导航参数叠加回归 |
| 离线与隔离测试 | [test_slam_toolbox_launch.py](test_slam_toolbox_launch.py) | `xxu_slam_toolbox` 启动参数展开与 `use_sim_time` 重写回归 |
| 离线与隔离测试 | [test_pcd_to_nav2_map.py](test_pcd_to_nav2_map.py) | `pcd_to_nav2_map.py` 3D→2D 地图导出单元与实测冒烟测试 |
| 离线与隔离测试 | [test_zlio_navigation.py](test_zlio_navigation.py) | ZLIO 转换契约、C++ 管线与 `BridgeReadiness` / `NavigationStartup` 测试 |
| 离线与隔离测试 | [test_idle_scan.py](test_idle_scan.py) | `IdleScan` 空闲自旋抢占与安全取消单元测试 |
| 离线与隔离测试 | [test_review_repairs.py](test_review_repairs.py) | 时间戳、进程归属、模块导入边界与停车判定回归 |
| 离线与隔离测试 | [test_yaw_stability.py](test_yaw_stability.py) | 航向稳定性辅助脚本单元测试 |
| 仿真检查与观测 | [check_yaw_stability.py](check_yaw_stability.py) | 180° 转后定位稳定性仿真回归（可选 [ega_imu_sample_noise.yaml](ega_imu_sample_noise.yaml)、[yaw_stability_profile.yaml](yaw_stability_profile.yaml)） |
| 仿真检查与观测 | [check_ega_navigation.py](check_ega_navigation.py) | EGA 仿真导航路线闭环观察器 |
| 只读观测与分析 | [analyze_mppi_benchmark.py](analyze_mppi_benchmark.py) | 离线分析 MPPI 路线基准 CSV/NPY 与 JSON |
| 只读观测与分析 | [observe_mppi_manual.py](observe_mppi_manual.py) | 只读记录手动导航速度链与参数 |
| 只读观测与分析 | [record_localization_jumps.py](record_localization_jumps.py) | 只读记录定位跳变与时间缺口 |
| 只读观测与分析 | [monitor_localization_integrity.py](monitor_localization_integrity.py) | 只读持续观察定位状态与传感器输入完整性 |
| 只读观测与分析 | [navigation_evidence.py](navigation_evidence.py) | 停车证据与轨迹飘移判定辅助模块 |
