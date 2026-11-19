# 累计平面地图 Gazebo 对照（2026-10-28）

## 结论与范围

两种地图都未通过现有完整导航回归。每种地图各运行两次：首次两组均到达 5 个目标，但 10 次停车证据均未通过；复测时 SmallIVox 在第二个目标超时，累计平面地图在首个目标途中触发急停并停滞。**累计地图目前不能据此切换为正式默认地图。**

相同的 8 秒静止窗口中，累计平面组的水平里程计速度噪声和位置变化较小。首次完整路线中，它的真值对齐里程计误差也较小；但两次复测没有完成相同路线，只有每组两次运行，不能将此结论外推为可靠的导航改善。

本次为 NVIDIA GPU 上的无界面 Gazebo/RMUL/EGA 仿真，非实机或真实传感器录包。RGL 当前模式按帧时间输出，不合成逐点采样时间；EGA 传感器固定在车体，测试覆盖车体旋转和平移，不覆盖云台相对车体转动，也未量化地图重影或扫描内去畸变。

## 运行条件

- 场地和车型：RMUL 2027、EGA 固定折叠车型，起点 `(2, 2)`；目标顺序 `(6.59,4.44) → (2,2) → (3,2) → (2,3) → (2,2)`。
- 地图：当前 SmallIVox `map_resolution=0.2 m`，候选上游累计平面地图 `moment_map_resolution=0.5 m`。两组使用相同的临时构建、场地、参数与目标，仅切换 `map_backend`。
- 实际参数：`laser_point_cov=0.001`、`extrinsic_est_en=false`、`planar_constraint_en=true`、`defer_map_insertion_by_scan=true`、`dynamic_init_en=false`，小陀螺配置 `1.5 rad/s`。两组的静止和运动初始化配置一致；这轮没有测试运动初始化。
- ROS 2 Jazzy；`ROS_DOMAIN_ID=73`、本机发现，每次使用独立 Gazebo 分区。临时合并安装中重建 7 个直接相关包并启用 `SMALL_POINT_LIO_ZLIO_MAP=ON`；部分仿真/雷达桥依赖沿用工作空间既有安装。没有替换日常安装。
- 检查器在导航 ready 后额外等待 8 个仿真秒，再发送目标。每目标最多 105 秒墙钟时间；动作成功、停点与独立停车观测均须通过，才算回归通过。
- 四次均通过 Nav2/传感器启动检查；参数服务确认每次请求的地图后端，后三次观察器还核验了运行中加载的临时 LIO 动态库。第一次 SmallIVox 的观察器退出时异常，已刷新的样本仍保留，检查器参数与 LIO 可执行文件路径可核对；没有该次运行中的动态库指纹。
- 每组两次顺序运行，仿真输入并非同一录包，控制、AMCL 及物理过程存在运行间波动；目标超时或停滞后检查器停止后续目标。所有运行的资源统计均仅覆盖 LIO 进程，不包含 Gazebo、GPU、RGL 或 Nav2。

资源采样约每 0.2 秒墙钟一次，CPU 为 `/proc` 累计用户加系统时间除以进程被采样的墙钟区间（单核 100%），RSS 为采样到的进程驻留内存最大值，包含共享库；不是地图单独内存或瞬时峰值保证。

## 静止与运动定位

从检查器发送首目标前的最后约 8 仿真秒提取样本。四次的 Gazebo 真值 XY 最大位移和 yaw 变化均为 0；里程计与真值按消息源时间对齐，真值在相邻采样间插值，超过 0.5 秒的真值间隔不外推。采样率约 5 Hz，峰值可能被漏采。

| 运行 | 静止里程计 XY 最大变化 cm | 静止水平速度 RMS m/s | 运动段 XY 误差 RMSE cm | 运动段 XY 误差 P95 cm | LIO 平均 CPU % | 采样 RSS 峰值 MiB |
| --- | --- | --- | --- | --- | --- | --- |
| SmallIVox 1 | 3.42 | 0.252 | 4.18 | 7.15 | 7.3 | 48.7 |
| SmallIVox 2 | 2.01 | 0.126 | 5.20 | 9.18 | 7.1 | 48.8 |
| 累计平面 1 | 0.94 | 0.047 | 1.50 | 2.53 | 7.4 | 49.7 |
| 累计平面 2 | 0.84 | 0.044 | 1.59 | 2.11 | 7.0 | 49.7 |

对齐只使用静止窗口最后 2 秒估计一次 yaw 和 XY 平移，不拟合整条轨迹；odom 的 `base_footprint` 与 Gazebo 模型真值为不同坐标系，车模中的 `base_footprint → base_link` XY 偏移为零。脚本为[真值分析器](../../xxu_nav_2026/src/small_point_lio/test/map_benchmark/analyze_gazebo_run.py)。

四次真值平移速度峰值分别约 0.62、0.60、0.65、0.53 m/s，真值角速度峰值约 1.61、1.61、1.61、1.56 rad/s，包含车体旋转、启动和减速过程。复测两组各只完成部分路线，运动段误差的时间长度、路径和停留时间不同；尤其累计平面复测包含急停静止时间，其 RMSE 不能与完整路线直接排名。

## 首目标点云补采（2026-10-28 至 2026-10-29）

两种地图各补采一次成功的首目标运行，各记录 45 帧 `/cloud_deskewed`，每帧最多 2500 点，帧间隔至少 0.4 仿真秒。两次动作均成功到达，但停车速度判据均未通过。此前两组各有一次导航未能在时限内就绪，未采到有效运动点云；失败记录一并保留，原因尚未确定。

对同一帧的同一批点，分别用插值后的 Gazebo 真值位姿和按静止窗口对齐后的里程计位姿投影到世界坐标系，计算水平投影差异。真值采样间隔不超过 0.05 秒，里程计间隔不超过 0.15 秒；不符合条件的帧舍弃。

| 运行 | 有效帧数 | 点数 | 水平投影差异 RMS | 水平投影差异 P95 |
| --- | --- | --- | --- | --- |
| SmallIVox 补采 | 45/45 | 24716 | 4.39 cm | 10.16 cm |
| 累计平面补采 | 44/45 | 24824 | 1.33 cm | 2.60 cm |

这项指标反映采样点受位姿差异影响后的投影偏差，不是墙面厚度、地图重影或扫描内去畸变误差。每组仅一次成功补采、轨迹不完全一致，不能据此认定方案性能改善。脚本的已知旋转、平移及错开时间戳的合成数据自检误差为 `3.6e-16 m`；并未因此验证真实传感器外参。点云、位姿和结果见[数据目录](data/moment_map_gazebo_20261028/)，采集和分析脚本分别为[点云采集器](../../xxu_nav_2026/src/small_point_lio/test/map_benchmark/gazebo_cloud_probe.py)与[投影分析器](../../xxu_nav_2026/src/small_point_lio/test/map_benchmark/analyze_cloud_projection.py)。

## 导航和停车

| 运行 | 已到达目标数 | 真实停点判定 | 停车证据 | 结果 |
| --- | --- | --- | --- | --- |
| SmallIVox 1 | 5/5 | 第 4 个目标 12.9 cm，超过 12 cm | 0/5 | 未通过 |
| SmallIVox 2 | 1/5 | 第 2 个目标未在 105 秒内完成；取消后距目标 8.4 cm | 0/2 | 未通过 |
| 累计平面 1 | 5/5 | 五个真实停点均在 12 cm 内 | 0/5 | 未通过 |
| 累计平面 2 | 0/5 | 第 1 个目标途中停滞；取消时距目标 2.856 m | 0/1 | 未通过 |

停车证据使用真值、里程计、安全速度和轮速在最后 0.75 秒内的时间覆盖及新鲜度：真实位移 ≤5 mm、转角 ≤0.01 rad，里程计平移速度 ≤0.02 m/s、角速度 ≤0.05 rad/s，同时安全命令及轮速近零。13 次评估均发现该窗口内的水平里程计速度超过 0.02 m/s；不能仅用 Nav2 动作状态或车轮零速判定通过。

累计平面第 2 次运行在真值位置约 `(4.322, 2.704)` 观察到 62 个采样周期的 `EmergencyStop`，安全速度为零；采样到的最小激光距离约 0.596 m。检查器随后以 12 仿真秒无进展判定停滞并取消目标。现有数据不能单独证明是地图后端引起急停，也不能排除场景中实际障碍或其他链路因素。
SmallIVox 第 2 次回程期间控制器曾报告无法前进，最后在 105 秒墙钟上限取消；该次未验证剩余三个目标。

## 证据与复现

运行记录、真值/里程计及资源样本、逐目标安全状态、分析结果均保存在 [数据目录](data/moment_map_gazebo_20261028/)。[元数据](data/moment_map_gazebo_20261028/metadata.json)记录源码、数据和实际临时 LIO 动态库的 SHA-256、环境及设置。工作区有未提交修改，不能仅用 Git HEAD 表示被测版本。

- 原始运行结果：
  - [SmallIVox 1](data/moment_map_gazebo_20261028/ivox.navigation.json)；[时间样本](data/moment_map_gazebo_20261028/ivox.samples.csv)；[分析结果](data/moment_map_gazebo_20261028/ivox.analysis.json)。
  - [SmallIVox 2](data/moment_map_gazebo_20261028/ivox_repeat.navigation.json)；[时间样本](data/moment_map_gazebo_20261028/ivox_repeat.samples.csv)；[分析结果](data/moment_map_gazebo_20261028/ivox_repeat.analysis.json)。
  - [累计平面 1](data/moment_map_gazebo_20261028/moment.navigation.json)；[时间样本](data/moment_map_gazebo_20261028/moment.samples.csv)；[分析结果](data/moment_map_gazebo_20261028/moment.analysis.json)。
  - [累计平面 2](data/moment_map_gazebo_20261028/moment_repeat.navigation.json)；[时间样本](data/moment_map_gazebo_20261028/moment_repeat.samples.csv)；[分析结果](data/moment_map_gazebo_20261028/moment_repeat.analysis.json)。
- `*.navigation_samples.json` 是检查器按约 5 Hz 记录的真值、规划与安全状态；后三次的 `*.observer.json` 保存运行参数及加载库指纹。
- [采样器](../../xxu_nav_2026/src/small_point_lio/test/map_benchmark/gazebo_observer.py)只订阅真值、里程计与时钟，并读取 LIO 进程 CPU/RSS；[检查器补丁](../../xxu_nav_2026/src/small_point_lio/test/map_benchmark/gazebo_warmup.patch)只在临时检查器副本中记录地图参数并增加 8 秒静止窗口。
- 分析器用一个已知旋转/平移变换、错开的真值和里程计时间戳进行解析自检，插值对齐误差小于 `1e-12`；它没有验证实际 TF、硬件标定或运动学模型。
- 构建时另 6 个包对仅用于 Small Point-LIO 的 CMake 选项输出“未使用变量”警告；7 个包均构建完成。DDS/时钟及检查器产生的 SIGINT 退出信息不作为传感器故障。

要复现，先在隔离临时目录按本次清单构建上述包，source ROS、工作空间及临时安装；再复制 `tools/navigation/check_ega_navigation.py` 到临时目录并应用补丁。对于每组，将原 EGA `lio_profile.yaml` 加入 `map_backend` 及 `moment_map_resolution: 0.5`，以 `--lio-profile` 传入检查器，同时运行采样器，设 `ROS_DOMAIN_ID=73`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 和唯一 `GZ_PARTITION`。检查器只停止自己启动的进程组。不要让记录器加入真实机器人所在 ROS 域。

本次未采集点云地图用于重影指标，也未验证动态云台、实机、真值标定、长期运行或地图资源上限。补采只覆盖首目标且未建立完整地图重影指标。完整导航回归未通过，因此保留正式默认 SmallIVox，并优先排查静止里程计速度噪声与重复运行的急停/停滞现象。
