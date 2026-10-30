# Small Point-LIO 与 Z-Localization 仿真录包对照（2026-10-30）

## 结论与范围

在 RMUL 场地和固定 EGA 车型的无界面 Gazebo 仿真中，新录制了两段包含点云、IMU、真值和原有里程计的 MCAP。第二段整包以 0.5 倍速完成 Z-Localization 建图回放，地图保存和重新加载检查通过；两套前端在共同输出的 75.8 秒内，与 Gazebo 真值分别作首次位姿对齐后，水平位置 RMSE 为 5.19 cm 和 2.01 cm。**默认固定窗口的先验图定位在整包中途发生后端段错误，因而整包定位测试未通过。**关闭固定窗口的诊断回放虽完成，但出现多次定位状态降级，不能据此认定可以替换现有定位或改善完整导航。

第二次导航目标的 Nav2 动作状态为成功，终点真值误差 0.0875 m，但独立停车判据不通过：停车观测窗口内里程计水平速度峰值 0.152 m/s，阈值为 0.02 m/s。此次 Z-Localization 没有接入 Nav2，因此没有进行两套完整导航架构的闭环性能对照。

## 架构对比

| 项目 | 现有导航链 | Z-Localization 候选 |
| --- | --- | --- |
| 局部里程计 | Small Point-LIO，输出底盘 `/odom` 和去畸变点云 | ZLIO 前端，输出传感器系里程计与同步观测帧 |
| 地图定位 | 当前导航使用二维地图和 AMCL | 独立后端支持三维先验地图定位、回环、重定位及可选 UWB；本次无先验图、无 UWB |
| 导航接入 | 已接入 Nav2、速度转换和安全链 | 本次仅独立回放，未接入 Nav2；传感器系到车体及地图/TF 接口仍需适配和验证 |
| 本次结果 | 导航动作成功但停车判据未通过 | 整包建图完成；默认固定窗口的先验图定位中途崩溃，不能比较闭环导航 |

## 运行条件

- 2026-10-30，Ubuntu 24.04 / ROS 2 Jazzy，RMUL 场地、EGA 固定雷达车型，隔离 ROS 域及 Gazebo 分区；无实车连接。导航目标 `(3, 2)`，目标前在检查器中额外等待 10 秒墙钟时间，非精确 10 秒仿真时间。
- 导航、AMCL 和 Small Point-LIO 使用工作区既有安装；`xxu_bringup` 从当前源码在临时目录单进程重建，以解决既有安装缺少 `interfaces.py` 的启动错误。其余包沿用原有安装，不能将 Git HEAD 视为所有运行二进制的来源。检测到的 `laser_point_cov=0.001`，无改动正式默认参数。
- Small Point-LIO 可执行文件 SHA-256 `95069c504360f5864b701e3e47fcc6bfb2c1c9b1d76e99596bcc16673d968933`，动态库 SHA-256 `f1d4393aa43c16f728fc08e095119840311c42328f51b5083b941413250536d5`。两次录制及回放均设置内存保护阈值 2 GiB；启动包构建设置 `MAKEFLAGS=-j1`。
- 对照数据来自同一份 [第二次录包](data/zlocalization_compare_20261030/navigation_retry_bag/)。Small Point-LIO 的 `/odom` 为实时仿真时录下的结果；Z-Localization 使用同包补偿后点云和 IMU 离线回放，速度 0.5 倍；先做前 40 秒初步检查，后执行整包检查。不同的运行时负载及交付时间不能用于比较实时性或资源消耗。
- Z-Localization 上游提交 `76e126328b670adc92981d9b4900cc059a6f2343`，含本地 Jazzy 兼容改动；前端程序 SHA-256 `5a7225bb0fbc4c60d9e6515ee56e5f003c6ae10d68fb891ddb73602e2a63b4e4`。建图回放的后端使用无先验地图、无 UWB 的 mapping 配置；另使用所建地图进行无 UWB 先验定位检查。回放配置将已补偿到同一虚拟传感器系的点云与 IMU 外参设为单位变换，初始化线程数设为 1；其余 LIO 参数沿用上游配置。点时间字段为绝对 `timestamp`，单位纳秒；IMU 使用 SI 加速度、倍率 1.0。
- RGL 本次输出的每帧点云各点时间相同，不模拟扫描内逐点时间差；固定雷达车型不覆盖真实云台转动、运动去畸变、实车标定或硬件噪声。

## 输入与运行结果

第二次录包包含 960 帧补偿点云、9590 条补偿 IMU、937 条 `/odom` 和 9604 条真值。IMU 最大源时间间隔 0.04 秒，点云最大间隔 0.2 秒，检查未发现上述输入时间回退；[检查结果](data/zlocalization_compare_20261030/retry_bag_inspection.json)记录字段和范围。包的 MCAP 使用 `fastwrite`，读取时提示无消息索引，回放可正常完成。

Z-Localization 回放记录 166 个同步观测帧、1975 条前端里程计，后端产生 1963 条兼容位姿；后端日志记录多次回环接受，也出现 `invalid ZLIO observation frame/time/point layout` 警告。对录得的 166 帧按当时后端输入条件检查，序号 101 的点云为空，其他 165 帧通过这些布局和位姿检查；记录见 [帧检查结果](data/zlocalization_compare_20261030/retry_frame_validation.json) 与 [检查脚本](data/zlocalization_compare_20261030/check_frontend_frames.py)。不能把后端启动和有输出等同于定位状态全程可靠。

同日后续排查确认：序号 101 的观测区间为 33.023–33.076351685 秒，相邻输入点云时间为 33.023 和 33.123 秒，该区间没有新增点；帧的 `lidar_updated=false`、`matches=0`。前端仍发布纯传播帧，原后端将空点云视为无效观测。后端现已改为在帧头、位姿和 XYZ 布局有效时安静跳过这种空传播帧，激光更新为真或布局异常的空帧仍警告。修改后的后端对原始前端观测重新回放：录得序号 100、101、102，101 仍为空但未出现上述警告；空帧区间及后续分别记录到 12 和 30 条后端位姿。[回放日志与输出](data/zlocalization_compare_20261030/skip_empty_check/)保留了本次结果；其中 `frame_validation.json` 按旧的“非空点云”规则检查原始输入，不代表新后端拒绝了该帧。`backend_lifecycle_test` 和 `backend_map_service_test` 均通过，后者同时验证预测连续、后续有效帧及异常空帧报警。该次只重放前端观测，没有重新执行完整导航或实车测试。

原有导航的 [复测结果](data/zlocalization_compare_20261030/capture_retry/result.json)记录了目标、真值和停车证据。该次为一条目标、一次运行：动作成功及停点误差满足 0.12 m 的条件，但停车速度判据未满足，整体 `passed: false`。

## 整包回放与先验图检查（2026-10-30）

使用第二次录包的完整 96.165 秒数据，以 0.5 倍速独立回放，回放前发布初始仿真时钟并延迟启动播放器，确保录制器完成发现。最初一次整包尝试的输出录制器漏记了开头少量输入，不能用于证明完整接收；后续修正后的回放对四个输入话题逐条比较序列化消息，录得 9590/9590 条 IMU、960/960 帧点云、937/937 条原有里程计和 9604/9604 条真值，均无缺失或额外消息。该核对证明录制器观察到完整输入，不能单凭它证明每条消息都送达前端。初始时钟使用 best-effort QoS，与回放时钟兼容。[输入核对](data/zlocalization_compare_20261030/full_bag_20261030_complete/input_payload_comparison.json)、[运行结果](data/zlocalization_compare_20261030/full_bag_20261030_complete/run_summary.json)及[回放脚本](data/zlocalization_compare_20261030/full_bag_20261030_complete/run_full_bag.py)保留了证据。

建图模式播放器正常退出，停止服务和保存地图服务均返回成功。前端输出 638 帧同步观测和 7592 条里程计；其中 4 帧是未发生激光更新的空传播帧，后端未再报告原先的无效观测警告。记录到的后端状态在初始化后为 NORMAL，没有观察到 SUSPECT 或 LOST。保存的地图包含 128 个关键帧和对应点云文件，全局点云读取到 3405 个有限坐标点；重新启动后端时成功加载 128 个关键帧及其描述子。该检查只验证保存和加载，不验证地图几何精度。[输出统计](data/zlocalization_compare_20261030/full_bag_20261030_complete/summary.json)、[地图检查](data/zlocalization_compare_20261030/full_bag_20261030_complete/map_validation.json)和[加载日志](data/zlocalization_compare_20261030/full_bag_20261030_complete/reload.log)记录了结果。

使用相同包生成的地图作为先验图，启用默认 30 秒固定窗口后重新回放：约至包内 55.67 秒时，后端进程发生段错误，未完成整包。内核记录将故障位置指向 GTSAM `ISAM2::marginalizeLeaves`；没有捕获调用栈，尚不能确定底层崩溃根因。进程退出时内存监控仍有约 7.2 GiB 可用，不能将这次崩溃归因于内存不足。[回放结果](data/zlocalization_compare_20261030/prior_localization_20261030/run_summary.json)、[后端日志](data/zlocalization_compare_20261030/prior_localization_20261030/backend.log)和[故障定位证据](data/zlocalization_compare_20261030/prior_localization_20261030/crash_kernel_evidence.txt)可复核；运行结果中的 `ros2 (0)` 是启动包装进程的退出码，后端日志记录了实际子进程 `-11`。

作为隔离诊断，仅在临时配置中将固定窗口设为 0，使用同一输入和先验图重新回放；播放器和停止服务正常完成，整包输入时间戳计数匹配。期间先验定位状态 22 次进入 SUSPECT，随后均恢复 NORMAL；按所录状态消息时间估算，SUSPECT 合计约 4.02 秒。对照支持固定窗口边缘化与崩溃存在关联，但不能证明完整根因；无固定窗口的图可能持续增长，不作为正式配置建议。[诊断结果](data/zlocalization_compare_20261030/prior_no_lag_diagnostic/run_summary.json)、[状态统计](data/zlocalization_compare_20261030/prior_no_lag_diagnostic/state_duration.json)及[实际参数](data/zlocalization_compare_20261030/prior_no_lag_diagnostic/parameter_check.txt)记录了条件和结果。先验图和定位输入来自同一个包，此项并非独立地图泛化测试。

整包建图回放期间设置可用内存低于 2 GiB 时终止，采样最低可用内存约 7288 MiB；前端和后端峰值 RSS 分别约 43.6 MiB 和 86.4 MiB。这些是 0.5 倍速单次回放的采样值，不代表实时运行或与原导航链的资源对比。

整包共同输出区间按下一节所述方法对齐，源时间为 21.223–97.023 秒，共 759 个样本：

| 指标 | Small Point-LIO 前端 | ZLIO 前端 |
| --- | ---: | ---: |
| 水平误差 RMSE | 5.19 cm | 2.01 cm |
| 水平误差 P95 | 8.38 cm | 2.58 cm |
| 最大水平误差 | 9.30 cm | 6.03 cm |

真值首尾水平位移约 0.898 m，不等同于行驶总里程。只比较仿真中初始化后的共同区间前端轨迹，不包含全局定位精度或 Nav2 闭环表现。[整包轨迹结果](data/zlocalization_compare_20261030/full_bag_20261030_complete/trajectory_comparison.json)。

## 前 40 秒初步回放的轨迹对齐

仿真真值按消息源时间插值到 Small Point-LIO 的 `/odom` 时间戳，再将 ZLIO 前端 `/odometry` 插值到相同时间；真值插值间隙上限 0.05 秒、ZLIO 上限 0.15 秒。每条轨迹只用第一个共同样本分别做一次平面 yaw 和 XY 平移对齐，没有对整条轨迹拟合。原有输出是 `odom → base_footprint`，ZLIO 输出是 `odom → imu`；仿真 EGA 模型的虚拟传感器与底盘在 XY 上共址，真实车体必须另行核对。真值为 Gazebo `sentry_bot` 模型位姿。

| 指标 | Small Point-LIO 前端 | ZLIO 前端 |
| --- | ---: | ---: |
| 有效配对样本 | 197 | 197 |
| 水平误差 RMSE | 5.47 cm | 2.17 cm |
| 水平误差 P95 | 8.60 cm | 2.84 cm |
| 最大水平误差 | 9.30 cm | 6.03 cm |

共同源时间区间 21.223–40.823 秒；真值首尾水平位移约 0.838 m。数字只反映该区间及对齐方法；没有覆盖整次导航，更没有比较 Z-Localization 后端位姿或闭环 Nav2 表现。原始结果和计算方法见 [对照 JSON](data/zlocalization_compare_20261030/retry_trajectory_comparison.json) 与 [计算脚本](data/zlocalization_compare_20261030/compare_odometry.py)。

## 首次录制与失败记录

[第一次录包](data/zlocalization_compare_20261030/navigation_bag/) 在雷达和 IMU 中同时出现约 16.26 秒源时间缺口；Small Point-LIO 报 `IMU_TIME_FAULT` 后停止输出，导航首目标中止，[结果](data/zlocalization_compare_20261030/capture/result.json)未通过。整包回放 ZLIO 也报告 IMU 间隔中断；只回放缺口之前的片段时，ZLIO 报告初始化最弱方向的几何支撑不足，未输出里程计。未降低几何验收阈值。

[数据目录](data/zlocalization_compare_20261030/)保留两次录包、回放输出包、日志、回放 YAML 和检查结果。`retry_replay_frontend.log`、`retry_replay_backend.log` 和 `retry_comparison_bag/` 对应第二次包前 40 秒；`prefix_frontend.log` 对应第一次包缺口前的单独回放。检查脚本 [inspect_bag.py](data/zlocalization_compare_20261030/inspect_bag.py) 接受 bag 路径与输出 JSON 路径；输出目录可能包含本机运行产生的日志与地图数据，不应默认提交。
