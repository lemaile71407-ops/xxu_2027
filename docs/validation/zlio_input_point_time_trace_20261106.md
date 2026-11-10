# capture03：输入点时间单值与整形器产物差异

核验日期：2026-11-06。对象：原 capture03 的 176–180 s 补偿点云、seq1256/1262 的实际更新点、当前安装的整形器，以及现有源码的独立构建。仅分析原录包、执行日志副本回放和合成消息探针；未修改业务源码、安装、默认参数或导航链，未启动新仿真。

## 结论与边界

**已确认进入 ZLIO 前的点时间每帧仅为帧头；当前安装的 `livox_pointcloud_shaper` 对已知非零输入能复现丢弃点偏移，而现有源码的独立构建能保留偏移。** 这确认了候选时序能力与运行产物的差异，尚不单独证明原始扫描的物理时间语义错误。

后续[端到端验证](zlio_point_time_e2e_20261106.md)已通过有时序 RGL 到 ZLIO 的点时间交付，并在新的同源对照中观察到误差下降，但原 46.56 cm 未复现。该轮还发现旧候选 RGL 实际不输出点时间：若源本为瞬时扫描，帧内同时间可能合理，不能仅凭本报告的单值时间把原漂移归因为丢失采样时刻。

- 原包 176–180 s 共 **40 帧、109531 点**，每帧 `timestamp` 的唯一值数量都是 **1**，且全部等于本帧头时间。该现象存在于原始录得的 `/mid360/livox_points_compensated`，不是本次回放、ZLIO 解码或采样才产生。
- 原采集启动明确传入 `rgl_point_timing=uniform`，运行日志也打印此模式；它表示配置意图，不能代替实际点时间交付证据。
- 当前安装程序在 `scan_period=0`、`input_time_offset_field=offset_time` 下，将已知的 `0/50/90 ms` 偏移全部输出为零。相同参数、相同消息交给现有源码的独立构建，则三个偏移全部正确保留。
- **尚未证明此项解释了全部 46.56 cm，也未确认原采集时最早丢失时间的具体边。** 原包只录了补偿后的点云，没有 RGL 原始云、整形器输出的同步副本；当时整形器的二进制摘要也未保存。当前安装程序的行为、路径及较早修改时间是明确线索，不能补写成当时的二进制身份或原始输入证据。

此前“已在前端输出中存在”“传播阶段积累位置偏差”“LiDAR 经位置—速度相关项推偏速度”的结论仍成立。不能默认 capture03 已具备有时序扫描，也不能默认其单值时间必然错误；需区分实际源扫描语义、候选功能交付与原故障因果。

## 1. 从实际更新点回查原消息

在复制的观测和准备源码中，只记录两个固定源时间的原始选点、去畸变点、逐轮目标平面及方差。完整 1 倍速预热回放中，两帧各 5 轮；seq1250–1287 的 38 个后验全部数值字段相对上轮诊断的最大差为 **3.31×10⁻¹³**。两路输入数量及顺序序列化摘要与原包一致。[构建](data/zlio_propagation_update_trace_20261106/point_trace_build02/build.json)、[运行](data/zlio_propagation_update_trace_20261106/point_trace01/run_summary.json)

| 实际更新 | 参考时间 s | 更新点数 | 这些点的时间 s | 点时间唯一值数 | 距参考时间 |
| --- | ---: | ---: | ---: | ---: | ---: |
| seq1256 | 177.687768442 | 1140 | 177.608000000 | 1 | 79.768442 ms |
| seq1262 | 178.407768442 | 1442 | 178.308000000 | 1 | 99.768442 ms |

再直接读取原包 PointCloud2 的 FLOAT64 `timestamp` 字节，按字段 offset、point_step、row_step 和大小端解析，核对 176–180 s 所有 40 帧。时间跨度全部为 0；上述两帧原输入分别有 2391、2193 点，也全部等于帧头。因而不能用“采样恰好只选中同时刻回波”解释这两帧。[检查程序](data/zlio_propagation_update_trace_20261106/inspect_input_point_times.py)、[逐帧字段、范围及消息摘要](data/zlio_propagation_update_trace_20261106/input_point_time_inspection.json)

当前前端解码按配置读取点内时间；输入缓冲只排序和赋 ID，未将每点时间替换成帧头。此处关键证据是**原输入本身已单值化**，不依赖仅从源码推断实际运行行为。[解码](../../xxu_nav_2026/src/z-localization/src/frontend/apps/ros.cpp)、[输入缓冲](../../xxu_nav_2026/src/z-localization/src/frontend/src/input.cpp)

## 2. 采集意图与实际交付不一致

[原采集启动](data/rgl_full_20261031/independent_route_check_20261101/capture_navigation.launch.py)显式选择 `uniform`，前端读取绝对纳秒 `timestamp`；[原运行日志](data/rgl_full_20261031/independent_route_check_20261101/capture03/checker/route01/launch.log)同时记录：

- RGL 模式为 `uniform`；
- `rm_lidar_shaper` 的 `scan_period=0.000`；
- 点云补偿器为 `point_timestamps=False`、`joint_interpolation=False`；
- 整形器可执行文件来自工作区安装路径。

当前[启动源码](../../xxu_nav_2026/src/xxu_bringup/launch/rm_sim_gazebo.launch.py)在 uniform 模式给整形器传 `input_time_offset_field=offset_time`；当前[整形器源码](../../xxu_nav_2026/src/xxu_livox_sim/src/livox_pointcloud_shaper.cpp)的预期行为为 `timestamp=header+offset_time`。配置、源码存在并不等于采集运行时已经交付该功能。

没有原始 RGL 消息，故本轮不能独立证明当时插件确已输出非零偏移或启用了畸变，也不能精确区分首次丢失在插件、整形器还是更早的运行配置。可以确认的是：本次录包没有保留预期的逐点时间；当前安装整形器也无法正确处理已知非零偏移。

## 3. 同参数、同消息的两个二进制探针

在 ROS 域 149 内顺序运行两个整形器，不启动 Gazebo、前后端或导航。输入为单行、3 个有限点、无填充、无过滤的 PointCloud2，头时间 10.125 s；输入 UINT32 `offset_time` 分别为 `0,50000000,90000000 ns`。两程序参数均为 `scan_period=0.0`、`input_time_offset_field=offset_time`、`max_points=12000`。

| 程序 | 输出时间 s | 坐标保留 | 点偏移保留 |
| --- | --- | --- | --- |
| 当前安装产物 | 10.125 / 10.125 / 10.125 | 是 | **否** |
| 当前源码独立构建 | 10.125 / 10.175 / 10.215 | 是 | **是** |

安装路径解析到 `xxu_nav_2026/build/xxu_livox_sim/livox_pointcloud_shaper`；当前摘要为 `0d067a02663d3bf173fdba36cb24c3b5d36c4a8f4ed6c061ba41f986c382c57e`，文件修改时间为 2026-10-02。该时间只是当前文件元数据，不能作为完整历史身份证明。

独立构建直接复制现有源码，不增加业务修改；原源码与副本摘要同为 `89d29fa695bccbde1227f854160effbfcf5d1f7ca7a2f7a5bb39e308fcc3b077`。可执行文件仅位于诊断数据目录，没有覆盖安装产物。

证据：[简单探针程序](data/zlio_propagation_update_trace_20261106/probe_installed_shaper.py)、[安装产物结果](data/zlio_propagation_update_trace_20261106/installed_shaper_probe02/result.json)、[独立构建](data/zlio_propagation_update_trace_20261106/shaper_probe_build/build.json)、[源码构建结果](data/zlio_propagation_update_trace_20261106/source_shaper_probe02/result.json)。首次使用旧有带行填充夹具的探索结果保留在 `installed_shaper_probe01`，其坐标解析还受旧程序布局处理影响，不用于这里的时间因果判别。

## 4. 与当前速度推偏的关系

逐点平面观测及滤波代数核验见[逐轮增量报告](zlio_lidar_increment_trace_20261106.md#7-实际点面观测的独立复算与运动敏感性)。使用当前声明的点时间，若只以真值相对运动替换从点时刻到帧末的变换、保持实际匹配平面与噪声，第一轮速度误差对照如下：

| 水平速度误差 | seq1256 | seq1262 |
| --- | ---: | ---: |
| 实际第一轮 | 0.29877 m/s | 0.71576 m/s |
| 真值相对运动、固定平面 | 0.18818 m/s | 0.58532 m/s |

这表明当前观测对帧内/帧间运动变换敏感，但**不是修复逐点时间的实验**：实际点时间已经塌缩，真值对照仍把整帧视作同一时刻，无法恢复每条原始射线的真实采样时刻。上述改善不能当作时间修复后轨迹，也不能给时间丢失分配完整漂移的百分比。

## 5. 后续端到端验证入口

已完成固定算法、噪声和门限的端到端实验，同步保存 RGL 原始云、两侧整形/补偿云、IMU、真值和 ZLIO 解码日志。实际点级交付、177.81–181.40 s 同源误差及未复现原故障的边界统一见[端到端验证报告](zlio_point_time_e2e_20261106.md)。

不能从原包的单值时间按点序强行补造偏移，也不能以新实验的改善追认原包一定丢失了采样时间。当前未完成原 46.56 cm 故障修复确认、安装验证、完整路线回归、导航验收或默认切换。

本轮核验与保护摘要：[验证文件](data/zlio_propagation_update_trace_20261106/point_trace_verification.json)。
