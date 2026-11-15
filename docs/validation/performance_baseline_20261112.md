# ROS 2 性能基准：环境检查与播放器探针

核验日期：2026-11-12。范围：当前机器和工作区、候选录包的元数据、现有启动入口，以及一次播放器采样探针。本记录不是定位或导航性能验收报告。

## 已核验环境

- Intel Core i7-14650HX，16 个物理核心、24 个逻辑 CPU，x86-64；CPU0 governor 为 `powersave`。CPU 核心性能并不完全相同，后续需记录亲和性、供电、温度和频率。
- Ubuntu 24.04.4 LTS，内核 7.0.0-42-generic，ROS 2 Jazzy。内存约 15 GiB，交换空间 4 GiB；初查可用内存约 6.3 GiB，只是瞬时值。文件系统可用约 38 GiB。
- 工作区有大量既有未提交变更和删除，HEAD 为 `ddba0d64bf289e85d936a513f6d6f37b6501c6c9`。已安装的二进制与当前源码是否一致尚未验证；不得将其归因于该 commit。正式测试须保存源码变更、构建配置、依赖版本和二进制摘要。
- 可用 `ros2 bag`、`psutil 5.9.8` 和 `matplotlib 3.6.3`。未安装依赖、修改算法或运行导航控制器。

## 实际入口和输入

当前主线是 ZLIO 前端、TDT 后端、底盘适配、Nav2。 [RMUL 入口](../../xxu_nav_2026/src/xxu_bringup/launch/rmul_navigation.launch.py)同时启动 Gazebo，因此直接运行它不能隔离录包负载。[统一定位入口](../../xxu_nav_2026/src/z-localization/src/bringup/launch/bringup.launch.py)启动 `frontend/zlio_node` 和 `backend/backend_node`；[底盘适配](../../xxu_nav_2026/src/xxu_bringup/launch/zlio_base_adapter.launch.py)输出里程计和 TF；[导航入口](../../xxu_nav_2026/src/xxu_bringup/launch/navigation.launch.py)包括 Nav2 controller、planner 等节点。[自主建图入口](../../xxu_nav_2026/src/xxu_slam_toolbox/launch/autonomous_mapping.launch.py)组合 slam_toolbox、Nav2 和探索。点云转 LaserScan、静态/动态 TF 与各入口的实际组合仍须在回放前逐项验证；现有文档明确表示新组合建图尚未验收。

候选录包 `xxu_nav_2026/log/velocity_after_yaw_20261111/calibrated_raw01/bag`：MCAP，95.017 秒录制时间，879 条补偿后点云、8795 条补偿后 IMU、88947 条 `/clock`，并混有 TF、旧定位和速度输出。`metadata.yaml` 的 rosbag 压缩字段为空，**不能据此断言 MCAP 内部未压缩**。MCAP SHA-256：`889e2ed5d666c690e78073b6e21a9256901b72b94b71e625f44670c2494a1b47`；metadata SHA-256：`5f83145f5ace0634e8575911bf0465230f98c7b0d895e4a7d8f6c602fba25cdc`。录包位于本机 log 目录，可能不受版本管理；复现时按摘要核对。

此包可用于短时可行性测试，但点云已完成补偿，无法评估补偿链或原始数据解压的完整成本；回放不能发布旧的定位输出或速度命令。95 秒包循环会令录制时钟跳回，不能作为连续一小时建图稳定性的证据。应准备包含完整原始输入、有效 TF 和至少一小时连续时钟的代表性录包；如果只作循环压力试验，须标明每轮重置及结论限制。

## 指标与测试方案

1. 单核满载定义为 CPU 100%，多线程可超过 100%；逐进程采样 CPU 增量、线程数、RSS、PSS、USS 和系统 `MemAvailable`，按 PID 加进程启动时间区分重启。RSS 含共享页，PSS 按共享比例分摊，USS 是独占页；不能将 RSS 之和等同系统总内存。采样读取 `/proc` 本身有成本，须单独评估监测开销。
2. 对同一输入、参数和时钟，先测仅播放，再测定位，再测定位加导航及建图；各阶段记录节点实际启动和生命周期状态。关闭 RViz 作为基线，再单独打开比较。播放、存储读取/解压发生在播放器进程内，单看进程 CPU 无法分离；需要存储插件层计时、系统调用/性能采样或匹配的预解压对照，并保持输入内容和缓存条件一致。
3. 每阶段以 1×、1.5×、2×运行，确认 ROS 回放时间与墙钟时间、话题速率及 QoS。记录预热区间、实际时长、命令、时间戳、失败和资源峰值；多次重复并报告样本数。回放包中 `/clock` 与 `--clock` 只能选择一种一致的时钟来源，压力倍率对仿真时间和消息时间戳的影响需验证。Nav2 仅启动并不等于实际规划控制负载，须设计隔离环境中的可重复目标序列并核验状态，避免接入真实底盘。
4. 外部订阅可测输入和输出的到达频率及接收时间；P95/P99 必须基于明确配对的同一帧与同一时钟域。消息头时间与接收时间之差可能是消息年龄，不等于算法处理耗时；录制时间、仿真时间及墙钟不可直接相减。应在前端接收、入队、出队、处理完成、发布及后端对应位置记录单调时钟和帧标识，分别计算排队、处理及端到端延迟。现有 `FrontendLidarDiagnostic` 包含序号和匹配信息，但没有处理耗时。
5. 对队列积压，需发布前端入队/出队数量、当前和峰值深度及队列满事件，并观察 DDS 消息丢失事件。以唯一帧标识对比播放器发送、订阅接收、处理完成和输出，区分 QoS 丢失、算法拒绝、初始化和故意降采样；频率差不是丢帧计数。现有代码存在队列容量检查，尚无可直接采集的队列深度证据。
6. 一小时稳定性需在代表性连续输入上验证时钟、定位健康状态、任务负载、内存趋势、交换、队列和掉线。测试结束后用有效样本报告 CPU 核数占用及峰值、PSS/USS、`MemAvailable` 和截止期违约率；实时余量应以具体处理周期减实测尾延迟表示，CPU 的空闲百分比不足以代表实时余量。
7. 未来 NUC/ARM 必须原机验证同样的录包、配置、目标和时钟，包括单核和多核吞吐、P95/P99、截止期违约、峰值内存、带宽、I/O、散热降频、供电和持续一小时表现；还要核验 ROS 依赖、指令集、驱动和架构兼容性。本机结果不能单独证明其他设备可用。

## 播放器探针：已执行

以独立 ROS 域 191，1× 仅播放点云、IMU 和录制的 `/clock`，持续 15 秒；未启动定位、Nav2、建图或 RViz。播放器正常退出，采集到 16 行、1 个 PID；首行 CPU 无前一个采样点，故留空。逐秒 CPU 算术均值约 4.2% 单核、最大约 29%；RSS 峰值约 81.6 MiB，PSS 峰值约 49.1 MiB，USS 峰值约 46.9 MiB，最大 17 线程。这些包括启动阶段，只有一次短时观测，**不代表稳定的播放器负载或任何算法性能**。

原始 [CSV](../../xxu_nav_2026/log/performance_20261112/player_probe/process.csv)、[播放器日志](../../xxu_nav_2026/log/performance_20261112/player_probe/player.log)、[图表](../../xxu_nav_2026/log/performance_20261112/player_probe/player_probe.png) 和 [探针脚本](../../xxu_nav_2026/tools/performance/player_probe.py)。脚本通过 `python3 -m py_compile`；本次采样正常完成。复现需确认域 191 未被其他系统使用，并指定一个不存在的输出目录：

```bash
python3 xxu_nav_2026/tools/performance/player_probe.py \
  xxu_nav_2026/log/velocity_after_yaw_20261111/calibrated_raw01/bag \
  /tmp/xxu_player_probe_new --duration 15 --domain 191
```

## 尚未完成

尚未运行定位或完整导航与建图的 1×/1.5×/2×测试、一小时稳定性、端到端延迟和丢帧测试；尚无算法瓶颈分析或部署余量。需要先验证录包与配置匹配、隔离启动组合和话题/TF/QoS，以及安装二进制和源码关系，再开展各阶段。当前文件与一次探针不构成完整可复现性能基准。

## 2026-11-12：按用户要求提前停止的运动录包

使用隔离 ROS 域 192 的 RMUL Gazebo、ZLIO/TDT 和 Nav2，在 `[3,2]` 与 `[2,2]` 之间往返。检查客户端记录 36 个完成目标，状态均为成功；提前停止前未记录定位失信。这不代表一小时运行验收。采集器和本次仿真均已停止，其他 ROS 域的运行进程未停止。

录包接收时间跨度 519.807 秒，仿真时钟跨度 465.419 秒；有 4650 帧补偿点云、46497 条补偿 IMU、1 条静态 TF，未检出 `/clock` 倒退。录制按用户要求中断，不满足 3600 秒校验，保留为短时运动样本。检查计数和时间并不能证明全部消息无损、TF 链完整或轨迹对其他场景有代表性。详见[采集清单](../../xxu_nav_2026/log/performance_20261112/moving_sim_capture_c/manifest.json)、[导航结果](../../xxu_nav_2026/log/performance_20261112/nav_capture_isolated_c/movement/result.json)和[录包目录](../../xxu_nav_2026/log/performance_20261112/moving_sim_capture_c/bag)。

[采集脚本](../../xxu_nav_2026/tools/performance/record_benchmark_bag.py)默认墙钟最多运行 3600 秒，拒绝更长的 `--duration`；还检查空闲磁盘、关键话题，并可监视导航进程。时长包含启动开销，仿真速度也可能低于实时，因此即使运行到上限，录包或仿真时钟跨度仍可能短于一小时；脚本会如实标记未达到 3600 秒。`python3 -m py_compile` 通过，超过一小时的参数被拒绝；短包录制及内容检查已执行。
