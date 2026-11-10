# Small Point-LIO 与 Z-Localization 仿真录包对照（2026-10-30）

## 结论与范围

2026-11-03 按地图栅格范围 13.6 × 9.6 m、当前 XY 分量限速各 0.5 m/s 新建参数候选：保留 SUSPECT 5 m，LOST 35→8 m，全局候选补充 5→0，跟踪／恢复累积位移上限 1.5→0.75 m。随后接入实时 Nav2：短目标全项通过，五目标全部到达，真值误差 0.78–4.55 cm，后端水平 RMSE 3.48 cm、最大误差 18.66 cm，无 SUSPECT/LOST 或米级／45° 输出跳变；首点停车里程计速度 0.0532 m/s 超过原 0.02 m/s 门限，整条路线未全项通过。此前同包回放精度基本持平，长路线仍有 2 次 LOST 和 8 次超过 45° 的相邻航向变化，失败证据继续保留。未切换正式默认，完整适用边界和证据见[地图与车速参数候选](data/rgl_full_20261031/map_speed_parameters_20261103/README.md)。

2026-11-02 按用户反馈，将已筛选候选的 SUSPECT 搜索半径从 3 m 改为 5 m，接入同一隔离 Nav2 链实测：短目标全项通过；五目标全部成功到点，真值误差 0.91–4.38 cm，后端水平 RMSE 3.60 cm、最大误差 19.26 cm，没有米级跳变或 LOST。但第 2、4 个目标的停车里程计速度超过原 0.02 m/s 门限，整条路线仍未全项通过。完整参数、一次到点后恢复及比较边界见[5 米候选导航实测](data/rgl_full_20261031/candidate_backend_navigation_20261102/README.md)，正式默认、业务源码和启动链未改。

同日此前的[默认参数导航实测](data/rgl_full_20261031/default_backend_navigation_20261102/README.md)中，短目标进入 LOST 并停车，五目标首点终止；机器人真值静止时全局估计在 0.01 s 内跳变 9.333 m。两轮都未主动注入瞬移或断流；候选还改变了体素、累积帧数及本地保护，且实时输入不同，不能把改善单独归因于搜索半径。

2026-11-01 按用户明确目标验证正常仿真，不主动中断 IMU。前端精度复核及保留 AMCL 的正常导航见[正常仿真对照](#正常仿真与-point-lio-对照2026-11-01)。随后按要求把 Z-Localization 后端接入隔离 Nav2 候选链：单目标通过，五目标测试出现错误全局恢复；启用已有连续性保护后，实际运行进入 LOST 并停车，任务仍未完成。当轮接入、证据和限制见[后端正常导航复核](#后端接入与正常导航复核2026-11-01)，正式默认继续保留旧链，不能将前端精度提升或导航动作成功等同于整套验收完成。

在 RMUL 场地和固定 EGA 车型的无界面 Gazebo 仿真中，新录制了两段包含点云、IMU、真值和原有里程计的 MCAP。第二段整包以 0.5 倍速完成 Z-Localization 建图回放，地图保存和重新加载检查通过；两套前端在共同输出的 75.8 秒内，与 Gazebo 真值分别作首次位姿对齐后，水平位置 RMSE 为 5.19 cm 和 2.01 cm。原后端在默认固定窗口先验图定位时崩溃；本地候选修复后的一次回放完成，但出现约 2.59 m 的恢复跳变。缩小搜索范围的两次同包诊断没有再出现同等幅度的位置误差，仍多次降级。**这些结果不足以支持替换现有定位或证明完整导航性能改善。**

第二次导航目标的 Nav2 动作状态为成功，终点真值误差 0.0875 m，但独立停车判据不通过：停车观测窗口内里程计水平速度峰值 0.152 m/s，阈值为 0.02 m/s。此次 Z-Localization 没有接入 Nav2，因此没有进行两套完整导航架构的闭环性能对照。

## 正常仿真与 Point-LIO 对照（2026-11-01）

本轮目标限定为正常仿真运行及相对 Small Point-LIO 的实际效果，不进行 IMU 暂停、转发丢弃或定位失信注入。没有修改补偿算法、ZLIO 源码、正式前后端参数或默认启动链；正常导航沿用既有 AMCL、二维地图、Nav2、碰撞监测及最终速度安全门，未接入 Z-Localization 全局后端。

### 同包 1 倍速前端复核

使用原 `navigation_retry_bag` 的完整 96.165 秒正常仿真输入，当前隔离候选前端连续进行两次独立 1 倍速回放。输入包括原始录得的补偿点云、补偿 IMU、Gazebo 真值及 Small Point-LIO 里程计；Point-LIO 基线是同一传感器序列实时仿真时的录制结果，本轮没有重新执行该前端，也不能比较两者资源消耗。当前 ZLIO 二进制与前一轮连续仿真验证一致，配置沿用该轮临时 0.5 m 初始化网格，未以本次误差反向调参。

两轮均逐话题核对输入数量和顺序序列化 SHA-256：点云 960/960、补偿 IMU 9590/9590、真值 9604/9604、原有里程计 937/937，全部一致。ZLIO 每轮输出 7592 条里程计，源时间最大间隔 0.01 s、无重复或倒退，代次均保持 0，日志没有输入中断。录制器完整接收不等同于逐条证明前端接收；持续输出和未重置是本轮额外运行证据。

对比先根据录得静态 TF 将 ZLIO 虚拟传感器位姿换算至 `base_footprint`，再分别用首次共同样本的平面位置和航向对齐真值，不作整条轨迹拟合或尺度拟合。Gazebo 模型原点与底盘原点关系由录得 link 位姿核对，矩阵元素最大差约 `5.05e-15`。共同区间为 21.223–97.023 秒、759 个采样点，真值水平累计路径约 4.137 m；以下只描述初始化后的该区间。

| 指标 | 录得 Small Point-LIO | 当前 ZLIO，两次独立回放 |
| --- | --- | --- |
| 水平位置 RMSE | 5.186 cm | 1.080 cm / 1.080 cm |
| 水平位置 P95 | 8.366 cm | 1.593 cm / 1.593 cm |
| 最大水平位置误差 | 9.300 cm | 5.038 cm / 5.038 cm |
| 航向误差 RMSE | 0.3785° | 0.0429° / 0.0429° |
| 一秒相对平移误差 RMSE | 4.918 cm | 0.954 cm / 0.954 cm |
| 一秒相对航向误差 RMSE | 0.5282° | 0.0344° / 0.0344° |

该输入上水平 RMSE 降低约 79.18%，两轮结果一致，支持当前候选的前端精度改善；不外推至其他路线、独立先验图或实车。[运行程序](data/rgl_full_20261031/normal_pointlio_comparison_20261101/run_replay.py)、[分析程序](data/rgl_full_20261031/normal_pointlio_comparison_20261101/compare_normal.py)、[第一次结果](data/rgl_full_20261031/normal_pointlio_comparison_20261101/run01/trajectory_comparison.json)、[重复结果](data/rgl_full_20261031/normal_pointlio_comparison_20261101/run02/trajectory_comparison.json)及[对照图](data/rgl_full_20261031/normal_pointlio_comparison_20261101/run01/trajectory_comparison.png)保留原始证据。两个播放器和各自录制/前端进程均正常退出，最低可用内存分别约 8257、8223 MiB。

### 加入诊断后的精度复核

2026-11-01 再次执行一轮独立 1 倍速回放，使用新增诊断的迁移候选并开启诊断输出；同时重新计算上一轮
诊断开启/关闭录包的精度。三份结果使用同一个分析程序及上述坐标转换、首次位姿对齐和共同时间采样口径，
当前候选二进制、参数和各输入话题的有序序列化指纹均已核对，未调参、未注入断流、未启动全局后端。

新回放仍有 759 个共同样本、75.8 秒评估区间、约 4.137 m 真值累计水平路径。
ZLIO 水平 RMSE **1.080 cm**、P95 **1.593 cm**、最大误差 **5.038 cm**，航向 RMSE **0.0429°**，
一秒相对平移 RMSE **0.954 cm**、相对航向 RMSE **0.0344°**；同包录得 Point-LIO 的水平 RMSE 为 **5.186 cm**，
水平 RMSE 降低约 **79.18%**。诊断开、关及本次新回放的全部上述指标一致；相对加入诊断前候选的
RMSE 差异仅为约 `1e-16 m` / `1e-15°` 的浮点数值量级，本次没有观察到精度回退。

新回放 637 个观测后验与上次诊断开启运行逐字节一致，637 条诊断全部对齐；按录包原始消息顺序核对
7592 条高频里程计，源时间严格递增、最大间隔 0.01 s，代次保持 0，未报告输入中断。
所有子进程退出码为 0，最低可用内存约 7975 MiB。本轮验证范围是该正常输入上初始化后的局部前端轨迹；
原 Nav2 12.51 cm 到点误差及前两个停车窗口的速度超限没有重测，完整导航验收状态不变。

证据：[新回放精度结果](data/rgl_full_20261031/precision_recheck_20261101/run01/trajectory_comparison.json)、
[轨迹与误差图](data/rgl_full_20261031/precision_recheck_20261101/run01/trajectory_comparison.png)、
[重复性与原始时序核验](data/rgl_full_20261031/precision_recheck_20261101/verification.json)、
[核验程序](data/rgl_full_20261031/precision_recheck_20261101/verify_precision.py)、
[执行指纹](data/rgl_full_20261031/precision_recheck_20261101/execution_manifest.json)、
[运行结果](data/rgl_full_20261031/precision_recheck_20261101/run01/run_summary.json)。
复算对照：[诊断开启](data/rgl_full_20261031/lidar_update_diagnostics_20261101/enabled01/trajectory_comparison.json)、
[诊断关闭](data/rgl_full_20261031/lidar_update_diagnostics_20261101/disabled01/trajectory_comparison.json)。

### 正常 Nav2 到点与停车

通过[隔离候选启动入口](data/rgl_full_20261031/normal_pointlio_comparison_20261101/normal_navigation.launch.py)连接当前 ZLIO、已有底盘/点云适配器和原 AMCL/Nav2。沿用 EGA/RMUL 车型与地图、1.5 rad/s 小陀螺配置、原导航几何参数及 12 cm 真值到点限值。复用[原导航检查器](../../xxu_nav_2026/tools/navigation/check_ega_navigation.py)；仅为其增加可选候选 launch/估计器参数入口及 3 GiB 内存保护，未改变动作、到点或停车判据。候选前端需先产生 `/odom` 才启动 Nav2，扫描来自前端去畸变点云。

| 运行 | 导航动作 | 到点真值误差 | 原严格停车检查 | 整体结果 |
| --- | --- | --- | --- | --- |
| nav01，单目标 `(3,2)` | 1/1 成功，无卡住 | 8.91 cm | 1/1 通过 | 通过 |
| nav02，原检查器五目标路线 | 5/5 成功，无卡住 | 各点约 12.51、2.75、7.99、6.61、4.35 cm，取动作完成/停稳误差较大值 | 3/5 通过 | 未通过 |

nav02 的第一个点超出原 12 cm 限值约 5.1 mm；前两个停车窗口的里程计水平速度峰值分别约 0.0274、0.0291 m/s，超过原 0.02 m/s 限值，但这些窗口中 Gazebo 真值位置与转角均不再变化，安全输出为零，轮速接近零。其余三个停车窗口通过。不能将里程计速度噪声直接视为底盘还在运动，也不能因物理停车而删除估计器验收失败项；没有放宽限值或按结论修改算法。此前 Point-LIO 同目标历史测试的停车速度峰值为 0.152 m/s，但不同运行不构成两套完整导航系统的严格性能对照。

[nav01 结果](data/rgl_full_20261031/normal_pointlio_comparison_20261101/nav01/result.json)、[nav02 结果](data/rgl_full_20261031/normal_pointlio_comparison_20261101/nav02/result.json)及各自 `launch.log`、`samples.json` 保留实际执行证据。两次最低可用内存分别约 6176、6245 MiB。本轮未主动中断 IMU；两个正常导航日志均未报告前端输入中断，结束后的 SIGINT/退出日志不能当作运行中故障。仿真和导航进程均已停止。

**当前可确认前端在此正常仿真输入上优于原 Point-LIO，且 ZLIO + 原 AMCL/Nav2 的正常五目标动作可以跑通；整体严格验收仍有上述到点和停车估计问题，正式默认仍保留旧链。** 后续只围绕正常运行中的实际问题收敛；不以六秒主动断流恢复作为本轮性能结论。候选入口依赖本轮加载的 ROS/隔离安装环境和候选 RGL 库，属于可复现验证入口，尚非正式安装发布入口。

## 逐次 LiDAR 更新诊断（2026-11-01）

为继续定位正常运行中的估计问题，在迁移后的前端增加可选 `/frontend/lidar_diagnostics`，启动参数
`publish_lidar_diagnostics` 默认关闭。只在隔离安装中验证，没有替换正式默认程序、改变匹配/滤波算法或放宽导航判据。
这是诊断接入，不是上述到点误差或停车速度估计问题的已确认修复。

每条消息记录最后一次实际线性化的纯 LiDAR 6×6 信息矩阵、有效匹配/采样数量、迭代次数、是否接受校正、
同一时刻的预测与后验、世界系位置/旋转向量校正量，以及先验/后验协方差。
保留求解器原生信息与换算至物理位姿的信息；包含物理 6×6 位姿协方差和完整 15×15 内部协方差。
矩阵按行展开，使用进程/代次/观测序号对齐原子帧。坐标、单位、线性化锚点和开启方法以
[前端接口说明](../../xxu_nav_2026/src/z-localization/src/frontend/README.md#可选-lidar-更新诊断)及消息定义为准。
该输出覆盖初始化完成后的已处理观测，不覆盖初始化联合优化窗口；不能仅凭混合米/弧度单位的特征值认定退化。

当前实际验证：

- 消息接口及前端隔离构建通过；四个测试程序共 28 个用例通过，包括非对称矩阵行列序、坐标协方差、诊断开关一致性和零匹配回退。
- 对同一份 96.165 秒正常录包，开启/关闭诊断各执行一次 1 倍速回放，均不注入 IMU 断流、不启动全局后端。
  两轮补偿 IMU 9590、点云 960、真值 9604、原有里程计 937，数量与有序序列化 SHA-256 完全一致。
- 开启时 637 个原子观测帧对应 637 条诊断，无重复/缺失；其时间、身份、匹配数、校正状态和后验里程计全部对齐。
  两轮 637 个后验里程计连同协方差逐字节一致，匹配数及校正状态一致。
- 633 次接受校正，4 次未接受校正的采样数和匹配数均为零；对应序号/时间保存在核验结果中，不能仅据零匹配将其归因为配准退化。
  所有矩阵有限，对称性与数值半正定检查通过；完整协方差到物理位姿协方差的换算和预测/校正位姿差检查通过。
- 两轮代次均为 0，日志没有输入中断或诊断发布失败；所有子进程退出码为 0。
  最低可用内存分别约 7069、7073 MiB。仅证明本次输入上的诊断正确性与后验一致性，不代表完成新的导航验收或根因确认。

证据：[核验结果](data/rgl_full_20261031/lidar_update_diagnostics_20261101/verification.json)、
[分析程序](data/rgl_full_20261031/lidar_update_diagnostics_20261101/verify_diagnostics.py)、
[构建与源码指纹](data/rgl_full_20261031/lidar_update_diagnostics_20261101/build_verification.json)、
[测试日志](data/rgl_full_20261031/lidar_update_diagnostics_20261101/frontend_tests.log)、
[开启运行](data/rgl_full_20261031/lidar_update_diagnostics_20261101/enabled01/run_summary.json)、
[关闭运行](data/rgl_full_20261031/lidar_update_diagnostics_20261101/disabled01/run_summary.json)。
两轮各自 `bag/` 保留原始记录，运行进程均已停止。

## 后端接入与正常导航复核（2026-11-01）

按用户要求，使用新增诊断的迁移前端与既有后端，组成 `RGL/IMU → ZLIO → 先验图后端 → Nav2` 隔离候选链。
已有底盘适配器提供 `odom → base_footprint` 和导航扫描，后端独占 `map → odom`；没有启动 AMCL/GICP。
前端开启协调重置和逐观测诊断，最终速度门开启 `require_localization=true`，沿用原碰撞监测、车型参数、
12 cm 外部真值到点和严格停车检查。本轮没有主动暂停、丢弃或伪造 IMU；未修改估计算法、正式安装或默认启动链。

历史三维先验图与二维导航地图原点不同。复用已有 `transform_map_bundle` 工具，将先验图候选副本转换到仿真世界/导航地图坐标，
变换沿用[历史先验来源录包的首关键帧锚点](data/rgl_full_20261031/prior_anchor.json)，没有用当前运行轨迹拟合。
原地图保留；128 个关键帧变换核对通过，局部点云逐文件字节相同。
该对齐依赖来源仿真真值，不能当作实车独立测绘精度证据。

### 实时正常运行

| 运行 | 恢复连续性保护 | 实际结果 | 原严格验收 |
| --- | --- | --- | --- |
| nav01，目标 `(3,2)` | 未启用 | 动作成功，最大终点真值误差约 5.40 cm，停车通过 | 通过 |
| nav02，原五目标路线 | 未启用 | 5 个动作均返回成功；真值误差分别约 0.102、9.855、8.070、9.020、9.779 m；停车 4/5 通过 | 未通过 |
| nav03_guarded，目标 `(6.59,4.44)` | 已启用 | 自然跟踪失信后进入 LOST；底盘停在约 `(5.479,2.657)`，距目标 2.101 m；检查器判定卡住并取消，停车通过 | 未通过 |

nav01 的后端底盘位姿直接与真值比较，水平 RMSE 约 2.73 cm、最大约 5.92 cm；没有再次作首次位姿对齐。
nav02 的全局位姿水平 RMSE 约 6.66 m、最大约 9.98 m，最大航向误差接近 180°，确认是错误定位，不能以动作成功视为到点。
同轮前端仅保持代次 0，首次位姿对齐后的局部水平 RMSE 约 4.88 cm、最大约 18.59 cm；
这与此前短录包的 1.08 cm 属于不同路线与范围，不能直接作为性能回退结论。
前端局部误差和后端错误全局位姿的量级差异，连同后端状态时间线，将当前排查重点指向后端跟踪/恢复路径。

nav02 首次失败在后端运行关键帧 22：534/723 个对应点、信息秩 6，16 次迭代结束仍未收敛；
同一录包的 DEBUG 回放中末次平移步长约 0.2208 m、旋转步长约 0.7081°，随后进入 SUSPECT 并执行 Scan Context 恢复。
恢复重新声明可信后，真值对照出现明显错误位姿。没有发现 AMCL 争抢 TF 或 IMU 断流：
三轮观测到的 TF 父关系均为 `map → odom → base_footprint`，原始/补偿 IMU 源时间最大间隔均为 0.01 s。
**恢复日志的 `delta_r_deg` 相对冻结的最后可信位姿，包含冻结期间的实际运动，不能直接等同于同一时刻的错误校正量。**
初始配准为什么不收敛、恢复为什么选择错误位置，仍需提取实际源/目标点云及初值进一步确认，未修改求解器试探。

### 现有连续性保护的对照与边界

先保留 nav01/nav02 原参数快照，再以 nav02 录包前 60 秒墙钟区间进行两次后端独立回放。
两次 `/frontend/frame` 374 条、`/frontend/odometry` 3391 条及其他回放输入的数量与有序序列化 SHA-256 完全一致；
没有启动仿真、Nav2 或实时前端重置服务，不能把这两次回放当作跨代次恢复测试。

仅切换已有 `recovery.continuity_guard`；启用时沿用此前保护候选实验的 0.5 m / 30° 阈值，不是正式推荐参数。
未启用的回放再次出现错误可信位姿；启用后记录 29 次同一时刻恢复校正拒绝并经历 LOST，也有后续恢复。
按接收顺序近似关联可信状态，两个回放的可信全局水平 RMSE 分别约 1.90 m / 5.15 cm；
此关联不是后端内部原子快照，且输入是固定轨迹，不能据此宣称完整导航稳定性已修复。

随后 nav03_guarded 以正常实时输入复核保护：后端进入 LOST 并触发两次前端重置，代次 0→1→2；
没有注入 IMU 故障。状态持续不可信超过 200 ms 后，录包未观察到非零安全速度输出；
最终真值位移/转角为零、安全输出为零、轮速接近零，原停车检查通过。
这说明现有门控在本次自然失信场景中可以停车；它不能识别后端错误地声明 `pose_trusted=true` 的位姿。

候选[启动入口](data/rgl_full_20261031/backend_integration_20261101/backend_navigation.launch.py)当前指向
[已启用保护的候选参数](data/rgl_full_20261031/backend_integration_20261101/backend.yaml)，仅供继续隔离诊断，尚不能替换正式导航。
该入口复用临时前端/后端安装、RGL 插件和既有地图，尚非正式安装发布入口。
最低可用内存 nav01/nav02 分别约 5095/5001 MiB；每轮均设置 3 GiB 下限。
检查器 nav01 返回 0，nav02/nav03_guarded 因验收失败返回 1，录制器均返回 0；
运行后的 SIGINT/退出错误日志属于清理，不能等同于运行中节点崩溃。运行进程均已停止。

证据：[实时运行与录制程序](data/rgl_full_20261031/backend_integration_20261101/run_navigation.py)、
[地图核验](data/rgl_full_20261031/backend_integration_20261101/map_verification.json)、
[nav01 分析](data/rgl_full_20261031/backend_integration_20261101/nav01/analysis.json)、
[nav02 分析](data/rgl_full_20261031/backend_integration_20261101/nav02/analysis.json)、
[保护运行分析](data/rgl_full_20261031/backend_integration_20261101/nav03_guarded/analysis.json)、
[回放对照](data/rgl_full_20261031/backend_integration_20261101/replay_verification.json)、
[分析程序](data/rgl_full_20261031/backend_integration_20261101/analyze_navigation.py)。
各运行目录的 `bag/`、`backend.yaml` 和 `checker/<运行名>/result.json`、`launch.log` 保留原始输入、参数与实际结果。

## 架构对比

| 项目 | 现有导航链 | Z-Localization 候选 |
| --- | --- | --- |
| 局部里程计 | Small Point-LIO，输出底盘 `/odom` 和去畸变点云 | ZLIO 前端，输出传感器系里程计与同步观测帧 |
| 地图定位 | 当前导航使用二维地图和 AMCL | 独立后端支持三维先验地图定位、回环、重定位及可选 UWB；本次无先验图、无 UWB |
| 导航接入 | 已接入 Nav2、速度转换和安全链 | 本次仅独立回放，未接入 Nav2；传感器系到车体及地图/TF 接口仍需适配和验证 |
| 本次结果 | 导航动作成功但停车判据未通过 | 整包建图完成；原后端先验定位崩溃，本地候选修复后仍有定位降级和恢复跳变，不能比较闭环导航 |

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

## 固定窗口修复与恢复搜索诊断（2026-10-30）

对原后端的整包故障进行了 GDB 检查，调用栈经过 `PoseGraph::updateWindow` 和 GTSAM `ISAM2::marginalizeLeaves`。另外构造互不连接的图分量，在旧实现中让一整个分量过期，可稳定触发同一函数内同一指令位置的段错误；这一证据支持两者有关，但未检查真实回放中的全部图分量，不能断言已完全确定实包根因。[整包 GDB 日志](data/zlocalization_compare_20261030/prior_gdb_20261030/)保留在本地数据目录。

独立的 Z-Localization 工作树中，[候选修复](../../z-localization/src/backend/src/pose_graph.cpp)在边缘化前识别并移除完全过期且内部依赖完整的独立分量，保留其余分量的固定窗口边缘化；不满足依赖条件时按数值错误处理。新增[图测试](../../z-localization/src/backend/test/integration/graph_test.cpp)覆盖独立分量过期及与连通分量同时过期，并与无窗口图的位姿和协方差对照。候选后端和相关测试目标在本机构建成功，`backend_graph_test`、`backend_innovation_test`、`backend_localization_test` 共 3 个 CTest 目标通过。该候选只使用本地构建目录运行，未安装或接入正式导航；上述源码链接依赖本地独立仓库。

同一先验图、同一 96.165 秒录包以 0.5 倍速回放候选后端，播放器退出码为 0，停止服务成功，未复现段错误。录制器观察到的 IMU、点云、原里程计和真值的数量及时间戳计数均与源包一致；这不证明前端逐条接收。默认 30 秒窗口下，日志记录 18 次进入 SUSPECT，状态采样估计 SUSPECT 共约 3.17 秒；两次恢复修正分别达到 2.5905 m / 83.6753° 和 2.55709 m / 62.9743°，随后状态仍转为 NORMAL。[运行结果](data/zlocalization_compare_20261030/prior_fixed_lag_candidate_20261030/run_summary.json)、[输入检查](data/zlocalization_compare_20261030/prior_fixed_lag_candidate_20261030/input_comparison.json)、[状态统计](data/zlocalization_compare_20261030/prior_fixed_lag_candidate_20261030/state_duration.json)和[后端日志](data/zlocalization_compare_20261030/prior_fixed_lag_candidate_20261030/backend.log)可复核。

在临时配置中将 SUSPECT 候选搜索半径设为 1 m、全局回退候选数设为 0，再做一次整包回放；其他参数及正式默认配置未改。复核代码确认 SUSPECT 分支原本就将全局回退候选数固定为 0，临时设置的全局回退参数影响其他恢复状态，本次诊断未观察到 LOST，不能将变化归因于该参数。该次播放器和停止服务均成功，录制器观察到的输入时间戳计数与源包一致；日志记录 25 次进入 SUSPECT，状态采样估计共约 3.65 秒，恢复修正最大平移约 0.158 m、最大旋转约 21.37°。这些参数只用于诊断，不能保证恢复位姿正确，也不能保证 LOST 后的搜索范围受同样限制。[运行结果](data/zlocalization_compare_20261030/prior_recovery_radius_diagnostic_20261030/run_summary.json)、[临时参数](data/zlocalization_compare_20261030/prior_recovery_radius_diagnostic_20261030/backend.yaml)、[状态统计](data/zlocalization_compare_20261030/prior_recovery_radius_diagnostic_20261030/state_duration.json)与[后端日志](data/zlocalization_compare_20261030/prior_recovery_radius_diagnostic_20261030/backend.log)记录此次运行。

对两次候选后端输出，分别在第一个共同样本按水平位置和航向对齐 Gazebo 真值，以原里程计 10 Hz 时间点插值比较；共同区间为 21.423–97.023 秒、757 个样本。默认搜索配置下，后端水平 RMSE 为 0.251 m、P95 为 0.075 m、最大误差为 2.587 m；临时搜索配置下分别为 0.052 m、0.080 m、0.147 m。原 Small Point-LIO 对同一真值的水平 RMSE 为 0.053 m、最大误差为 0.096 m。[默认对照](data/zlocalization_compare_20261030/prior_fixed_lag_candidate_20261030/backend_truth_comparison.json)和[临时参数对照](data/zlocalization_compare_20261030/prior_recovery_radius_diagnostic_20261030/backend_truth_comparison.json)包含方法及结果。这里的首次对齐只比较相对轨迹，不证明全局地图坐标精度；地图由同一录包生成，不能据此认定搜索参数在其他场景中有效。尚未验证实车外参、独立先验图、定位降级时导航保护及 Nav2 闭环，不切换正式定位链。

## 搜索边界与重复回放（2026-10-30）

检查召回及配准流程发现，`suspect_search_radius` 约束的是召回时地图关键帧距预测位置的距离，而不是最终配准恢复位姿距预测位置的硬限制。新增[重定位测试](../../z-localization/src/backend/test/integration/relocalization_test.cpp)，构造半径内关键帧对应半径外真实恢复位姿，验证该结果仍能匹配成功；`backend_relocalization_test` 在加载 ROS Jazzy 和独立仓库依赖环境后通过。首次未加载该环境的测试因 `libusb_set_option` 动态链接失败，不计为测试通过。该测试用于确认现有行为，不构成恢复保护或参数修改。

使用同一输入、先验图和临时参数重复整包回放一次，播放器与停止服务成功，录制器观察到的四个输入话题数量和时间戳计数匹配。此次日志记录 21 次进入 SUSPECT，状态采样估计合计约 2.62 秒；后端相对真值的水平 RMSE 为 0.049 m，最大误差为 0.147 m，日志中的最大恢复平移和旋转分别约为 0.119 m 和 11.46°。[重复运行结果](data/zlocalization_compare_20261030/prior_recovery_radius_repeat_20261030/run_summary.json)、[输入检查](data/zlocalization_compare_20261030/prior_recovery_radius_repeat_20261030/input_comparison.json)、[状态统计](data/zlocalization_compare_20261030/prior_recovery_radius_repeat_20261030/state_duration.json)和[轨迹比较](data/zlocalization_compare_20261030/prior_recovery_radius_repeat_20261030/position_heading_status_comparison.json)保留证据。重复结果支持该录包上位置误差有所改善，但没有对默认搜索配置进行重复，也不覆盖其他路线或先验地图。

补充相同对齐与采样条件下的航向分析：默认搜索一次回放的最大相对航向误差为 72.95°；两个临时参数回放分别为 29.42° 和 19.14°。按录包时间近似关联最近 0.3 秒内的状态，两个临时参数回放中航向误差超过 10° 的样本分别为 26 和 20 个，关联到的状态均为 SUSPECT、不可信；冻结位姿与机器人实际转动会导致此差异，不能把它直接等同于正常跟踪精度。默认搜索回放中的 27 个此类样本，有 6 个近似关联到 NORMAL、`pose_trusted=true`。状态和位姿并非严格同步，该统计仅作诊断，不作为逐帧安全判定。[计算脚本](data/zlocalization_compare_20261030/compare_backend_position_heading.py)、[默认结果](data/zlocalization_compare_20261030/prior_fixed_lag_candidate_20261030/position_heading_status_comparison.json)及[首次临时参数结果](data/zlocalization_compare_20261030/prior_recovery_radius_diagnostic_20261030/position_heading_status_comparison.json)可复核。后续需为恢复结果设计并验证可解释的位姿连续性检查，同时在导航侧处理状态不可信、消息过期及恢复后的重新放行；不能仅以缩小候选搜索或收到 TF 作为安全条件。

整包共同输出区间按下一节所述方法对齐，源时间为 21.223–97.023 秒，共 759 个样本：

| 指标 | Small Point-LIO 前端 | ZLIO 前端 |
| --- | ---: | ---: |
| 水平误差 RMSE | 5.19 cm | 2.01 cm |
| 水平误差 P95 | 8.38 cm | 2.58 cm |
| 最大水平误差 | 9.30 cm | 6.03 cm |

真值首尾水平位移约 0.898 m，不等同于行驶总里程。只比较仿真中初始化后的共同区间前端轨迹，不包含全局定位精度或 Nav2 闭环表现。[整包轨迹结果](data/zlocalization_compare_20261030/full_bag_20261030_complete/trajectory_comparison.json)。

## 恢复连续性候选检查（2026-10-31）

在本地独立 Z-Localization 工作树中，新增可选的[恢复提交检查](../../z-localization/src/backend/src/localization_recovery.cpp)：匹配完成后、写入恢复锚之前，将匹配位姿与捕获帧的 `map → odom` 加里程计预测在同一时刻比较，检查三维平移及三维旋转角。开启后，`SUSPECT` 和 `LOST` 均受约束；超限按恢复失败处理，保持不可信，并沿既有失败计数升级状态。两个阈值缺失、无效或没有同代次可信参考时也拒绝恢复。初始化无历史参考，不执行该检查；里程计预测本身不可靠时，该检查不能保证恢复正确。前端代次改变后若缺少同代次可信参考，也可能持续无法自动恢复，需要后续设计可信重置和恢复流程。

参数 `recovery.continuity_guard.enabled` 默认关闭；启用时须同时提供正且有限的 `recovery.continuity_guard.max_translation`（米）和 `recovery.continuity_guard.max_rotation_deg`（度）。正式配置没有改动。相关[集成测试](../../z-localization/src/backend/test/integration/localization_test.cpp)覆盖异常平移、异常旋转、升级至 LOST 后继续拒绝、有效恢复、缺失阈值和匹配期间新帧到达；`backend_localization_test`、`backend_graph_test`、`backend_relocalization_test` 三个 CTest 目标在本地构建目录通过。此检查仅覆盖恢复路径，不覆盖初始化误匹配、正常跟踪误匹配、导航状态超时和停车控制。

在同一先验图和原始 96.165 秒包上，以 0.5 倍速启用临时阈值 0.5 m / 30° 做一次独立回放，[实际参数](data/zlocalization_compare_20261030/prior_continuity_guard_20261031/parameter_check.txt)已从运行节点读取；该数值仅用于实验，并非实车阈值。播放器与停止服务成功，录制器观察到的四个输入话题数量和时间戳计数与源包一致，后端日志未记录段错误。日志记录 25 次 SUSPECT，状态采样估计合计约 3.61 秒，未观察到 LOST；本次没有触发连续性拒绝，因此不能据此证明检查挡住了旧回放中的大跳变。[运行结果](data/zlocalization_compare_20261030/prior_continuity_guard_20261031/run_summary.json)、[输入检查](data/zlocalization_compare_20261030/prior_continuity_guard_20261031/input_comparison.json)、[状态统计](data/zlocalization_compare_20261030/prior_continuity_guard_20261031/state_duration.json)和[后端日志](data/zlocalization_compare_20261030/prior_continuity_guard_20261031/backend.log)记录此次回放。

按照原报告的首次平面位姿对齐与插值方法，在 756 个共同样本上，后端水平 RMSE 为 0.051 m，最大误差为 0.150 m；按录包时间近似关联最近 0.3 秒的状态，NORMAL 且可信的 723 个样本中最大水平误差约 0.150 m、最大相对航向误差约 5.43°。这些是单次同包、同图结果，状态关联并非严格同步，不能据此判定连续性检查带来性能提升或允许接入导航。[分析结果](data/zlocalization_compare_20261030/prior_continuity_guard_20261031/position_heading_status_comparison.json)和[计算脚本](data/zlocalization_compare_20261030/compare_backend_state_conditioned.py)保留方法。

另以 0.5 倍速回放同一包的 32 秒前缀，临时使用刻意收紧的 0.000001 m / 0.00001° 阈值，只验证实际拒绝路径，不作为可用参数。日志记录三次恢复结果被门拒绝后升级为 LOST；随后前端连续重置至代次 1 和 2，47 次匹配因缺少可用的同代次参考等条件被拒绝。录得状态在 LOST 时均为 `pose_trusted=false`、`tf_frozen=true`，状态采样的 LOST 时长约 8.02 秒直到回放结束；`/localization/odometry` 在此期间仍有输出，消费者不能以话题存在作为定位可信依据。播放器和停止服务成功；此实验只验证拒绝与不可用状态，未验证从新代次安全恢复，也未验证导航停车。[实验参数](data/zlocalization_compare_20261030/prior_continuity_forced_rejection_20261031/backend.yaml)、[运行结果](data/zlocalization_compare_20261030/prior_continuity_forced_rejection_20261031/run_summary.json)、[状态统计](data/zlocalization_compare_20261030/prior_continuity_forced_rejection_20261031/state_duration.json)和[后端日志](data/zlocalization_compare_20261030/prior_continuity_forced_rejection_20261031/backend.log)保留证据。

## 工作区源码副本定位回放（2026-10-31）

使用 [工作区源码副本](../../xxu_nav_2026/src/z-localization/)隔离构建的前端和后端，以 0.5 倍速回放同一份 96.165 秒仿真录包；先验地图仍由该录包生成，不是独立地图。点云和 IMU 使用已补偿的共同虚拟传感器系，外参为单位变换；不连接实车或 Nav2。后端实际读取的固定窗口为 30 秒、SUSPECT 搜索半径为 30 米，恢复连续性检查保持默认关闭。[程序与配置摘要](data/zlocalization_compare_20261030/src_localization_check_20261031/binary_and_config_sha256.json)和[运行时参数](data/zlocalization_compare_20261030/src_localization_check_20261031/parameter_check.txt)记录了本次对象。

播放器正常退出，停止服务成功，前后端进程未中途退出；后端加载 128 个先验关键帧，录制到 638 个前端观测帧及 7567 条后端里程计。录制器观察到 9590 条 IMU、960 帧点云、937 条原里程计和 9604 条真值，数量和时间戳计数均与源包一致，但不能据此证明前端逐条接收。后端日志记录 25 次进入 SUSPECT、未记录 LOST；状态采样估计 SUSPECT 合计约 3.62 秒，末次状态为 NORMAL。日志记录的恢复前后最大平移变化约 0.158 m、最大旋转变化约 21.37°，其中可能包含冻结期间的实际运动，不能直接当作纯定位跳变。[运行结果](data/zlocalization_compare_20261030/src_localization_check_20261031/run_summary.json)、[输入核对](data/zlocalization_compare_20261030/src_localization_check_20261031/input_comparison.json)、[状态时间统计](data/zlocalization_compare_20261030/src_localization_check_20261031/state_duration.json)和[后端日志](data/zlocalization_compare_20261030/src_localization_check_20261031/backend.log)记录了过程。

以前述首次共同样本的平面位置和航向对齐方法与 Gazebo 真值比较，前端 760 个样本的水平 RMSE 为 0.0127 m、最大误差 0.0543 m；后端 757 个样本的水平 RMSE 为 0.0522 m、P95 为 0.0797 m、最大误差为 0.1466 m。后端最大相对航向误差为 29.42°；超过 10° 的 25 个样本按录包时间近似关联到 SUSPECT 且不可信状态，冻结输出期间的运动可能导致此差异。状态与位姿并非严格同步，不能以此作逐帧安全判定。[前端结果](data/zlocalization_compare_20261030/src_localization_check_20261031/frontend_truth_comparison.json)和[后端结果](data/zlocalization_compare_20261030/src_localization_check_20261031/backend_truth_comparison.json)包含方法和数值。

这些数字只描述同包同图、0.5 倍速的单次相对轨迹；首次对齐抹去了初始平面坐标差，不能证明全局地图坐标精度。未验证实时性、独立先验地图、实车标定、状态超时及导航闭环，也不能排除前述其他回放中出现的错误恢复；本次结果不足以切换正式导航链。

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
