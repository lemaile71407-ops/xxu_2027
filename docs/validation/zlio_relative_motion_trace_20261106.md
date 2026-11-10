# runtime 248–254 相对运动误差溯源

核验日期：2026-11-06。对象：2026-11-01 `capture03` 原始采集、由它筛选出的 `route_input`，以及 2026-11-03 `freeze_diagnostic01` 诊断回放。范围：原始 ZLIO 发布消息 → 后端输入 → 后端实际捕获位姿，与同源时间仿真真值对照。只读录包及日志，未启动 ROS 节点、定位重放、Gazebo、Nav2 或实车，未修改业务代码、参数及安装。

## 结论

**46.56 cm 相对运动误差已经存在于原始采集的 ZLIO 后验发布消息中，后端输入适配没有产生这段误差。**

- runtime 248–254 对应原始前端 sequence `1257,1262,1267,1272,1277,1282,1287`；逐一用整数纳秒时间戳精确匹配，后端捕获矩阵与原始 `/frontend/frame.odometry` 的最大元素差不超过 **2.22×10⁻¹⁶**。
- 原始采集、筛选输入和诊断回放的五个相关话题，其全程数量及按顺序序列化 SHA-256 一致；误差不是筛包或诊断回放改写消息产生的。
- 同窗口原始高频 `/odometry` 与 `/frontend/odometry.odometry` 的 **751 条**消息，按同一纳秒时间戳核对底层 Odometry 序列化内容，全部存在完全相同的匹配。高频输出独立按片段起点对齐后的末端相对误差为 **48.54 cm**，也已在原始前端输出中出现。
- 底盘适配输出 `/odom` 的 **751 条**消息可由同时间原始高频位姿和录得的固定外参重建，最大矩阵元素差 **4.44×10⁻¹⁶**；该适配输出不进入这里的后端关键帧路径。
- 真值插值括号均为 10 ms；改用两端真值样本的四种起终组合，末端误差仍为 **46.5576–46.6102 cm**。在 ±300 ms、步长 5 ms 的统一时间偏移敏感性检查中，最小末端误差仍为 **46.5059 cm**。普通插值选点或这个范围内的统一常量时间偏移不能解释并消除误差。

**本轮尚未区分 ZLIO 内部估计误差与进入 ZLIO 前的点云/IMU 时间、动态补偿或坐标语义问题。** “已经在前端输出中存在”是本轮确认的故障边界，不是对前端算法或补偿器的最终归因。特别是点级时间、时变延迟和动态杆臂补偿不能被一次统一时间偏移检查排除。后续[同时间预测/后验实验](zlio_propagation_update_attribution_20261106.md)已确认指定窗口新增偏差首先显现在 IMU 传播阶段；速度/偏置、输入补偿及积分根因仍待区分。

后续[输入点时间溯源](zlio_input_point_time_trace_20261106.md)确认原补偿输入为每帧单值时间，当前安装整形器可复现丢弃已知输入偏移。新的[端到端实验](zlio_point_time_e2e_20261106.md)验证有时序输入的交付及同源改善，但原源扫描是否为瞬时扫描、原漂移因果与修复仍未确认。

## 1. 原始资料与链路身份

| 阶段 | 资料入口 | 本轮用途 |
| --- | --- | --- |
| 原始采集 | [capture03](data/rgl_full_20261031/independent_route_check_20261101/capture03/run_summary.json) 与其 `bag/` | 定位后端当时未启动；AMCL 导航下录得原始 ZLIO 发布消息 |
| 后端输入包 | [route_input 来源清单](data/rgl_full_20261031/independent_route_check_20261101/route_input.json) 与相邻 `route_input/` | 原始七话题筛选，没有 epoch/时间截断或序列化改写 |
| 诊断回放 | [freeze_diagnostic01](data/rgl_full_20261031/map_speed_parameters_20261103/freeze_diagnostic01/run_summary.json) 与其 `bag/` | 回放后的实际输入副本 |
| 后端内部捕获 | [原诊断分析](data/rgl_full_20261031/map_speed_parameters_20261103/freeze_retest_20261103/diagnostic_analysis.json) 的 `tracks` | runtime 身份、源时间和后端实际使用的 `odom` 矩阵 |

本轮对五个共同话题重新逐条计算完整序列化摘要：`/frontend/frame`、`/frontend/odometry`、`/odom`、`/tf_static`、`/rm_sim/ground_truth`。三阶段各话题数量分别为 1484、17438、17438、1、20341，逐话题顺序摘要均一致。[本次原始结果](data/backend_factor_graph_review_20261105/relative_motion_trace.json)

这证明记录到的消息交付一致；完整摘要本身不等于证明每条都被后端消费。这里另外用七个内部 runtime 捕获矩阵与原始帧逐一对照，核实本任务指定的实际消费值。

### 两条前端输出必须区分

- `/odometry`：ZLIO 高频预测。前端将同一个 Odometry 对象原样放入 `/frontend/odometry` 身份包装再发布。[node.cpp 的 publish](../../xxu_nav_2026/src/z-localization/src/frontend/apps/node.cpp)
- `/frontend/frame.odometry`：点云观测处理完成后的后验，与点云同封装发布。后端关键帧使用这个位姿。本段七帧都为 `lidar_updated=true`，不是空点云传播片段。独立 `/lidar_odometry` 话题未录入本次原始包；本轮取得的是原始 frame 消息中真实发布的后验，不伪称有该独立话题的证据。[node.cpp 的 collect](../../xxu_nav_2026/src/z-localization/src/frontend/apps/node.cpp)
- `/odom`：导航用底盘适配输出，计算 `T_odom_base=T_odom_sensor T_sensor_base`。它不是后端这里的输入。[适配器](../../xxu_nav_2026/src/xxu_bringup/scripts/zlio_base_odometry.py)

本段原始位姿头为 `odom → imu`，前端 epoch 全为 0、进程身份一致。这里 `imu` 按该实验契约表示已经补偿后的刚性虚拟传感器，底盘适配时对应 `lio_base_sensor`，不是拿原始转动云台 IMU 坐标直接当底盘。前端启动的输入为 `/mid360/livox_points_compensated` 和 `/imu_lio_compensated`，前端 LiDAR 到 body 外参为单位变换。[原采集启动](data/rgl_full_20261031/independent_route_check_20261101/capture_navigation.launch.py)、[原前端配置](data/rgl_full_20261031/normal_pointlio_comparison_20261101/run01/frontend.yaml)

以上说明契约和实际接入，不能据此证明补偿器满足了契约。

## 2. 同一源时间逐项比较

所有后端 runtime 时间均与原始 `/frontend/frame` 的整数纳秒头时间精确匹配，没有用接收时间或近邻帧替代。后端回调对封装里的 Odometry 使用别名指针，`makeFrontendFrame` 直接取该头时间并将 pose 转成矩阵；这条 ZLIO 专用输入路径没有重新把两路话题作近似同步，也没有插入动态 TF 位姿变换。[node_input.cpp](../../xxu_nav_2026/src/z-localization/src/backend/apps/node_input.cpp)、[node.cpp](../../xxu_nav_2026/src/z-localization/src/backend/apps/node.cpp)

仿真真值按源时间做位置线性插值和旋转 SLERP，禁止外推；从真值模型位姿转换到与原诊断一致的虚拟传感器系。录得的 `T_base_footprint_lio_base_sensor` 旋转为单位阵、平移为 `(0,0,0.0697382371)` m。没有把底盘 `/odom` 直接拿来与 `imu` 位姿比较。

每条输出分别在 runtime 248 的同一时间作一次 SE(3) 起点对齐，此后不再拟合。故下表后验和高频输出的误差具有各自的起点对齐；两列不是后验校正量，也不能相减来评价点云更新收益。

| runtime | 前端 sequence | 源时间 s | 原始后验相对水平误差 | 原始高频相对水平误差 |
| --- | --- | --- | ---: | ---: |
| 248 | 1257 | 177.807768442 | 0 | 0 |
| 249 | 1262 | 178.407768442 | 28.557 cm | 24.661 cm |
| 250 | 1267 | 179.007768442 | 41.230 cm | 50.447 cm |
| 251 | 1272 | 179.602125512 | 46.716 cm | 49.690 cm |
| 252 | 1277 | 180.199515141 | 46.402 cm | 47.796 cm |
| 253 | 1282 | 180.799515141 | 46.663 cm | 49.730 cm |
| 254 | 1287 | 181.399515141 | **46.559 cm** | **48.544 cm** |

后验与后端捕获值逐帧差仅为浮点舍入，因此后端的相对误差列与原始后验相同，没有另作一套对齐隐藏差别。高频输出在这些点云结束时间上没有恰好相同的样本，使用两侧 10 ms 高率样本插值；每个括号、确切时间及位姿矩阵均存于结果 JSON。这与后验到后端的精确逐时间戳比对有明确区别。

同时间后验与插值高频位姿的水平差范围约 0.656–9.985 cm。它们是不同处理阶段输出，差别本身不能称为后端适配错误。本轮没有把高频传播状态误认为后端使用的点云后验。

## 3. 时间与运动转换的排除范围

### 能确认的部分

1. 原始前端输出、输入筛包、诊断回放三阶段源时间及封装内容保持不变；本段同一进程、epoch 0，没有重置引起的坐标代次混用。
2. 后端七个关键帧的时间精确对应原始后验，其 pose 矩阵一致；后端侧同步/适配不是这 46.56 cm 的生成点。
3. 原始高频消息与身份包装底层 payload 完全匹配；导航适配器对固定虚拟传感器到 base 的转换也与实际输出吻合。
4. 真值和高频插值无外推。真值括号端点敏感性只改变约 0.05 cm 量级的末端结果，不能解释 46.56 cm。

### 有界时间偏移检查

把真值时间整体移动 −300…+300 ms，步长 5 ms；每个候选偏移都重新使用对应真值起点对齐，再评价七个原始后验。零偏移七点水平 RMSE 为 **0.39993 m**；拟合后最小 RMSE 为 **0.39516 m**（+90 ms），该候选末端仍为 **0.46805 m**。所有候选中最小末端误差为 **0.46506 m**（+25 ms）。

这只是同一故障窗口上的敏感性诊断，没有标定或修改时间。不能把 +90 ms 当成正确偏移；也没有覆盖非恒定延迟、点云起止时间定义、每点时间和 IMU/LiDAR 之间的相对时差。

### 尚未排除的部分

- 前端内部 IMU 高频传播、点云更新、历史状态重传播、速度和偏置估计。
- 进入前端之前的虚拟传感器点云/IMU 补偿、旋转/杆臂加速度处理、实际输入的点级时间语义。
- 虚拟传感器契约是否与当时已安装的实际预处理实现完全一致。当前源码和启动说明不能替代对原包输入的物理核验。

因此下一步最小范围是：保留后端及参数不变，在原始 `capture03` 中锁定同段点云更新和 IMU 输入，比较更新前后状态及真实运动，检查偏差在传播还是点云校正中增长。若要复算原始前端，先核对原二进制、配置和输入身份；这一步本轮未执行。

## 4. 本次执行与证据

分析程序：[trace_relative_motion.py](data/backend_factor_graph_review_20261105/trace_relative_motion.py)。只读上述三包和原诊断分析，结果写入新的 [relative_motion_trace.json](data/backend_factor_graph_review_20261105/relative_motion_trace.json)，没有覆盖旧实验。

复现：从仓库根加载已有隔离依赖环境后运行 `python3 docs/validation/data/backend_factor_graph_review_20261105/trace_relative_motion.py`。无需启动 ROS 节点或安装依赖；依赖 ROS 消息反序列化和已有 NumPy/SciPy。MCAP 提示缺少接收时间消息索引，但读取正常结束；本分析依据源时间而不是接收时间对齐。

本轮两次执行均正常退出；第二次增加了有界时间偏移及插值敏感性复算。相关指纹、精确时间、匹配数量、变换矩阵和数值误差均见结果 JSON；程序和当前相关源码指纹及文档链接核验见[本次核验](data/backend_factor_graph_review_20261105/relative_trace_verification.json)。没有执行修复、安装验证、导航回归或发布。
