# rm-sim gpu_lidar 与 ZLIO 前端初步验证（2026-10-31）

## 对象与范围

使用 ROS 2 Jazzy、无界面 Gazebo、rm_sim_26 上游哨兵模型及 `lidar_mode:=gpu_lidar`，在隔离 ROS 域和 Gazebo 分区运行；该模型的雷达随大 yaw 云台安装，测试中未主动旋转云台。现有启动代码不支持在 EGA CAD 车型上选择 gpu_lidar，因此这不是同车型的 RGL 对照。`xxu_bringup` 由当前源码构建至临时安装目录，原 Small Point-LIO 和其他依赖沿用已有工作区安装；ZLIO 为 `src/z-localization` 的隔离构建。启动时不运行 Nav2，也不启动 Z-Localization 全局定位后端；ZLIO 的 `publish_tf` 关闭，未将其接入导航 TF。

点云和 IMU 经现有预处理后送入 ZLIO，使用与先前录包回放相同的单位雷达至虚拟传感器外参、绝对纳秒点时间配置。点云、IMU 与 TF 的话题观察、ZLIO 静止观察、短距离运动观察分别进行；ROS 观察器使用 best-effort 订阅，消息数和间隙受订阅发现及调度影响，不能视作精确丢包率。

## 实际结果

- 30 秒话题观察中，收到 286 帧整形点云、283 帧补偿点云、2899 条补偿 IMU 和 288 条旧 LIO 里程计。补偿点云及 IMU 帧名为 `lio_base_sensor`；抽取的各 8 帧整形和补偿点云中，每帧点时间均只有一个值。原始 Gazebo 点云字段不包含逐点时间。启动时出现一次点云等待队列满的告警；该现象的原因和长期运行影响未验证。
- 观察到原链路的 `odom → base_footprint → base_link → lio_base_sensor` 及 `base_link → gimbal_link → livox_lidar` 边；这只证明观察期间边存在，不证明各变换数值正确，也不是 ZLIO 已接入完整 TF。
- 静止场景观察 ZLIO 前端 25 秒，收到 2498 条 `/odometry`，坐标为 `odom → imu`，位置无非有限值，相对首个样本的最大三维位移约 9.35 mm。
- 另一次仿真中发送 0.18 m/s 前进命令约 8 秒，安全输出观测到同等峰值。23 秒观测窗口内，Gazebo 模型真值水平首尾位移约 1.442 m，ZLIO 水平首尾位移约 1.440 m。两路首尾时间未严格对齐，也未将 ZLIO 输出转换为底盘位姿；两数之差不是定位误差或精度指标。

## 限制与证据

此次不覆盖 Z-Localization 后端、真实雷达扫描内逐点时间、动态云台、导航闭环、实车标定或长时间回归；不能据此认定完整定位可用或替换原方案。[话题与 TF 观察](data/rm_sim_gpu_lidar_20261031/observation.json)、[静止观察](data/rm_sim_gpu_lidar_20261031/zlio_observation.json)、[运动观察](data/rm_sim_gpu_lidar_20261031/motion_observation.json)及同目录观察脚本保存了本次结果。原始启动日志保存在本机临时目录，未作为持久证据提交。
