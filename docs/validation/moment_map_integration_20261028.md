# 累计平面地图接入核验（2026-10-28）

## 完成范围

将文章对应仓库 [mozihe/z-localization](https://github.com/mozihe/z-localization) 的实际地图源码接到 Small Point-LIO。
固定上游提交 `76e126328b670adc92981d9b4900cc059a6f2343`，最小源码子集逐文件未修改，保留原 GPL-2.0 许可证和来源清单。
原 Small Point-LIO 文件的 MIT 版权声明保留；可选组合构建包含上述 GPL 组件。

新增 `SMALL_POINT_LIO_ZLIO_MAP` 构建选项和 `map_backend: moment` 运行参数。默认构建选项关闭、默认地图仍为 `ivox`，没有修改正式平台 YAML 或日常安装。
[候选覆盖文件](../../xxu_nav_2026/src/small_point_lio/config/moment_map.candidate.yaml)可在完整平台参数之后叠加，具体用法见[包 README](../../xxu_nav_2026/src/small_point_lio/README.md#累计平面地图候选)。

当前交付是已接入、可显式选择的地图候选，不等于实机验收通过或已切换正式导航默认值。此前[地图微基准](voxel_map_comparison_20261028.md)与本次接入测试分开保存。

## 适配内容

- 保留上游哈希根体素、八叉树、累计点矩与噪声矩、概率平面匹配、冻结和资源限制。
- 静止及运动初始化累积支持点后批量建立初始地图；正常阶段支持即时插入及按扫描延迟插入。
  延迟支持点在采集时保存协方差，扫描提交时不使用后续状态覆盖；重置销毁地图和未提交点。
- 本包误差坐标为 `[delta_p_world, delta_theta_body_right]`，上游点 Jacobian 使用 `[delta_t_world_left, delta_theta_world_left]`。
  设当前状态为位置 p、旋转 R，变换为 `delta_t_left = delta_p + [p]x R delta_theta_right`、`delta_theta_left = R delta_theta_right`。
  适配器完整变换 6×6 协方差及交叉项；数值测试验证远离世界原点时，两种表达产生相同点边缘协方差。
- 查询使用激光点噪声加当前预测位姿传播的点边缘协方差；ESKF 观测方差使用激光点方差、地图平面方差和表面散布，避免将当前状态先验再次当作观测噪声。
- 仍使用原 30 维 ESKF。当前只支持固定外参，`extrinsic_est_en=true` 与候选组合会明确拒绝；非有限或非正 `laser_point_cov` 同样拒绝。
- 入图沿用现有各向同性 `laser_point_cov`，未移植完整雷达射线噪声标定。初始化窗口支持点共享窗口末状态的刚性位姿先验，未恢复每帧独立初始化协方差。

接口见 [MomentMap](../../xxu_nav_2026/src/small_point_lio/src/small_point_lio/moment_map.h) 和 [Estimator](../../xxu_nav_2026/src/small_point_lio/src/small_point_lio/estimator.cpp)。

## 当前构建、安装及回归证据

在仓库外的独立 build/install 目录分别开启和关闭可选组件，Release 构建均通过，无编译警告。没有替换工作空间日常安装或安装新依赖。

- 组件开启：CTest 3/3 通过，包括已有运动求解器与前端回归，以及新增真实地图桥接、协方差坐标转换、延迟插入、激光观测生成、静止/运动初始化交接与重置。
- 组件关闭：CTest 3/3 通过；新增测试验证请求未编译的地图会报错，默认共享库没有 `zlio::VoxelMap` 导出符号。
- 临时开启组件的安装目录包含许可证、上游提交/指纹清单以及候选 YAML；未启动实际里程计节点或验证实机驱动安装。
- 临时环境未找到 Livox 驱动包，因此未编译 Livox 自定义消息入口。本次是直接回调与解析传感器输入测试，不代替实际雷达输入链验证。

[当前构建/CTest 输出](data/moment_map_integration_20261028.tests.txt)；[源码、二进制、结果及环境元数据](data/moment_map_integration_20261028.metadata.json)。

## 同输入前端对照

三场景 × 两种初始化选择（原静止 / 自动）× 三种子 × 两种地图，共 36 次，每次 6 秒、墙钟 1 倍速。
数据生成与统计沿用[自动初始化报告](automatic_initialization_20261027.md#数据与指标)：IMU 200 Hz、点云 10 Hz、三平面 243 点/帧，固定噪声和零偏。
两种地图除后端及各自根体素尺寸外使用相同输入和滤波参数；平面约束、空间下采样、按扫描延迟插入均关闭。延迟插入另由前端测试覆盖。

下表为自动初始化分支，三次中位数；位置误差为初始化源时间 +2 秒，经首帧 yaw 和平移对齐，不消除倾角误差。CPU 为每次 6 秒回放累计进程 CPU 秒数，含初始化后台任务。

| 场景 | 地图 | 出位姿 | 启动时间 s | 2 s 位置误差 mm | CPU s |
| --- | --- | --- | --- | --- | --- |
| 静止 | SmallIVox | 3/3 | 1.000 | 3.054 | 0.060 |
| 静止 | 累计平面 | 3/3 | 1.000 | 3.953 | 0.068 |
| 持续 yaw | SmallIVox | 3/3 | 2.195 | 0.847 | 0.145 |
| 持续 yaw | 累计平面 | 3/3 | 2.200 | 0.743 | 0.154 |
| 平移并三轴旋转 | SmallIVox | 3/3 | 2.245 | 2.043 | 0.190 |
| 平移并三轴旋转 | 累计平面 | 3/3 | 2.245 | 1.881 | 0.198 |

36 次均无初始化后的 IMU 时间故障或无效回调状态。原静止初始化在静止场景两种地图均 3/3 输出，在两个持续运动场景均 0/3，保留原有等待行为。
静止场景原静止初始化的两秒位置误差为 SmallIVox 2.664 mm、累计地图 4.119 mm；自动静止的结果见表。
候选在两个运动合成场景略低、静止略高，没有证据证明全面改善。没有据此修改正式默认地图。

逐次原始数据：
- [moment_map_frontend_ivox_moving_room_20261028.csv](data/moment_map_frontend_ivox_moving_room_20261028.csv)
- [moment_map_frontend_ivox_static_room_20261028.csv](data/moment_map_frontend_ivox_static_room_20261028.csv)
- [moment_map_frontend_ivox_yaw_room_20261028.csv](data/moment_map_frontend_ivox_yaw_room_20261028.csv)
- [moment_map_frontend_moment_moving_room_20261028.csv](data/moment_map_frontend_moment_moving_room_20261028.csv)
- [moment_map_frontend_moment_static_room_20261028.csv](data/moment_map_frontend_moment_static_room_20261028.csv)
- [moment_map_frontend_moment_yaw_room_20261028.csv](data/moment_map_frontend_moment_yaw_room_20261028.csv)

复现对照使用候选构建后的 `benchmark_initialization`：

```bash
OMP_NUM_THREADS=1 ROS_DOMAIN_ID=194 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  build/small_point_lio/benchmark_initialization --map-backend moment \
  --scenario moving_room --repeats 3 --duration 6 --csv /tmp/moment_moving.csv
```

改为 `--map-backend ivox` 可运行基线，其他场景为 `static_room`、`yaw_room`；每个 CSV 内仍分别记录静止和自动初始化方式。

## 未完成验证

没有真实录包、实机连接、完整导航或长时间闭环结果。原生平面判据、最大深度和冻结上限保持上游值，细密台阶、动态物体及早期拟合错误仍有此前微基准暴露的限制。
根上限固定为 100 万，达到时上游抛出资源限制异常，本次没有添加局部地图淘汰或自动回退。长期资源占用与真实定位表现尚待验证。
