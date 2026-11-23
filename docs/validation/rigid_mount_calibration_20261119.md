# 2026-11-19 实车刚性外参标定工具核验

## 范围

在 ROS 2 Jazzy 工作空间内构建和安装 `xxu_bringup`，核验三阶段 C++ 标定库、MCAP/SQLite3 CLI、在线 Trigger 采集、确认门控、独立复核和生产 C++ 预处理/里程计适配。标定节点直接订阅原始 IMU、LiDAR、Odometry，使用 `SensorDataQoS`。在线集成测试在隔离域中确认标定节点没有全局 `/tf` 订阅。

现场操作和参数入口见 [LIO 工具 README](../../xxu_nav_2026/tools/lio/README.md#实车刚性外参标定流程在线与离线录包)。PointCloud2 使用现有 `CloudLayout` 的 FLOAT32 XYZ 布局，保留组织化行跨距和端序语义。Livox CustomMsg 由可选构建依赖启用。当前核验包含已安装 C++ 链路上的合成闭环，以及一个历史 CustomMsg 录包的只读解析。实车三动作数据采集、物理外参和部署后地图定位验收仍待现场完成。

## 数学约定

- `T_A_B` 将 B 系坐标映射到 A 系。固定轴欧拉角为 `Rz(yaw) * Ry(pitch) * Rx(roll)`。在欧拉角奇异位形用零 yaw 表示同一旋转。名义 `T_imu_lidar` 的旋转为单位阵，平移为 `[-0.011, -0.02329, 0.04412]` m。
- 静止加速度均值给出 IMU 的朝上向量。调平点云后优先从有足够支持的近水平平面中选低层地板，PCA 复核法向和厚度。最低平面候选仍须达到最小内点数，并达到最强平面内点数的约七分之一。地板方程 `n_Lᵀ p_L + d_L = 0`，`tz = d_L - base_height_above_ground`。
- 直行对调平姿态做四元数平均，结合水平 PCA 与首尾位移计算 `psi`。 `R_base_imu = Rz(psi) * R_level_imu`。自旋用 `p_odom_imu - R_odom_base * t_base_imu` 恢复中心，以 Huber IRLS 解水平杆臂和中心，并记录条件数、协方差及补偿前后 RMS。有两段运动数据时，迭代三次直行杆臂补偿和自旋回归。
- `base_aligned` 的虚拟传感器保持 IMU 原点并使用底盘轴。 `imu_aligned` 保留物理 IMU 的原点和轴。两种分解对 `T_base_sensor * T_sensor_lidar`、`convert_odometry()` 和后端 `BodyTransform` 的底盘位姿等价。后端 offset 为在底盘轴中表达的 `[-tx, -ty, -tz]`，使用 `R_parent_body = R_parent_sensor * R_body_sensorᵀ` 和 `p_parent_body = p_parent_sensor + R_parent_body * offset`。
- `--verify-mount` 使用加载的固定 `T_base_imu` 和 `T_base_lidar`，直接检查底盘系的重力、地板、直行和自旋。三段均执行并通过才能通过复核。自动分段沿用外参文件中的 IMU 加速度倍率。求解导出将未测量的量保留为 `null`。确认由用户显式触发，并同时要求量齐全且采集阶段通过。

## 构建及自动检查

2026-11-20（北京时间）完成 Codex CLI（`gpt-6.2-sol`，`high`）多轮审查后全部标定问题的修复与主工作区全量重测：

- **在线确认失效、未确认后端隔离与采集时序对齐**：在线节点调用 `start_*`、`stop_*`（含 `capture_stationary` 定时结束）或 `reset` 时自动重置 `confirm:=false`，防止复用旧确认误发布新标定；未确认求解或 `--verify-mount` 失败时不再新建或覆盖 `--output-backend-body`，仅写诊断报告与 `calibration_confirmed: false` 的挂载文件；显式选窗、分录包和自动分段统一按 `header.stamp` 排序去重，自动分段同时按 LiDAR `header.stamp` 与静止 IMU 有效区间交集选窗，避免接收缓冲延迟引入静止窗外点云。
- **多文件切分录包、默认话题隔离与流连续性门控**：多文件 split bag 关闭跨文件提前 `break` 并取非空文件最早起始时间，保留单文件索引包早退；`start_zlio:=true/false` 与离线 CLI 默认里程计话题均隔离为 `/calibration/lidar_odometry`，默认拒绝 `/lidar_odometry` 等共享生产话题（需显式 `allow_unisolated_odom_topic` opt-in），并在直行/旋转阶段检查里程计姿态与静止重力调平一致性（`max_odom_tilt_deg`）；共享静止 IMU 与直行/自旋里程计求解及复核均强制执行连续性门控（`auto_max_sample_gap`）和点云必须在静止 IMU 时间区间内的约束。
- **生产点云验证去 Python 化与驱动点时间核验**：在线闭环对 `/livox/imu_lio_compensated` 和 `/livox/lidar_compensated` 的校验改由原样序列化写 MCAP 后交由 C++ `rigid_mount_calibrate` 求解评估，消除测试脚本中对 `PointCloud2` 字节负载的 Python 遍历；同时代码核实 `livox_ros_driver2`（`pub_handler.cpp:274,388`、`lddc.cpp:239,250,297`）在 `PointCloud2` 中写入的是 UNIX 纪元纳秒，保留 `point_time_absolute:='true'`（仅 `CustomMsg` 减 `base_time`）并补充注释。

构建和测试前清空 `/opt/MVS/lib/64` 所在的 `LD_LIBRARY_PATH`，加载 `/opt/ros/jazzy/setup.bash` 和工作空间 `install/local_setup.bash`，随后加入项目 GTSAM 依赖目录 `z-localization/deps/lib`。Python 回归命令设置了 `ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1`，既有脚本中显式选择的子测试域继续生效。

| 命令 | 退出码 | 实际输出与计数 |
| --- | --- | --- |
| `MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-select xxu_bringup --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release` | 0 | `Finished <<< xxu_bringup [46.0s]`，`Summary: 1 package finished [46.2s]`。colcon 提示所加载工作空间已有该包。CLI 编译启用 `XXU_HAVE_LIVOX_CUSTOM_MSG`。 |
| `ctest --test-dir build/xxu_bringup --output-on-failure` | 0 | `100% tests passed, 0 tests failed out of 3`，总耗时 `0.38 sec`。XML 中标定 29、底盘/传感器桥 10、连续自旋 critic 8，共 47 个 GTest，全通过。 |
| `python3 tools/navigation/test_rigid_mount_calibration.py` | 0 | `Ran 18 tests in 12.180s`，`OK`。 |
| `python3 tools/navigation/test_rigid_mount.py` | 0 | 子进程 `Ran 2 tests in 4.303s`，`OK`。主进程 `Ran 8 tests in 4.686s`，`OK`。 |
| `python3 tools/navigation/check_interfaces.py` | 0 | `106 interface bindings checked in 9 static configs/models and sensor_chain.yaml; 0 errors`。 |
| `python3 tools/navigation/test_vehicle_parameters.py` | 0 | `Ran 15 tests in 0.896s`，`OK`。输出包含 Nav2 YAML 和临时目录 ResourceWarning，以及拒绝 `amcl` 的负对照。 |
| `python3 tools/navigation/test_review_repairs.py` | 0 | 子进程 `Ran 1 test in 1.983s`，`OK`。主进程 `Ran 11 tests in 2.549s`，`OK (skipped=1)`（未构建 `chassis_controller` 跳过该可选项）。 |
| 历史实车录包只读检查 | 0 | `Calibration unconfirmed`。见[历史实车录包只读检查](#历史实车录包只读检查)。 |
| 仓库根目录 `git diff --check` | 0 | 无输出。 |
| `python3 -m py_compile src/xxu_bringup/launch/rigid_mount_calibration.launch.py tools/navigation/test_rigid_mount_calibration.py` | 0 | 无输出。 |

2026-11-19 的启动核验在本地域 198 使用 `driver:=false start_zlio:=true`。ROS 图显示 `/calibration_zlio` 注册了六个隔离的数据话题发布者和隔离的控制服务，标定节点订阅 `/calibration/lidar_odometry`。前端无 `/tf` 或 `/tf_static` 发布者，启动进程收到 SIGINT 后正常退出。

GTest XML 位于工作空间 `build/xxu_bringup/test_results/xxu_bringup/`。早期各轮复现和临时覆盖层验证日志位于 `/tmp/xxu-rigid-calibration-p2-20261120/`、`/tmp/xxu-calibration-round4.5FwddR/`、`/tmp/xxu-calibration-round5.pg3xFd/` 与 `/tmp/xxu-round6-eJGVwh/`。

## 合成数据闭环

| 试验 | 实测结果 |
| --- | --- |
| 合成 MCAP 识别静止、3 m 直行和 360° 自旋 | `base_aligned` 和 `imu_aligned` 均导出确认文件。相对真值的旋转误差按输出精度均为 `0.000000 deg`，平移误差 `0.0267 mm`。生产 YAML 加载器接受两个模式。两个模式的独立验证均通过，水平杆臂加 0.12 m 的篡改均被拒绝。 |
| 在线 Trigger → 导出 → 已安装的 `rigid_sensor.launch.py` | 按先 `confirm:=false` 预览（无 `backend_body.yaml`）、再设 `confirm:=true` 导出、触发 `start_spin`/`stop_spin`/`reset` 后验证 `confirm` 自动复位且已有 `backend_body.yaml` 原样保留。生产闭环收到 121 条 `child_frame_id=base_link` 底盘里程计，重力误差 `0.000000 m/s²`，自旋中心 XY RMS `0.0000 mm`；捕获 `/livox/imu_lio_compensated` 与 `/livox/lidar_compensated` 写入 MCAP 经 C++ 拟合地板高度误差 `0.0221 mm`（RMS < 5 mm）。 |
| SQLite3 静止单段 | 只求得调平、倍率和高度，未测 yaw/XY 导出为 `null` 且不创建 `backend_body.yaml`，生产 YAML 加载器拒绝。传 `--confirm` 返回 1，仍不写后端文件并写未确认报告。指定缺失 IMU 话题返回 1 并写输入错误报告。 |
| SQLite3 静止单段复核偏航误差 90° 的外参 | 静止检查通过。CLI 输出 `Verification failed`，退出 1，报告同时列出直行和自旋缺段。传 `--confirm` 后，报告 `verification_passed: false`、导出外参 `calibration_confirmed: false`，已有 `backend_body.yaml` 逐字节保留且不创建新后端文件。 |
| milli-g MCAP，静止 3 s 后立即移动，里程计滞后 250 ms | 静止接收窗从 0 s 开始，结束时间早于 3 s，持续至少 2 s。直行、自旋接收窗分别从 3.25 s 和 7.25 s 开始。`--imu-scale 0.00981 --confirm` 求解成功，复核沿用外参中的倍率，输出 `Verification passed`，退出 0。 |
| `0.18 m/s` 直行 MCAP，里程计交替增加 `60 ms` 接收延迟 | `100 ms` 自动分段及 `40 ms` 采样的自动分段、显式选窗、分阶段三包求解均以 `--confirm` 退出 0，直行距离超过 2.95 m，里程计拒收为零。四组旋转误差均为 `0.000000 deg`，平移误差均为 `0.0267 mm`。 |
| 独立延迟 LiDAR（`lidar_lag=3.5 s` 及 `1.0 s + 0.6 m` 错层干扰） | 自动分段均正确选中全部 31 帧静止点云（头时间戳 `[t0, t0+3]`）、剔除重叠到达的运动点云，`--confirm` 与 `--verify-mount --auto-segment` 均准确通过且拒收为零。 |
| 静止 IMU 区间外点云与 2 s IMU 断流 | 接收窗内滞留的前期点云被拒绝（`clouds: 0`，`tz: null`）；2 s IMU 断流（区间内含错层点云）在求解与复核中均因样本间隔超限退出 1，不写 `backend_body.yaml`。 |
| 运动里程计 3.8 s 断流与伪自旋 | 两个 10° 旋转片段隔 3.8 s 断流被直行/旋转连续性检查拒绝（`max_sample_gap_sec > 0.5`），求解与复核均退出 1，不写 `backend_body.yaml`。 |
| 共享生产话题隔离与补偿后倾角保护 | 未显式 opt-in 时默认及显式使用 `/lidar_odometry`、`start_zlio:=false` 均拒绝或默认落回 `/calibration/lidar_odometry`；显式 `--allow-unisolated-odom-topic` 后原始物理里程计可确认，而生产补偿轴向（水平化）里程计因重力倾角不一致被拒绝。 |
| 切分多文件 MCAP 与无索引 Fastwrite MCAP | 文件间接收时间重叠的双文件切分 MCAP 在部分窗与自动分段下均完整保留 501 条 IMU，零拒收且外参准确；半秒反序无索引 Fastwrite MCAP 与有序对照一致，三流各拒收一条重复记录。 |
| 保留 TF 帧与物理 IMU 里程计帧 | GTest 与 CLI 验证拒绝 `map`、`odom`、`base_footprint` 保留帧及错误/不匹配的里程计子帧（如 `base_footprint`）。 |
| PointCloud2 启动参数与导出写入失败回滚 | 集成测试确认 `offset_time`、`1e-6` 和 `false` 传入 `lio.launch.py`；挂载、后端或报告目录缺失时退出 1，报告与既有目标文件保持安全状态，无残留临时文件。 |

## 历史实车录包只读检查

```bash
ros2 run xxu_bringup rigid_mount_calibrate \
  --bag bags/mid360s_corridor_test_20261115_1900 \
  --stationary-window 0:3 --lidar-topic /livox/lidar --imu-topic /livox/imu \
  --output-report /tmp/real_bag_smoke_report.yaml
```

2026-11-20 重测以录包绝对路径运行上面的命令，退出码为 0，输出如下。

```text
[WARN] [1795106040.425944620] [rosbag2_storage_mcap]: attempted to read in receive timestamp order with no message index
[WARN] [1795106040.425973030] [rosbag2_storage_mcap]: attempted to read in receive timestamp order with no message index
stationary: gyroscope indicates motion or unstable samples | gravity leveling is unmeasured | psi is unmeasured | tx is unmeasured | ty is unmeasured | tz is unmeasured
Calibration unconfirmed
```

CLI 按文件顺序读取无索引 MCAP，并按接收时间筛选。`/tmp/real_bag_smoke_report.yaml` 记录 600 条 IMU、30 帧 CustomMsg，两个输入均观测到 `livox_frame`，拒收计数为零。原始加速度均值模长为 `0.9969686975689066`，识别倍率为 `9.81`。

该窗口陀螺最大偏差为 `0.08020935591649202 rad/s`，超过默认 `0.08 rad/s`，因此静止门控失败，报告标记地板拟合未执行及 `calibration_confirmed: false`。本次历史录包核验的证据范围是输入解析、识别单位和拒绝不合格窗口。真实安装外参及独立实车闭环仍待验证。
