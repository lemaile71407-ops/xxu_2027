# XXU RM 26

XXU 全向机器人 ROS 2 Jazzy + Gazebo Harmonic 仿真、定位、导航与底盘控制工程。

ROS 工作空间位于 `xxu_nav_2026/`。仓库根目录用于项目说明和版本管理，工作空间内的
`src/` 存放 ROS 包，`build/`、`install/`、`log/` 等目录是构建或运行产物。

## 文档入口

本文保存项目用途和基本使用方式，专项说明见下方入口。当前实验数据目录已不存在，部分历史报告仍保留；
当前导航主线与收敛范围见[导航说明](xxu_nav_2026/tools/navigation/README.md#当前主线与收敛范围)。构建产物、运行日志和用户原始 CAD 输入不随源码发布。

相关专项文档：

- [小陀螺导航与回归验证](xxu_nav_2026/tools/navigation/README.md)
- [LIO 调试与回归验证](xxu_nav_2026/tools/lio/README.md)
- [ZLIO 输入与集成](xxu_nav_2026/tools/lio/README.md)
- [点云转 LaserScan](xxu_nav_2026/src/pointcloud_to_laserscan/README.md)

## 快速开始

### 1. 构建工作空间

工作区 `src` 中包含 Z-Localization 候选源码，全量构建前还需准备 GTSAM 4.2 和 Livox SDK2；本机依赖位置及迁移范围见 [源码迁移记录](docs/validation/zlocalization_src_migration_20261031.md)。

```bash
cd ~/xxu_2026/xxu_nav_2026
source /opt/ros/jazzy/setup.zsh
git -C .. submodule update --init --recursive
vcs import src < simulation.repos
colcon build --symlink-install
source install/setup.zsh
```

若仅构建实车侧导航与硬件驱动（不拉取/构建仿真模型与 Gazebo 依赖），可忽略仿真包或仅构建目标包：

```bash
colcon build --symlink-install --packages-up-to xxu_hardware xxu_slam_toolbox pb_nav2_plugins pb_omni_pid_pursuit_controller pointcloud_to_laserscan frontend backend
```

### 2. 启动 RMUL 导航

当前只保留 **ZLIO → TDT → 底盘里程计适配 → Nav2 MPPI Omni** 主线，使用固定折叠 EGA 与单台 MID360。Small Point-LIO 及其旧 XXU 仿真、实车整套启动链已撤除。传感器预处理和底盘接口继续保留；实车刚性传感器可使用[独立安装配置](xxu_nav_2026/tools/lio/README.md#实车刚性安装与多底盘配置)，安装距离待测，实车 ZLIO 整套接入尚待验证。

在已构建的 `xxu_nav_2026` 目录执行统一入口（只启动系统，不发导航目标）：

```bash
./tools/navigation/start_rmul.sh --profile tested_sim
```

该配置显式选择旋转实测新图、区间平均 IMU、实验后端和快速 MPPI。`--profile baseline` 用于默认算法对照，`--headless` 关闭界面。两者均适用于已知出生点 `(2,2,0)`，需保留地图数据及已构建的逐点时间插件。安装与验证范围、已知失败统一见[导航主线](xxu_nav_2026/tools/navigation/README.md#当前主线与收敛范围)和[阶段三报告](docs/validation/nav2_stage3_20261108.md)；实验组合仍未完成全航向和长路线验收。

### 3. 启动职责与检查

- `xxu_sim_bringup/rmul_navigation.launch.py`：组合传感器仿真、唯一 ZLIO/TDT、底盘适配与 Nav2；`start_navigation:=false` 可关闭 Nav2，继续启动定位。
- `xxu_sim_bringup/rm_sim_gazebo.launch.py`：仅提供仿真、传感器预处理和底盘，不包含 LIO；不应单独用于导航验收。
- `xxu_hardware/chassis_lower_bridge.launch.py`：实车 STM32 串口下位机通信桥（`/dev/ttyACM0`，不依赖仿真）。
- `xxu_bringup/zlio_base_adapter.launch.py`：提供 ZLIO 到底盘的里程计、TF 和派生点云转换。
- `xxu_bringup/navigation.launch.py`：保留通用 Nav2 构造；主线由 RMUL 入口固定选择外部定位模式。

RViz 使用 `map` 为 Fixed Frame。导航健康条件包括定位状态、位姿和 TF 持续新鲜；地图显示正常不等于定位或导航验收完成。

## 重要启动参数

| 参数 | 默认值或要求 | 说明 |
| --- | --- | --- |
| `localization_backend` | `zlio_tdt`，仅此选项 | 保留旧调用参数名，拒绝旧 AMCL 分支 |
| `start_navigation` | `true` | 是否启动 Nav2 |
| `enable_localization` | 必须为 `true` | 启用 ZLIO/TDT 定位 |
| `frontend_config` / `backend_config` | 显式提供文件 | 前后端配置 |
| `prior_map` / `external_map` | 显式提供 | 匹配的先验地图和 Nav2 地图 |
| `external_nav2_params` | 显式提供文件 | 当前联调参数 |
| `rviz` / `gazebo_gui` | `true` | 可视化开关 |

基础导航参数与共享启动配置职责见[导航说明](xxu_nav_2026/tools/navigation/README.md#共享启动与参数来源)。RMUL 主线固定使用真实底盘坐标，不启用虚拟参考帧与自动叠加自旋（显式 `--spin` 试验模式由 MPPI 自身预测并输出角速度）。

## 速度与底盘链路

导航命令从 Nav2 到安全门控及底盘的链路为：

```text
/cmd_vel_nav
  -> /cmd_vel_smoothed
  -> /cmd_vel_collision
  -> cmd_vel_watchdog（xxu_bringup）
  -> /cmd_vel（默认底盘输入）或 /rm_sim/cmd_vel_safe（rm_sim 仿真安全输出）
  -> /wheel_velocity_controller/commands / 下位机桥
```

其中：

- `cmd_vel_watchdog`（位于 `xxu_bringup`）检查命令、里程计、激光、关节状态、关键 TF 与定位健康状态；任一必需输入超时或定位失信就发布零速。
- `xxu_chassis_controller` 把 `TwistStamped` 转成四个全向轮的角速度指令；实车部署时由 `xxu_hardware` 串口桥接下发至 STM32 下位机。
- 底盘控制器默认命令超时为 `0.3 s`，最大轮速参数为 `100 rad/s`。

2026-10-17 的清理仅代表当时范围。后续新增的 ZLIO/TDT 适配与接口检查仍在源码中，当前保留范围以[导航说明](xxu_nav_2026/tools/navigation/README.md#当前主线与收敛范围)为准。

## 建图与探索

保留 [xxu_slam_toolbox](xxu_nav_2026/src/xxu_slam_toolbox) 的建图与探索工具。运行前必须提供匹配的 ZLIO 底盘里程计、TF 与激光输入；旧 Small Point-LIO 仿真组合已撤除。新组合的建图与探索尚未在本轮验收，不能沿用旧启动成绩。

传感器预处理和 ZLIO 输入说明见 [LIO 工具](xxu_nav_2026/tools/lio/README.md)。

## 源码结构

```text
xxu_nav_2026/src/
├── xxu_bringup/                    # 共享导航核心：传感器预处理、通用启动、Nav2/地图与接口契约（零仿真/串口依赖）
├── xxu_hardware/                   # 实车硬件驱动与 STM32 下位机串口通信桥（依赖 xxu_bringup，零仿真依赖）
├── xxu_sim_bringup/                # Gazebo 仿真启动、适配器、CAD/场地模型资源及仿真清理脚本
├── xxu_description/                # URDF/Xacro、网格、世界、显示与 ros2_control 配置
├── xxu_chassis_controller/         # 全向底盘 Twist 到四轮角速度转换
├── xxu_livox_sim/                  # Livox Mid-360 激光雷达仿真
├── z-localization/                # 唯一 ZLIO 前端、TDT 后端及统一定位启动
├── xxu_slam_toolbox/               # SLAM 和自主探索入口
├── pointcloud_to_laserscan/        # PointCloud2 转 LaserScan
├── pb_nav2_plugins/                # 自定义 Nav2 后退行为
├── pb_omni_pid_pursuit_controller/ # 备用 PID 控制器与目标接近包装插件
└── m-explore-ros2/                 # explore_lite / map_merge 子模块
```

## 自定义 Nav2 插件

`pb_nav2_plugins` 当前包含：

- `pb_nav2_behaviors::BackUpFreeSpace`：根据局部 costmap 选择较宽的自由空间执行备份。

相关配置：

```text
xxu_nav_2026/src/xxu_bringup/config/nav2_navigation.yaml
xxu_nav_2026/src/xxu_bringup/config/nav2_exploration.yaml
```

## 常用排查

检查 ROS 包是否可见：

```bash
ros2 pkg prefix xxu_bringup
ros2 pkg prefix frontend
ros2 pkg prefix backend
ros2 pkg prefix xxu_chassis_controller
```

检查关键节点和话题：

```bash
ros2 node list
ros2 topic list | rg '/map|/scan|/odom|cmd_vel|wheel_velocity'
ros2 topic info /map -v
ros2 run tf2_tools view_frames
```

如果 Gazebo 或 RViz 窗口消失，先确认启动终端没有被 `Ctrl-C` 终止；重新启动前运行：

```bash
src/xxu_sim_bringup/scripts/kill_simulation.sh
```

可先加 `--dry-run` 查看目标 PID。清理仅选择启动工作目录位于本工作空间内的 XXU 仿真/建图白名单启动树，先请求正常退出以便保存地图，再对未退出进程逐级结束；实车入口和其他工作空间不在选择范围内。从其他目录启动或已脱离启动树的进程需在原终端退出。
