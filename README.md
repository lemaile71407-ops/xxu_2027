# XXU RM 26

XXU 全向机器人 ROS 2 Jazzy + Gazebo Harmonic 仿真、定位、导航与底盘控制工程。

ROS 工作空间位于 `xxu_nav_2026/`：`src/` 存放 ROS 包，`build/`、`install/`、`log/` 为构建与运行产物。

## 文档入口

- [项目上下文索引](PROJECT_INDEX.md)：全局文档路由、当前状态与验证报告入口
- [导航工具说明](xxu_nav_2026/tools/navigation/README.md)：导航主线、共享启动与参数来源、坐标链与速度保护
- [LIO 工具说明](xxu_nav_2026/tools/lio/README.md)：ZLIO 输入预处理、实车刚性安装外参配置与录包排查
- [点云转 LaserScan](xxu_nav_2026/src/pointcloud_to_laserscan/README.md)：`pointcloud_to_laserscan` 节点说明

## 快速开始

### 构建工作空间

全量构建前需先准备 `GTSAM 4.2` 与 `Livox SDK2`（本机依赖位置及迁移范围见[源码迁移记录](docs/validation/zlocalization_src_migration_20261031.md)）：

```bash
cd ~/xxu_2026/xxu_nav_2026
source /opt/ros/jazzy/setup.zsh
git -C .. submodule update --init --recursive
vcs import src < simulation.repos
colcon build --symlink-install
source install/setup.zsh
```

若仅构建实车侧导航与硬件驱动（不拉取或构建 Gazebo 仿真包）：

```bash
colcon build --symlink-install --packages-up-to xxu_hardware xxu_slam_toolbox pb_nav2_plugins pb_omni_pid_pursuit_controller pointcloud_to_laserscan frontend backend
```

## 启动入口

当前主线为 **ZLIO → TDT → 底盘里程计适配 → Nav2 MPPI Omni**（固定折叠 EGA + 单台 MID360），详细收敛范围、参数来源与已知限制见[导航主线与收敛范围](xxu_nav_2026/tools/navigation/README.md#当前主线与收敛范围)及[阶段三报告](docs/validation/nav2_stage3_20261108.md)。

在 `xxu_nav_2026/` 目录执行 RMUL 仿真导航统一入口（仅启动系统，不自动下发目标；适用于已知出生点 `(2, 2, 0)`）：

```bash
./tools/navigation/start_rmul.sh --profile tested_sim
```

常用选项：`--profile baseline`（默认算法对照）、`--headless`（关闭界面）、`--spin`（试验边移动边自旋）。

主要独立启动与职责：

- `xxu_sim_bringup/rmul_navigation.launch.py`：组合传感器仿真、ZLIO/TDT、底盘适配与 Nav2（`start_navigation:=false` 可仅启动定位）
- `xxu_sim_bringup/rm_sim_gazebo.launch.py`：仅启动仿真、传感器预处理与底盘（不含 LIO）
- `xxu_hardware/chassis_lower_bridge.launch.py`：实车 STM32 串口下位机通信桥（`/dev/ttyACM0`，不依赖仿真）
- `xxu_bringup/zlio_base_adapter.launch.py`：ZLIO 到底盘的里程计、TF 与派生点云转换
- `xxu_bringup/navigation.launch.py`：通用 Nav2 构造（外部定位模式）

实车刚性安装多底盘外参入口见[刚性安装说明](xxu_nav_2026/tools/lio/README.md#实车刚性安装与多底盘配置)；坐标链与 `cmd_vel_watchdog` 速度保护见[坐标链与速度保护](xxu_nav_2026/tools/navigation/README.md#坐标链与速度保护)。RViz 使用 `map` 为 Fixed Frame，导航健康运行需定位状态、位姿与 TF 持续新鲜。

## 建图与探索

保留 [xxu_slam_toolbox](xxu_nav_2026/src/xxu_slam_toolbox) 的建图与自主探索入口，运行前需提供匹配的 ZLIO 底盘里程计、TF 与激光输入（详见 [LIO 工具说明](xxu_nav_2026/tools/lio/README.md)）。新组合的建图与探索尚未完成本轮验收。

## 源码结构

```text
xxu_nav_2026/src/
├── xxu_bringup/                    # 共享导航核心：传感器预处理、通用启动、Nav2/地图与接口契约（零仿真/串口依赖）
├── xxu_hardware/                   # 实车硬件驱动与 STM32 下位机串口通信桥（依赖 xxu_bringup，零仿真依赖）
├── xxu_sim_bringup/                # Gazebo 仿真启动、适配器、CAD/场地模型资源及仿真清理脚本
├── xxu_description/                # URDF/Xacro、网格、世界、显示与 ros2_control 配置
├── xxu_chassis_controller/         # 全向底盘 Twist 到四轮角速度转换
├── xxu_livox_sim/                  # Livox Mid-360 激光雷达仿真
├── z-localization/                # ZLIO 前端、TDT 后端及统一定位启动
├── xxu_slam_toolbox/               # SLAM 与自主探索入口
├── pointcloud_to_laserscan/        # PointCloud2 转 LaserScan
├── pb_nav2_plugins/                # 自定义 Nav2 后退行为（BackUpFreeSpace）
├── pb_omni_pid_pursuit_controller/ # 备用 PID 控制器与目标接近包装插件
└── m-explore-ros2/                 # explore_lite / map_merge 子模块
```

## 常用排查

```bash
# 检查包与运行状态
ros2 pkg prefix xxu_bringup
ros2 node list
ros2 topic list | grep -E '/map|/scan|/odom|cmd_vel|wheel_velocity'
ros2 run tf2_tools view_frames

# 清理本工作空间内的仿真/建图残留进程（可先加 --dry-run 预览）
src/xxu_sim_bringup/scripts/kill_simulation.sh --dry-run
```
