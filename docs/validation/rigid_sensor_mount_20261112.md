# 刚性传感器安装与多底盘解耦核验

核验日期：2026-11-12。范围：实车传感器刚性安装配置、静态 TF、已有预处理与底盘适配器的构造和合成运行。用户确认当前安装方式为底盘刚性连接，安装距离待测。本次不修改定位算法、底盘运动学、RMUL 默认参数或导航容差。

## 实现与配置职责

[刚性安装入口](../../xxu_nav_2026/src/xxu_bringup/launch/rigid_sensor.launch.py) 从每种安装的独立文件读取外参，无 EGA 模型或云台状态依赖。静态 `base → sensor → lidar`、点云坐标变换、IMU 旋转及底盘适配器帧名共用该配置；虚拟传感器原点取物理 IMU 原点，避免引入杆臂加速度估计。实现复用 [sensor_launch.py](../../xxu_nav_2026/src/xxu_bringup/xxu_bringup/sensor_launch.py) 及现有预处理节点，没有新 ROS 包。

[待测模板](../../xxu_nav_2026/src/xxu_bringup/config/rigid_mount.pending.yaml) 不填入猜测尺寸，缺少确认或有效数值时在启动节点前拒绝。原云台与仿真函数未改。IMU 节点新增可选的输入帧检查，默认空字符串保持原调用行为；刚性入口显式指定帧并固定零杆臂，不订阅关节。

使用方法及 TF 单发布者、LIO 单位外参、单位换算约束统一见 [LIO 工具说明](../../xxu_nav_2026/tools/lio/README.md#实车刚性安装与多底盘配置)。底盘轮廓、运动约束、车轮模型和通信协议仍属于各底盘，不由传感器安装配置代替。

## 实际验证结果

证据根目录：[rigid_mount_20261112](../../xxu_nav_2026/log/rigid_mount_20261112/)。合成配置中的数值仅为测试夹具，不能作为实车标定。

| 核验对象 | 本次结果 | 证据 |
| --- | --- | --- |
| 刚性安装回归 | 8 项通过；含待测/非法参数拒绝、两底盘配置独立、发布职责切换、底盘适配帧传递、带旋转和平移的点云与速度换算、时间字段保留、IMU 单位/协方差及输入帧检查 | [测试源码](../../xxu_nav_2026/tools/navigation/test_rigid_mount.py)、[测试日志](../../xxu_nav_2026/log/rigid_mount_20261112/rigid_tests.log) |
| 原车型与启动参数 | 12 项通过；未启动导航或仿真 | [日志](../../xxu_nav_2026/log/rigid_mount_20261112/vehicle_tests.log) |
| 既有保护检查 | 10 项通过，1 项硬件可执行文件检查未启用而跳过 | [日志](../../xxu_nav_2026/log/rigid_mount_20261112/review_tests.log) |
| 原接口绑定 | 113 项、0 错误 | [日志](../../xxu_nav_2026/log/rigid_mount_20261112/interfaces.log) |
| `xxu_bringup` 隔离构建 | 单包构建成功，临时 build/install 与日常安装分离；构建只加载 ROS Jazzy 基础环境 | [构建日志](../../xxu_nav_2026/log/rigid_mount_20261112/build/latest_build/xxu_bringup/stdout_stderr.log) |
| 隔离安装入口 | 新入口参数展开成功；默认待测模板返回非零，报外参待测，无节点启动 | [参数展开](../../xxu_nav_2026/log/rigid_mount_20261112/installed_args.log)、[拒绝日志](../../xxu_nav_2026/log/rigid_mount_20261112/pending_rejection.log) |
| 隔离域 194 合成运行 | 启动两个静态 TF 发布者及点云/IMU 预处理；未启动底盘适配、定位、Nav2 或硬件。`base ← lidar` 复合平移 `(0.18,-0.09,0.27)` m 与预期一致，点云与 IMU 输出正确、逐点时间保留、无关节话题，launch 正常退出 0 | [探针](../../xxu_nav_2026/log/rigid_mount_20261112/smoke.py)、[结果](../../xxu_nav_2026/log/rigid_mount_20261112/smoke_result.json)、[运行日志](../../xxu_nav_2026/log/rigid_mount_20261112/runtime.log) |

Python 语法检查及本次改动的空白检查通过。车型检查有上游 `RewrittenYaml` 文件句柄 ResourceWarning，测试返回 0；不视为运行闭环证据。测试进程已退出，未连接真实传感器或电机。

## 尚未验证

真实安装距离、姿态、雷达/IMU 内外参及驱动单位仍待测；尚未验证实车 TF、真实输入时序、ZLIO/TDT 整链、地图兼容、定位失信停车或导航闭环。本次仅证明接口可配置、数学变换和隔离入口合成运行成立。未替换日常安装、启动既有服务、提交、推送或发布。
