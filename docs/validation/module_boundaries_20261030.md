# 传感器、定位与导航启动边界核验（2026-10-30）

## 本次范围

- [传感器启动构造](../../xxu_nav_2026/src/xxu_bringup/xxu_bringup/sensor_launch.py)分为平台预处理和现有 Small Point-LIO 节点；原 `lio_preprocessing` 组合接口保留。Gazebo、rm_sim 和实车入口分别组装两者。
- 实车直接入口增加 `enable_lio` 参数，默认 `true`，显式设为 `false` 时仅关闭 Small Point-LIO；不会因此提供其他定位或 `/odom`。使用入口与前置条件见 [LIO 工具说明](../../xxu_nav_2026/tools/lio/README.md#预处理工具归属)。
- [导航入口](../../xxu_nav_2026/src/xxu_bringup/launch/navigation.launch.py)分别构造地图服务、AMCL/GICP、Nav2 服务器和定位生命周期管理器，再按原顺序组合。底盘与速度保护仍使用既有独立 ROS 包和接口，未更改其实现或默认参数。

未新增 ROS 包或接入 Z-Localization；现有导航入口仍将上述节点放在原有启停组，尚不支持直接选择外部定位。仅检测 `map → odom` 存在不足以证明外部定位可信，后续独立运行需明确定位状态、TF 唯一发布者、断流和失信时的安全行为，并完成相关回归。

## 实际验证

- 以已有 ROS Jazzy 和工作空间安装为依赖，在临时 build/install/log 目录构建 `xxu_bringup`、`xxu_slam_toolbox` 通过；不代表干净设备安装。
- 分别比较实车、旧 Gazebo、rm_sim 上游和 rm_sim CAD 四组构造的预处理和 LIO 节点参数、重映射、条件和顺序，结果与变更前一致。独立构造检查覆盖三个平台不创建估计器，实车新参数默认启用且关闭时只关闭估计器。
- 地图、定位及导航的 15 个 Node 定义和源码顺序的语法树与变更前相同；对 GICP、RViz、fake 开关的 8 组启动描述进行节点顺序、有效参数、重映射和条件对照，一致。构造检查证明 Nav2 服务器列表不包含定位节点；不能将其视为独立闭环导航验证。
- 加载临时安装后，导航、实车 MID360、Gazebo、rm_sim Gazebo、自主探索五个入口的 `--show-args` 通过。车型及构造测试 10 项通过；审查修复检查 11 项中 10 项通过，1 项因未指定重建底盘可执行文件而跳过。接口检查覆盖 156 个绑定，零错误；`git diff --check` 通过。

未启动 Gazebo 或实车，没有执行导航闭环、安装部署或性能验收。日常工作空间安装未更新；使用源码更改需重建并重新加载。
