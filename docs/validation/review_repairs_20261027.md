# 审查修复核验（2026-10-27）

范围：既有未提交改动之上的命令保护、静态 LIO 初始化、仿真清理、接口读取边界、CAD 模型契约及导航检查器停车证据修复。原有车型参数和候选成果保留；本次没有拆分 ROS 包、修改车型导航默认参数或提高到点容差。

## 修改与证据入口

- [watchdog](../../xxu_nav_2026/src/fake_vel_transform/scripts/cmd_vel_watchdog.py)：源时间戳与接收年龄双重检查，输出副本保留缓存的原始时间戳。
- [底盘控制器](../../xxu_nav_2026/src/xxu_chassis_controller/src/chassis_controller.cpp)：异常秒/纳秒值在构造 ROS 时间前被拒绝，并使旧命令失效。
- [LIO 前端](../../xxu_nav_2026/src/small_point_lio/src/small_point_lio/small_point_lio.cpp)：静态初始化缓存有界，仅以校准窗及共同传感器时间边界内的点云建图；未来数据保留。回归用例加入[既有前端测试](../../xxu_nav_2026/src/small_point_lio/test/test_dynamic_initialization_frontend.cpp)。
- [进程清理](../../xxu_nav_2026/src/xxu_bringup/xxu_bringup/simulation_processes.py)：按工作空间内的启动目录及入口白名单选择进程树，先 SIGINT，再逐级退出；支持 dry-run 和进程身份复核。实际运行的既有仿真没有被清理。
- [接口模块](../../xxu_nav_2026/src/xxu_bringup/xxu_bringup/interfaces.py)不导入 launch/Nav2；adapter 直接读取接口数据。原 helper 继续导出接口符号。
- [接口检查器](../../xxu_nav_2026/tools/navigation/check_interfaces.py)增加 CAD SDF/URDF 静态消费者；[导航检查器](../../xxu_nav_2026/tools/navigation/check_ega_navigation.py)通过连续新鲜观测判定停车，不再仅根据零指令和零轮速。

## 实际验证结果

| 核验对象 | 范围及实际结果 |
| --- | --- |
| Release 构建 | `small_point_lio`、`xxu_chassis_controller`、`xxu_bringup`、`fake_vel_transform` 四包在临时 build/install/log 目录中构建通过；依赖来自已有 Jazzy 与工作空间 underlay；没有安装新依赖 |
| LIO CTest | 两项测试通过，新增缺失 IMU 缓存上限、运动后静止的旧点排除、共同时间边界与未来数据保留断言；保留原动态初始化及故障/重置测试 |
| [审查修复测试](../../xxu_nav_2026/tools/navigation/test_review_repairs.py) | 10 项测试全部通过，包含实际底盘节点在隔离域 193 对两种时间戳检查配置的异常输入停车、有效命令恢复及接收超时；其他用例覆盖源时间戳不会被转发续期、暂停时钟超时、清理进程归属、接口负例与停车误报 |
| [车型参数测试](../../xxu_nav_2026/tools/navigation/test_vehicle_parameters.py) | 六项通过，未改变已有车型覆盖优先级 |
| 接口检查 | 14 份静态配置/模型加传感器链共 156 项绑定通过；错误轮速/RGL 契约的负例能报错 |
| 安装入口 | 临时安装中的接口模块可导入；实车、LIO、Gazebo、导航、仿真、RMUC、RMUL 共八个 launch 描述构造成功，没有启动这些入口的进程；不是完整节点展开或仿真运行验证 |
| 清理脚本 | Bash 语法检查及 dry-run 通过，未向现有仿真发送信号；选择范围在永久回归中检查，完整退出流程未实测 |

复现入口：加载 ROS Jazzy 和重建后的工作空间，运行 `python3 tools/navigation/check_interfaces.py`、`python3 tools/navigation/test_review_repairs.py`、`python3 tools/navigation/test_vehicle_parameters.py`，并在对应构建目录执行 `ctest --output-on-failure`。实际底盘测试需额外设置 `XXU_REVIEW_CHASSIS_EXECUTABLE` 为本次重建的可执行文件；未设置时明确跳过该项。

## 验证边界

本次未替换日常安装、重启服务、运行 Gazebo 闭环或进行实车验收，未提交、推送或部署。新增 Python 模块需要重建并加载完整安装环境，旧安装不能作为其安装验证来源。导航真值误差和车型参数的最新权威证据仍见 [CAD 运行核验](../../xxu_nav_2026/src/xxu_bringup/models/ega_folded/runtime_validation.json)；本次没有用加强后的停车检查器重跑完整路线，既有 passed 报告不能自动视为通过新增停车条件。实车/仿真依赖拆分尚未实施。

## 自主探索车型参数修复核验（2026-10-28）

范围：[自主探索入口](../../xxu_nav_2026/src/xxu_slam_toolbox/launch/autonomous_mapping.launch.py)声明 `robot_geometry_file`，默认空值，并传入共享 Nav2 参数构造器的 `exploration` 分支。单独启动探索时需要显式选择与仿真相同的几何文件，使用方法见[导航工具说明](../../xxu_nav_2026/tools/navigation/README.md)。

- 新增回归在修复前因入口缺少车型参数声明而失败；修复后[车型参数测试](../../xxu_nav_2026/tools/navigation/test_vehicle_parameters.py)共 7 项通过。
- 新用例构造真实探索 launch 描述，在 fake/base 两种坐标系下展开默认及 EGA 参数，确认局部/全局 costmap 半径均为几何文件中的 0.533 m，停车多边形各边至少包含车体加 3 cm；不指定车型时与既有探索参数一致。
- 接口检查通过：14 份静态配置/模型及传感器链共 156 项绑定，0 错误。
- 执行环境：加载 ROS Jazzy 与现有工作空间，将源码中的 `xxu_bringup` 加入 `PYTHONPATH` 后执行上述两个测试入口；没有重建或替换日常安装，没有启动 ROS 节点、Gazebo 或实车。Nav2 `RewrittenYaml` 报告文件未关闭的 `ResourceWarning`，测试仍通过。

本项验证仅覆盖源码启动参数展开，安装后启动与闭环运行仍未验证。
