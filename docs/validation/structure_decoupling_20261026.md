# 结构解耦整改核验（2026-10-26）

范围为接手时 HEAD `21b7837c` 加已有未提交改动之上的 8 项结构整改。未新建 ROS 包，未提交、推送或部署；已有源码收敛、CAD 资产和子模块改动保留。原工作区差异和本次涉及文件的迁移前副本保存在本地 scratch，未随源码发布。

## 最终结构

| 原问题 | 当前实现 |
| --- | --- |
| 描述包承担集成职责 | `xxu_description` 仅保留描述、显示、控制资源；共享 Python、Gazebo/实车 LIO 启动及运行脚本迁入 `xxu_bringup` |
| 三份传感器预处理链 | [launch_helpers.py](../../xxu_nav_2026/src/xxu_bringup/xxu_bringup/launch_helpers.py) 统一构造；[sensor_chain.yaml](../../xxu_nav_2026/src/xxu_bringup/config/sensor_chain.yaml) 保存各平台校准和默认参数 |
| 车型名称分散判断 | 共享 `robot_profile` / `RobotGeometryFile` 按模型目录选择 CAD 固定传感器、四轮独立驱动车型；入口不再判断 `ega_folded` |
| 机器人预处理侵入上游算法包 | 三个脚本归入 `xxu_bringup`，与接手时逐字节一致；Small Point-LIO 不再安装它们 |
| 接口约定仅靠手工同步 | 启动读取 [robot_interfaces.yaml](../../xxu_nav_2026/src/xxu_bringup/config/robot_interfaces.yaml)，[离线检查器](../../xxu_nav_2026/tools/navigation/check_interfaces.py) 检查静态消费者 |
| 直接依赖漏声明 | bringup 显式依赖底盘、Python 安装工具及迁入的运行依赖；SLAM 不再反向依赖描述包 |
| 建图保存路径依赖 cwd | `map_save_path` 显式覆盖优先；默认按 `XXU_MAP_DIR`、可写源码地图链接、用户 ROS 地图目录解析 |
| 仿真重复实现轮系逆解 | CAD 使用 `xxu_chassis_controller`；adapter 仅保留上游车型的速度转发和云台稳定 |

使用说明及参数权威来源见 [导航 README](../../xxu_nav_2026/tools/navigation/README.md#共享启动与参数来源) 和 [LIO README](../../xxu_nav_2026/tools/lio/README.md#预处理工具归属)。两个直接入口改为 `ros2 launch xxu_bringup gazebo.launch.py` 和 `ros2 launch xxu_bringup mid360_lio.launch.py`。

## 静态与构建验证

实际结果及数值证据保存在 [验证数据](structure_decoupling_20261026.json)，不含设备私有路径或原始日志。

- 日常工作空间 16 包以 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` 构建通过；最终 Python/配置调整后的 bringup、SLAM 两包再次构建通过。3 个包仅报告既有 ament 头文件安装目录提示。
- 对 18 组配置递归展开节点包、可执行文件、名字、有效参数、重映射和参数列表：覆盖导航/探索 fake 开关、实车、Gazebo 补偿/原始/里程计诊断、单点、RMUC RGL/GPU、RMUL fake 开关。迁移阶段零非预期差异；最终仅允许节点包名迁移、CAD 的 adapter→底盘替换、上游 adapter 移除空的旧几何参数。桥接端点顺序按集合比较。
- 24 个相关 Python 文件语法检查通过；描述包无本地算法依赖；旧 helper import、旧脚本包引用、车型等值分支和建图 cwd 路径均已移除。清除了此次迁移遗留的 9 个失效安装软链接。
- 接口检查覆盖 12 份静态配置及传感器链、144 个绑定，零错误。把扫描契约误改成已有 `/odom` 的负例会失败。
- 5 项地图路径检查通过：源码软链接、跨 cwd、绝对环境变量覆盖、拒绝相对环境变量、复制安装回退。显式 `map_save_path` 的启动接口保留。
- 1000 组随机速度、四个 CAD 轮子的公式比较最大差值 `8.5265e-14`，小于 `1e-9`。CAD 四轮半径略有差异，因此新增可选 `wheel_radii`，避免单一半径近似。

## 运行验证

底盘隔离域 212：实际编译节点对实车默认参数及 CAD 参数各验证 64 组轮速输出；共同斜坡、零指令立即停车、过期/未来时间戳、接收超时和仿真时钟暂停后的停车均通过。新增参数的默认值保留旧平台行为；CAD 显式保留时间戳年龄和本地接收年龄双重检查。

旧 Gazebo 隔离域 218：IMU 重发布、IMU 补偿、补偿点云、里程计和扫描均收到数据，三项 ros2_control 控制器查询均为 active。第一次使用不同 RMW 的观察器查询服务超时，统一到入口指定的 CycloneDDS 后通过；未因此修改产品代码。该项为冒烟验证，未进行旧平台闭环导航精度验收。

RMUL 隔离域 219、独立 Gazebo partition、无 GUI/RViz：9 个被查询生命周期节点均为 active；原始点云 411 帧，LIO 里程计、去畸变点云及扫描各 388 帧。

| 目标 XY（m） | Nav2 状态 | 本次真实停点误差 | 10-18 历史基线 |
| --- | --- | --- | --- |
| (3, 2) | succeeded | 20.28 cm | 20.64 cm |
| (2, 3) | succeeded | 5.76 cm | 2.96 cm |
| (2, 2) | succeeded | 8.47 cm | 13.96 cm |

原 12 cm XY 容差保持不变，第一个目标仍超限，**导航精度回归未通过**。单次运行不能证明性能改善或确定根因；本次未调导航算法及容差。每个目标完成后安全速度、四轮命令均为零，watchdog 健康；额外静止 2 秒真值位移为 0。观测实时率约 0.886，真值仅供观察器评估，不作为导航输入。

所有本次测试进程已结束。实车验收、整体部署安装验证、发布与推送均不在本次完成范围。折叠车原有 π 朝向容差、自车过滤和 fixed_sensor 行为保留；CAD 物理标定及单雷达停点误差仍待后续专项核验。
