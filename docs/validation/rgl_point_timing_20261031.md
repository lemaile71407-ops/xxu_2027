# RGL 逐点时间能力验证

核验日期：2026-10-31。范围为本机 RGL 0.21.0、RGL 最小场景，以及 ROS 2 Jazzy 点云适配节点；并非 Gazebo 整机或导航回归。

## 结论和限制

- 本机 RGL 支持为射线设置毫秒偏移，并可按传感器线速度和角速度模拟运动畸变；测试中的输出逐点时间可取秒或纳秒相对偏移。[官方说明](https://github.com/RobotecAI/RobotecGPULidar)也将该畸变描述为只考虑传感器运动的简化模型，不代表动态物体运动的完整模拟。
- 上游 RGL Gazebo 插件默认不设置射线时间、速度和畸变，也不发布逐点时间。现有 MID-360 二进制扫描文件仅提供射线位姿矩阵，其每条射线的采样时刻与排列顺序尚无可靠依据，不能直接按索引声称真实采样时间。下面的候选接入明确使用均匀时序假设。
- 本次为现有 [点云适配器](../../xxu_nav_2026/src/xxu_livox_sim/src/livox_pointcloud_shaper.cpp) 增加默认关闭的 `input_time_offset_field`。指定字段时，输入必须为相对 `header.stamp` 的 UINT32 纳秒偏移，并设置 `scan_period=0`；输出 `timestamp` 为 FLOAT64 的绝对纳秒。时间字段缺失、类型错误、字段越界或点云行布局不合法会拒绝该帧。未启用时沿用原行为。
- 输入时间必须与实际射线采样位姿匹配，适配器不会生成或验证物理上正确的时间。输出使用现有 FLOAT64 纳秒格式，实际分辨率受数值大小影响。
- 如果将该输出给 ZLIO 的 PointCloud2 解码器，须匹配 `point_time_field=timestamp`、`point_time_scale=1e-9`、`point_time_absolute=true`。本次没有更改 ZLIO 或正式启动配置。
- 候选链路已验证 RGL、Gazebo 到 ROS 的字段桥接和 ZLIO 前端，见下文；尚未完成导航闭环、全局定位后端、动态云台及真实雷达时序验证，不能据此判断实际定位精度改善。

## RGL 最小场景

使用 [GPU 验证程序](data/rgl_timing_capability_20261031/rgl_timing_probe.cpp)：对静态斜墙发射四条方向相同的射线，设置偏移 0、25、50、75 ms，运行两次，断言均通过。

| 场景 | 输出时间偏移 | 沿射线测距 |
| --- | --- | --- |
| 畸变关闭 | 全部为 0 | 全部为 10 m |
| 畸变开启、传感器静止 | 0、25、50、75 ms | 全部为 10 m |
| 畸变开启、沿 x 方向 1 m/s | 0、25、50、75 ms | 10、10.025、10.05、10.075 m |
| 畸变开启、绕 y 轴 1 rad/s | 0、25、50、75 ms | 10、10.2597、10.54、10.8429 m |

在仓库根目录，已构建本机 RGL 插件后可复现：

```bash
c++ -std=c++17 -O2 -Ixxu_nav_2026/build/rgl_plugin/external/RobotecGPULidar/include \
  docs/validation/data/rgl_timing_capability_20261031/rgl_timing_probe.cpp \
  -Lxxu_nav_2026/install/rgl_plugin/RGLServerPlugin \
  -Wl,-rpath,"$PWD/xxu_nav_2026/install/rgl_plugin/RGLServerPlugin" \
  -lRobotecGPULidar -o /tmp/xxu_rgl_timing_probe
/tmp/xxu_rgl_timing_probe
```

## ROS 适配节点验证

使用 [ROS 验证程序](data/rgl_timing_capability_20261031/shaper_time_probe.py)向独立构建的节点发送合成 PointCloud2：验证抽样与 NaN 点过滤后时间仍跟随原点，覆盖大小端及多行填充；缺失字段、错误类型、截断数据和越界字段被拒绝，冲突的时间配置不能启动；未启用逐点时间时，`scan_period=0` 仍接受无时间字段点云。上述断言通过。

```bash
source /opt/ros/jazzy/setup.bash
cd xxu_nav_2026
colcon --log-base /tmp/xxu-rgl-timing-log build --base-paths src/xxu_livox_sim \
  --build-base /tmp/xxu-rgl-timing-build --install-base /tmp/xxu-rgl-timing-install \
  --packages-select xxu_livox_sim
cd ..
ROS_DOMAIN_ID=127 python3 docs/validation/data/rgl_timing_capability_20261031/shaper_time_probe.py \
  /tmp/xxu-rgl-timing-install/xxu_livox_sim/lib/xxu_livox_sim/livox_pointcloud_shaper
```

## RGL 到 ZLIO 的候选接入

同日将候选实现保存为 [上游插件补丁](../../xxu_nav_2026/tools/navigation/rgl_timed_scan.patch)，用 [独立构建脚本](../../xxu_nav_2026/tools/navigation/build_rgl_timed_candidate.sh) 从固定版本生成插件，不覆盖上游仓库或默认 RGL 安装。启动文件增加显式参数 `rgl_point_timing:=uniform`；设置 `enable_zlio_timed:=true`、`enable_lio:=false` 时，还会以相同虚拟传感器坐标系启动 ZLIO 前端，并关闭其 TF 发布。这是一条候选验证链，默认仍为关闭。

逐帧保存上一时刻的雷达位姿，在当前帧计算这段时间的平均传感器局部线速度与角速度。按文件中射线顺序，将整帧时间均匀分配在两个时刻之间；从上一时刻位姿出发用 RGL 速度畸变模型射线追踪，头时间为扫描起点，输出 `offset_time` 为相对起点的 UINT32 纳秒。过滤未命中射线后，每个命中点的时间仍与其射线绑定。下游整形器保留时间，固定雷达的静态坐标转换保留整个点记录。速度坐标系和输出坐标以 [非零位置、90 度旋转的斜墙程序](data/rgl_timing_capability_20261031/rgl_velocity_frame_probe.cpp)及解析结果验证；RGL 0.21.0 的[射线实现](https://github.com/RobotecAI/RobotecGPULidar/blob/a65f07f9565adbfe5cda33a58732638786cb34e8/src/gpu/optixPrograms.cu)也使用局部速度。

这是假设模型，不是从扫描文件恢复的真实 MID-360 时序；它使用两帧之间的近似匀速和本次射线追踪时刻的场景，不能反映扫描期间动态物体的真实运动。时间倒退或两帧间隔超过 0.5 秒时会重新建立起点；候选模式不发布无法用单个时刻准确表达的 `/world` 点云。浮点绝对纳秒的分辨率仍随时间数值变化。

构建并安装当前源码的 `xxu_bringup`、`xxu_livox_sim` 和 ZLIO `frontend`，并加载相应 ROS 工作空间后，在 `xxu_nav_2026` 目录可用以下方式启动 EGA 候选：

```bash
tools/navigation/build_rgl_timed_candidate.sh
# 将输出的 RGL timing candidate 路径赋给 RGL_TIMED_PLUGIN_DIR。
ros2 launch xxu_bringup rm_sim_gazebo.launch.py \
  robot_model:=ega_folded lidar_mode:=rgl rgl_point_timing:=uniform \
  rgl_plugin_dir:="$RGL_TIMED_PLUGIN_DIR" enable_lio:=false enable_zlio_timed:=true
```

上述启动不发布 ZLIO 的 TF，也不启动 `odom → base_footprint` 适配器或全局定位后端；因此不能直接视为完整导航定位。更换车型前须重新验证传感器外参及点云补偿链。

### 候选启动实测

2026-10-31 使用独立 ROS 域、无界面 Gazebo、EGA 固定雷达模型、独立构建的 RGL 补丁和当前源码构建的 ZLIO 前端进行短时测试：

- [点云链路观察](data/rgl_timing_capability_20261031/combined_chain_observation.json)：匹配 60 帧原始、整形和补偿点云；逐帧的射线时间跨度约 99.46–100.00 ms。检查了原始点、抽样后的点及对应时间，以及补偿后保留的时间集合。采集程序保留每个话题最近 60 帧，计数不代表总帧数或丢包率。
- [静止 ZLIO 观察](data/rgl_timing_capability_20261031/combined_zlio_observation.json)：约 25 秒墙钟窗口内收到 1475 条里程计，位置均为有限值，相对首样本的最大三维位移约 3.2 mm；输出帧为 `odom → imu`。
- [直行观察](data/rgl_timing_capability_20261031/combined_straight_observation.json)：命令速度 0.15 m/s，在共同时间窗内 ZLIO 与 Gazebo 真值水平位移分别为 0.8336 m、0.8340 m。
- [旋转观察](data/rgl_timing_capability_20261031/combined_yaw_observation.json)：命令角速度 0.25 rad/s，在共同时间窗内 ZLIO 与 Gazebo 真值的航向变化分别为 1.485151 rad、1.485160 rad。

直行和旋转均在隔离仿真中临时暂停速度安全门并直接发送测试指令，结束后恢复安全门；不构成安全链或导航验证。真值与 ZLIO 通过时间插值计算各自首尾变化，未转换虚拟传感器到 `base_footprint`，因此这些数值不是完整底盘定位误差。单次短程测试也不能证明相对默认方案的性能改善。

直行、旋转两个探针交接时，第二个探针启动日志出现一次 Fast DDS 共享内存端口初始化失败；旋转观察仍收到消息并完成上述断言，传输方式及其丢包影响未进一步核实。
