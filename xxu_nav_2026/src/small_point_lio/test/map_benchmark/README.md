# 地图独立对照

此目录仅包含手动测试程序，不参与 ROS 包构建、安装或正式定位。SmallIVox 使用当前包的原始实现；`pch.h` 仅提供它实际用到的头文件依赖。此独立对照程序直接编译外部 z-localization 的源码；后来接入候选地图的上游子集另存于 [3rdparty/zlio_map](../../3rdparty/zlio_map/README.md)。

需要系统已经具备 g++、Eigen3、gtest 和 Python 3；脚本不安装依赖。准备仓库外的上游检出目录，固定到报告中的提交，再从本仓库根目录执行：

```bash
python3 xxu_nav_2026/src/small_point_lio/test/map_benchmark/run_benchmark.py \
  --upstream /tmp/z-localization \
  --build-dir /tmp/xxu_map_probe \
  --output /tmp/xxu_map_probe.csv
```

脚本先构建并运行上游地图测试，再启动 108 个独立对照进程。输出 CSV、同名 `.metadata.json` 和构建目录中的 `upstream_map_test.log`。
上游采用 GPL-2.0；实验二进制仅在仓库外生成，不安装进本包。

输入为有解析平面真值的合成点云。每组含三批独立空间/噪声采样，输入全部预先分配；同种子下三种地图方案收到完全相同的输入。固定查询集不参与入图。
堆内存为 `mallinfo2` 活跃分配量增量，包含节点与容器开销；它不是地图的序列化大小或完整 ROS 进程内存。匹配沿用各方案自身门限，另用解析真值核验法向及残差，避免只以匹配率判断质量。

已执行范围、门限、原始数据和限制见[2026-10-28 对照报告](../../../../../docs/validation/voxel_map_comparison_20261028.md)。

## Gazebo 真值对照

[Gazebo 对照报告](../../../../../docs/validation/moment_map_gazebo_20261028.md)记录隔离运行条件与原始数据。
`gazebo_warmup.patch` 用于检查器的临时副本；`gazebo_observer.py` 只读采样，
`analyze_gazebo_run.py` 将 Gazebo 真值插值到里程计时间并用静止窗口对齐坐标。该采样器的安装来源标记针对报告中使用的临时安装目录；其他安装路径需自行核对运行动态库。

重算任一已保存运行，例如：

```bash
python3 src/small_point_lio/test/map_benchmark/analyze_gazebo_run.py \
  --samples ../docs/validation/data/moment_map_gazebo_20261028/moment.samples.csv \
  --result ../docs/validation/data/moment_map_gazebo_20261028/moment.navigation.json \
  --output /tmp/moment_gazebo_analysis.json
```

此命令在 `xxu_nav_2026` 目录执行，不启动仿真或节点。
