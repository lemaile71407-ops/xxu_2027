# EGA 折叠车模转换

`rmul_navigation.launch.py` 接入固定折叠构型，继续使用现有 Small Point-LIO、AMCL、MPPI、fake 坐标系及 watchdog。
模型入口为 [model.sdf](../../../src/xxu_bringup/models/ega_folded/model.sdf)，ROS 描述为 [robot.urdf](../../../src/xxu_bringup/models/ega_folded/robot.urdf)。
几何参数以 [geometry.json](../../../src/xxu_bringup/models/ega_folded/geometry.json) 为准，物理参数来源和近似以 [physical_properties.json](../../../src/xxu_bringup/models/ega_folded/physical_properties.json) 为准。

## 固定折叠构型

用户已确认新压缩包保存的折叠形态；新主装配 SHA-256 为 `7dc96c6144956791e94abc36e0db66cc013a646b5409f921d608eef8905660b4`，不得混用之前的同名装配。
折叠机构和其他非驱动结构固定，四个全向轮独立驱动。轮心、轴向、实体轮面半径和宽度从 CAD 提取，轮组的“范围”参考体不参与尺寸、碰撞或惯性计算。

实车只有一台 MID360；生成器保留原先用于导航的第一台及其 CAD 安装姿态，移除第二台雷达、对应保护罩、传感器输出与 TF，并扣除质量、重心及惯量贡献。第二处安装支架仍保留，原始 CAD 不改写。
雷达光学原点和轴向按 [Livox 手册 v1.2](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf) 第 12、14、19 页设置：光学原点高于底面 47 mm，IMU 在光学坐标系中偏移 `[11,23.29,-44.12]` mm。
这属于厂家尺寸与 CAD 装配换算，未进行实车外参标定。固定安装的点云通过静态 TF 变换，IMU 保留杆臂补偿；进入 LIO 前按车体凸包及轮体过滤自身回波。

CAD 的封闭半球形雷达保护罩会遮挡扫描。对保留的雷达，生成器支持 `closed`（真实遮挡）、`removed`（取下，扣除 CAD 罩体质量与惯性）与 `transparent`（保留显示及质量，假设透激光）三种模式；被移除的雷达不再保留罩体。
当前模式及是否取下保护罩见物理参数清单；实车罩体使用方式待用户确认，不能将透射假设当成已标定材料属性。

原始双雷达 CAD 保存的质量约 18.03 kg，尚未实车称重。原始两台雷达的缓存惯量明显违反尺寸上界，因此先从根装配惯量中扣除其错误张量，替换为 65×65×60 mm、265 g 盒体初值，保留 CAD 重心及平行轴项，再扣除第二台雷达及移除的罩体。当前单雷达、取下罩体的运行质量约 17.744 kg，精确值以物理参数清单为准。
四轮与雷达拆分后的总质量、重心、惯量通过 [校验工具](validate_folded_robot.py) 检查；它不证明实车材料设置正确。

碰撞使用四组固定结构凸包和四个轮面圆柱，外观保留已解析的三角形并合并同材质绘制批次。
部分导入件没有可读显示网格，缺失实例逐项记录在物理参数清单，模型不是完整实体 CAD 的无损替代。
全向轮方向摩擦、关节阻尼、速度/加速度限制均是未辨识初值，驱动仍为理想关节速度控制，不代表实车电机、电流或制动距离。

导航包络取实际可见车体最大 XY 旋转半径再加 2 cm，覆盖 fake 模式共享参数；近场停车区外扩 3 cm。
扫描高度按折叠车高加 2 cm 设置。其他平台和上游车模继续使用原配置。
折叠车型通过 `geometry.json` 选择已有 `StoppedGoalChecker`，平移速度低于 0.05 m/s 且仍在位置容差内才报告到达，不约束目标朝向。车型导航配置可收紧估计位置的判定阈值，为定位误差留余量；真实停点验收仍按运行报告的原有容差。该设置不引入预测定位器。

单点导航参数由 [navigation_profile.yaml](../../../src/xxu_bringup/models/ega_folded/navigation_profile.yaml) 保存，LIO 仿真观测权重由 [lio_profile.yaml](../../../src/xxu_bringup/models/ega_folded/lio_profile.yaml) 保存。前者的 `navigation` 分支协调膨胀范围、控制器与速度平滑器的 XY 限速以及到点判定；膨胀范围至少覆盖停车多边形并留一格缓冲，已有更宽的配置不会被缩小。后者仅用于 `rm_sim` 的该车型，不代表实车标定。生成车模时沿用这两份配置，不在生成器内重复保存导航数值。

## 生成与检查

以下路径均相对于 `xxu_nav_2026`，原始 CAD 只读，输出保存至指定目录。工具环境需有 numpy、scipy、trimesh 4.12.0、fast-simplification 0.1.13；原网格参考体分离依赖 trimesh 的相关依赖。

```bash
python tools/navigation/cad/convert_ega_sentry.py \
  local_assets/cad_source/folded_0517 <patched_reader> local_assets/ega_folded_decode
python tools/navigation/cad/prepare_folded_robot.py \
  local_assets/cad_source/folded_0517 local_assets/ega_folded_decode \
  <patched_reader> src/rm_sim_26 src/xxu_bringup/models/ega_folded --cover-mode removed
python tools/navigation/cad/validate_folded_robot.py src/xxu_bringup/models/ega_folded
colcon build --symlink-install --packages-select xxu_bringup small_point_lio xxu_description
```

新增资源文件后必须重新构建启动包，不能仅依赖已有文件的安装软链接。
当前折叠车型使用 RGL，`gpu_lidar` 后端仍仅支持上游车型。

生成物校验见 [static_validation.json](../../../src/xxu_bringup/models/ega_folded/static_validation.json)。当前导航修复、实测参数、重复回归及剩余限制统一见 [runtime_validation.json](../../../src/xxu_bringup/models/ega_folded/runtime_validation.json)。2026-10-18 的三个目标中有两个真实停点超差，原报告归档为 [20261018_runtime.json](../../../src/xxu_bringup/models/ega_folded/validation/20261018_runtime.json)。核验均使用取下保护罩的运行假设；实车罩体处理、质量、摩擦和电机参数仍未实测确认。

### 导航适配回归

以下命令在 `xxu_nav_2026` 内执行。先检查解析后的配置及覆盖顺序：

```bash
source install/setup.zsh
python3 tools/navigation/test_vehicle_parameters.py
python3 tools/navigation/check_interfaces.py
```

运行回归需要 RGL 已构建，并使用空闲的 ROS 域 73 和唯一的 Gazebo 通信分区。检查器独立启动无界面仿真，只停止自己创建的进程组；输出目录下的运行名称必须未被使用：

```bash
export ROS_DOMAIN_ID=73
export GZ_PARTITION=xxu_vehicle_check_20261027_run1
python3 tools/navigation/check_ega_navigation.py rmul_single_point \
  --output-dir /tmp/xxu_navigation_checks
```

默认依次测试原卡住目标、回程及三个短距离目标。检查器只读观测 Gazebo 真值，分别记录 Nav2 成功状态、宣布到达时和停车后的真实位置误差、速度出口、轮速、急停状态与数据计数；有目标未完成、超出原 0.12 m 真实误差要求或未停车时返回失败，不能仅凭 Action 成功判通过。停车观测取最后 0.75 秒，各数据流至少 3 个样本、覆盖至少 0.25 秒源时间，末样本的源时间和接收年龄均不超过 0.5 秒；真值平移/转动范围不超过 5 mm/0.01 rad，里程计速度不超过 0.02 m/s/0.05 rad/s，同时要求安全指令和轮速为零附近。证据保存在各目标的 `parking_evidence`，旧报告未包含此字段的结果不能代替该项验证。`--profile` 和 `--lio-profile` 可显式加载候选 YAML；后者使用仅对子进程生效的临时包目录，避免覆盖正式配置。

当前改善仍有范围限制：部分目标存在进度恢复与到点收敛慢，雷达盲区继续存在。运行结果不能替代实车标定、用户验收或发布授权；具体样本和未完成项以当前运行报告为准。

### 单雷达盲区量化

2026-10-19 已按当前单雷达、取下保护罩构型，结合详细车体网格、厂商视场和已读取的运行过滤参数完成离线盲区计算，并用运行点云抽样核对车体遮挡。26.63 cm 高障碍物在距底盘中心 1 m 环上的导航几何盲区方位约 66.1%；这不是实车漏检概率。盲区图、分高度/方向数据、采样证据及复现入口统一见[盲区量化报告](../../../src/xxu_bringup/models/ega_folded/blindspot_analysis/REPORT.md)。本次未修改导航默认参数或安装姿态。

## 底层显示网格读取器

仅提取保存的显示网格、装配位姿和名称，再作展示上色；不求解 SolidWorks 配合，
不生成质量/惯量或驱动关节，也不求解折叠角度。单独的缓存网格不能替代折叠后的动力学模型；上方生成器补充物理属性与固定构型的连接关系。
如更换姿态，需在 SolidWorks 2025 中切换配合、重建并保存；完整实体转换优先导出保留零件层级的整车 STEP AP214/AP242，仅导出可见实体，排除参考球和平面。
原始 CAD 输入保留在本地 `local_assets/cad_source/`，不纳入 Git；转换输出应写入新的本地目录。

读取器来自 [XRTC5/sldprt-export](https://github.com/XRTC5/sldprt-export)，MIT 许可，
固定提交 `ebde7524b3876c5d78c7e2248f2eba9b5725a6c0`。
本目录的补丁针对这批 CAD 的目录跳号、缓存选择、覆盖计数和父隐藏状态作适配。
只在独立读取器 checkout 应用，不修改用户 CAD。

准备独立读取器和 Python 环境后：

```bash
git -C <reader_checkout> checkout ebde7524b3876c5d78c7e2248f2eba9b5725a6c0
git -C <reader_checkout> apply <project>/xxu_nav_2026/tools/navigation/cad/sldprt-export-ega.patch
<python_env>/bin/python -m pip install numpy scipy trimesh==4.12.0 fast-simplification==0.1.13
<python_env>/bin/python convert_ega_sentry.py <extracted_cad> <reader_checkout> <output>
```

`extracted_cad` 应包含哨兵主装配及配套 SLDPRT/SLDASM，保持原文件名。
输出 GLB 明确标为 draft；JSON 给出未解析实例、显示块 CRC、原始与可视边界、
零件数量和来源散列。NPZ 保留原始提取网格。文件读取和转换在本机完成。

取消了破坏开槽薄板的减面；UW01 的大球/平面参考体按其随文件预览排除，
不使用这些参考体作车体碰撞或雷达遮挡。黑/灰/浅色是展示风格，不是材料密度判断。
