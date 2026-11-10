# Z-Localization 源码迁移记录（2026-10-31）

## 当前状态

按本次迁移要求，将完整 Z-Localization 源码及配置复制到 [导航工作区 src](../../xxu_nav_2026/src/z-localization/)。它包含前端、后端、消息接口、bringup 和 Livox 驱动；不是仅复制 ZLIO 前端。来源为独立仓库的提交 `76e126328b670adc92981d9b4900cc059a6f2343` 加该仓库当时已有的未提交修改，包括固定窗口候选修复和默认关闭的恢复连续性检查。复制时逐项比对了源码、配置、cmake 文件、README 和 LICENSE；未复制独立仓库的 `.git`、`build`、`install`、`log`、`deps`、`third_party` 或机器本地构建说明。原 [独立工作树](../../z-localization/)及其本地修改、依赖和构建产物保留。

复制后的源码仅进入工作区，尚未接入正式传感器、TF 或导航启动链。现有启动代码仍调用 Small Point-LIO 和 AMCL/GICP；删除范围待用户确认。后端恢复、跨代次可信参考、Nav2 降级保护及实车适配的限制仍以[仿真对照报告](zlocalization_comparison_20261030.md#恢复连续性候选检查2026-10-31)为准。

已增加单独启用的 ZLIO 底盘里程计适配器，并用 EGA 录包检查候选 TF 链；后端定位状态和地图输出一致性仍有待解决的问题。范围和结果见 [ZLIO 底盘 TF 验证](zlio_base_tf_20261031.md)。

## 旧代码本地备份

[备份压缩包](data/localization_migration_20261031/previous_localization_and_bringup.tar.gz)包含旧 Small Point-LIO、`xxu_gicp_localization`、`xxu_bringup` 启动文件、Python 启动工具、配置及包清单，以及 LIO 和导航工具目录。压缩包解压后与原目录逐项比较一致，[SHA-256 校验](data/localization_migration_20261031/previous_localization_and_bringup.sha256)通过。该包及录包数据仅保留在本地，不包含整个仓库、构建安装产物或独立 Z-Localization 工作树。

## 检查

ROS 2 Jazzy 下使用 `colcon list --base-paths xxu_nav_2026/src --names-only` 能发现 `frontend`、`backend`、`localization_interfaces`、`bringup` 和 `livox_ros_driver2`，未报告同名包冲突。包发现不等于构建或运行验证。

2026-10-31 在隔离的临时构建与安装目录，以单进程构建新位置的 Livox 驱动、消息接口、前端和后端，4 个包完成；随后单独构建 bringup，1 个包完成。构建使用原独立仓库的本地 `deps` 前缀中的 GTSAM 4.2.0 和 Livox SDK2，清空旧 `LD_LIBRARY_PATH` 后加载 ROS 和该前缀；使用 `-Wno-error=maybe-uninitialized` 以兼容本机 GCC 13，编译仍有警告。[构建事件](data/localization_migration_20261031/colcon_build_events.log)和 [bringup 构建事件](data/localization_migration_20261031/bringup_build_events.log)只保留包开始、结束及退出码，已移除环境变量、命令输出与设备路径。

在该隔离构建目录执行 `backend_graph_test`、`backend_relocalization_test`、`backend_localization_test`、`frontend_input_test`、`frontend_adapter_test`，5 个 CTest 目标通过，[测试结果](data/localization_migration_20261031/test_results/)留在本地。未进行整工作区构建、安装到正式工作区、传感器运行、导航闭环或实车验证。

同日修复审查问题：迁移副本的 [Warnings.cmake](../../xxu_nav_2026/src/z-localization/cmake/Warnings.cmake) 对 GNU 13.x 将 `maybe-uninitialized` 保留为警告，其他编译警告仍按原设置处理。清空之前手动设置的 `CMAKE_CXX_FLAGS` 后，使用相同本地依赖和隔离目录重建驱动、接口、前端、后端，4 个包成功；前端的该类警告仍有输出。复跑上述 5 个 CTest 目标全部通过；额外用未使用变量告警确认其他告警仍导致构建失败。[筛选后的构建结果](data/localization_migration_20261031/warnings_fix_build_events.log)和[脱敏测试结果](data/localization_migration_20261031/warnings_fix_test_results/)留在本地。此检查没有运行整工作区构建或设备测试。

两次构建事件记录均只保留包开始、结束和退出码，测试 XML 中的源码路径改为项目相对路径；原始环境字段和本机路径未纳入上述证据。

因为候选包现在位于工作区 `src`，普通工作区全量构建会发现它们；构建前需要准备 GTSAM 4.2、Livox SDK2 及其 CMake/动态库搜索路径。本机可参照原独立仓库的 [Jazzy 依赖说明](../../z-localization/JAZZY_LOCAL.md)，但该 `deps` 是本地未纳入主仓库的产物，不能假定其他机器已有。原有启动链仍依赖旧包，不应把源码复制或上述测试视作完成替换。
