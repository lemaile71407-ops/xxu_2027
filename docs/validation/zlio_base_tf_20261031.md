# ZLIO 底盘 TF 候选适配与回放（2026-10-31）

## 结论与使用范围

已新增单独启用的 [ZLIO 底盘适配器](../../xxu_nav_2026/src/xxu_bringup/scripts/zlio_base_odometry.py)及[启动入口](../../xxu_nav_2026/src/xxu_bringup/launch/zlio_base_adapter.launch.py)。该候选未接入正式导航启动，旧定位未删除。EGA 固定雷达录包中，适配器输出的 `odom → base_footprint` 与输入和固定 TF 一致，回放时也能查询到 `map → base_footprint`。完整定位可靠性及地图位姿输出的一致性仍未验证通过。

适配器将 ZLIO 的输入子帧 `imu` 明确映射为已补偿的刚性虚拟传感器 `lio_base_sensor`，按输入时间查找 `lio_base_sensor ← base_footprint`，变换位姿、刚体速度和对应协方差，发布 `/odom` 与相同时间的 `odom → base_footprint`。错误帧、零时间或缺少 TF 时不发布。刚体速度换算不适用于仍相对底盘运动的原始云台传感器帧；该映射和标定须与实际输入一致。入口、参数与单发布者条件见 [LIO 使用说明](../../xxu_nav_2026/tools/lio/README.md#zlio-底盘里程计候选适配)。

## 回放验证

2026-10-31 使用此前第二段 [EGA 仿真录包](data/zlocalization_compare_20261030/navigation_retry_bag/)与同包生成的先验地图，0.5 倍速完整回放。隔离运行 `src/z-localization` 候选前后端和当前适配器，前端关闭自身 TF；播放器不发布旧 `/odom`，发布原包 `/tf_static`；后端负责 `map → odom`。前端使用原补偿点云/IMU 回放参数，后端使用原先验定位参数。回放与现有历史对照一样不是独立地图、实车、动态云台或导航闭环测试。

- 回放正常结束，停止服务成功。记录到 7591 条前端传感器里程计、7591 条底盘 `/odom` 和同数的 `odom → base_footprint` TF；输出消息和 TF 同时间戳、位姿完全一致。基于录得的两条静态变换逐条重算传感器到车体，7591 条最大平移数值差约 `4.2e-17 m`，旋转差为零。
- 固定链 `base_footprint → base_link → lio_base_sensor` 的平移为 `(0, 0, 0.069738 m)`，旋转为单位四元数。同一次回放内传感器和底盘位姿的 XY 差最大约 0.610 mm，RMS 约 0.574 mm；因此该录包中漏掉此固定变换不足以解释此前米级恢复跳变。这一结论不推广到安装位置或姿态不同的实车。
- 底盘 `/odom` 与 Gazebo 真值按首个共同样本做平面 yaw/XY 对齐，760 个样本的水平 RMSE 为 1.51 cm、最大 6.33 cm；这是当前前端回放的相对轨迹指标，非全局定位精度。不同回放的结果不构成严格同条件性能对照。
- 录得 632 条后端 `map → odom` TF，并在回放期间通过 TF 查询获得 `map → base_footprint`。后端日志记录 21 次进入 SUSPECT，未见进入 LOST 的日志；不能把 TF 连通当作全程可信。原后端兼容输出还发布独立的 `map → body`，本次配置把 body 外参设为单位变换，因此此 `body` 不代表已验证的 `base_footprint`，不能用于替代该底盘 TF。
- `map → odom` 与 `/localization/odometry` 分别由后端帧更新与高频预测发布，源时间接近不保证发布于同一后端状态。按源时间最近且最大间隔 7 ms 的粗略对照，最大平移差约 0.110 m，该样本的旋转差约 20.3°；全体最大旋转差约 21.55°。最大平移差样本的地图 TF 比被对照的预测晚约 0.120 s（录包时钟）。再按 TF 实际录制之后的首条预测对照，632 个样本的平移差 P95 约 1.25 cm、最大 6.86 cm，旋转差 P95 约 0.51°、最大 12.38°；这些样本仍可能跨状态更新或冻结，不能作为严格同版本的一致性判定。需要按后端版本、发布顺序和可信状态继续核验，不能宣称该项通过。

[逐项分析和样本统计](data/zlocalization_compare_20261030/tf_adapter_retry_20261031/tf_analysis.json)、[分析脚本](data/zlocalization_compare_20261030/tf_adapter_retry_20261031/analyze_tf_chain.py)、[回放状态](data/zlocalization_compare_20261030/tf_adapter_retry_20261031/run_summary.json)及该目录下录包为证据。第一次启动因环境中不兼容的 USB 库导致后端退出，未进入回放；清理旧动态库搜索环境后重试得到上述结果。整包回放后调整了 SIGINT 关闭处理和 TF 缓存使用节点时钟的方式，未改变坐标换算；重建后用该输出包中的前端里程计与静态 TF 重播 38 秒的录包片段，观察到各 1685 条输入、底盘里程计和 TF，[复测结果](data/zlocalization_compare_20261030/tf_adapter_retry_20261031/post_change_replay.json)留存。第一次短回放没有采到预期输出，重新启动观察器后复测得到该结果，未确定首次未采到的原因；独立启动及关闭检查通过。

## 其他验证与未完成项

三个针对性测试通过，覆盖带旋转和杆臂的位姿、速度及协方差换算、非有限数据、帧名不符和 TF 缺失；`xxu_bringup` 已在隔离安装目录构建成功。尚未进行候选链的 Nav2 闭环、后端退化时导航保护、独立地图、动态云台、实车标定或全工作区回归。模拟点云帧内时间相同，不能验证真实逐点去畸变；不得据此切换正式默认定位。
