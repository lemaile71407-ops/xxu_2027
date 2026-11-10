# fake_vel_transform

上位机小陀螺与导航速度适配节点。详细职责、TF、启动及回归说明见
[小陀螺导航说明](../../tools/navigation/README.md)。

- 读取真实 `/odom`，发布唯一的稳定坐标系 `gimbal_yaw_fake`。
- fake TF 的父坐标系是 `/odom.child_frame_id`（项目中为 `base_footprint`）。
- 发布 `/odom_nav`：稳定参考系的位姿与平面速度，虚拟角速度为零。
- 将稳定参考系的导航平移指令转为 `base_link` 指令，移动时叠加 `spin_speed`。
- 真实的 `base_link → gimbal_link → 雷达` TF 保持独立，供传感器补偿与定位使用。

`cmd_vel_watchdog.py` 也在本包安装，负责最终速度出口的命令、里程计、扫描、关节和 TF 新鲜度检查；它在普通坐标系模式下同样运行。命令同时检查单调时钟接收年龄和 ROS 源时间戳年龄，过期、未来或格式异常的源时间戳会关闭输出。输出消息使用副本，刷新输出时间戳不会续期缓存的源指令。共享启动定义位于 `xxu_bringup`，按平台参数调用本包的 watchdog。

候选全局定位可显式启用 `require_localization:=true`，默认关闭。定位门同时要求后端声明可信、未冻结，且定位状态及 map 系全局里程计的接收时间、源时间和源时间推进均新鲜；`NORMAL` 和后端声明可信的 `OBS_DEGRADED` 可进入恢复观察，其余状态、字段异常或版本回退均关闭输出。重复旧消息、零时间戳状态通知不能续期源时间。ROS 源时间最多容忍 50 ms 时钟投递偏差，接收和源时间推进另用单调时钟检查，因此暂停 `/clock` 不会让旧源消息一直有效。

`localization_state_topic` 默认 `/backend_node/localization_state`，`localization_pose_topic` 默认 `/localization/odometry`；后者要求 `header.frame_id=map`。`localization_timeout` 默认 0.5 秒，`localization_recovery_hold` 默认 1.0 秒，均为候选配置值。失信或超时清除缓存命令；连续可信且新鲜达到恢复观察时间后，还需新的速度请求才能放行。后端本地代次独立于前端线上代次，不按数值等同。该门控没有提供 Nav2 任务取消或重新规划策略，短时仿真结果见[候选链验证报告](../../../docs/validation/rgl_zlio_full_chain_20261031.md#导航及保护)。

下表为速度转换器参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `odom_topic` | `Odometry` | 原始底盘里程计；项目 launch 设置为 `/odom` |
| `nav_odom_topic` | `/odom_nav` | 导航用里程计 |
| `nav_velocity_smoothing_window` | `0.0` | 水平速度的因果平均窗口（秒）；项目仿真/实车入口设为 `0.3`，设 `0` 可关闭 |
| `robot_base_frame` | `base_link` | 输出指令坐标系 |
| `fake_robot_base_frame` | `gimbal_yaw_fake` | 唯一稳定导航参考系 |
| `input_cmd_vel_topic` | `cmd_vel` | 项目使用 `/cmd_vel_collision` |
| `output_cmd_vel_topic` | `aft_cmd_vel` | 项目使用 `/cmd_vel_transformed` |
| `spin_speed` | `0.0` | 可运行时更新；项目仿真/实车入口默认传入 `1.5` rad/s |
| `gyro_linear_threshold` | `0.01` | 叠加自旋的平移速度阈值 |
| `publish_rate` | `100.0` | 按最新底盘航向重算输出的频率 |
| `command_timeout` | `0.3` | 原始速度指令超时 |
| `odom_timeout` | `0.5` | 里程计接收间隔及消息时间戳的最大允许年龄（秒） |

到点/碰撞零指令会停止底盘自旋。无输入时不输出运动；断流时输出零速度。
重复、倒序、过期或超前当前 ROS 时钟超过 0.1 秒的里程计会被丢弃，
不会刷新超时计时或更新航向。有效且时间戳递增的数据恢复后可继续输出。
仿真时间回退后，在时间戳超过上一条有效数据之前保持停车；重置仿真时应重启此节点。
运行时仅支持修改 `spin_speed`；其他参数修改后重启。

2026-10-17 已按用户确认删除有界航向外推、对应参数及诊断。节点只保存最新有效里程计航向，按最新测量值转换指令；帧变化重置速度窗口。历史实验数据与报告已删除。

导航速度先从底盘系旋转到稳定的 odom 系，再对当前及最近窗口内的有效消息做
算术平均。只滤波 `/odom_nav.twist` 的水平速度，保留最新位姿和时间戳；原始 `/odom`
继续用于定位、TF、底盘航向与数据新鲜度判断。角速度仍为虚拟参考系定义的零。
不同父/子坐标系之间不混合历史，断流超过窗口后第一条有效速度直接使用新样本。
没有速度死区；低速运动不被强制置零。滤波协方差保持原有保守值，未经实测标定。

0.3 秒窗口在约 9 Hz 里程计上通常平均 3 条消息，带来约 0.1 秒量级的速度反馈延迟。
