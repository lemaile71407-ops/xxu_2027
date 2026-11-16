# C++ perf 火焰图（2026-11-16）

按用户要求对重写后的 `zlio_base_odometry`、`cmd_vel_watchdog` 生成真实 CPU 火焰图。图形使用 Speedscope 1.25.0，提供 Time Order、Left Heavy、Sandwich 与线程选择。截图为主线程 Left Heavy 视图，交互文件保留所有采样线程。

| 节点 | PNG | 离线交互 HTML | Speedscope JSON | 进程样本 / 主线程样本 / 线程数 |
| --- | --- | --- | --- | --- |
| `zlio_base_odometry` | [截图](../../xxu_nav_2026/log/cpp_perf_20261116/zlio_base_odometry.png) | [交互图](../../xxu_nav_2026/log/cpp_perf_20261116/zlio_base_odometry.html) | [完整线程数据](../../xxu_nav_2026/log/cpp_perf_20261116/zlio_base_odometry.speedscope.json) | 658 / 117 / 3 |
| `cmd_vel_watchdog` | [截图](../../xxu_nav_2026/log/cpp_perf_20261116/cmd_vel_watchdog.png) | [交互图](../../xxu_nav_2026/log/cpp_perf_20261116/cmd_vel_watchdog.html) | [完整线程数据](../../xxu_nav_2026/log/cpp_perf_20261116/cmd_vel_watchdog.speedscope.json) | 1327 / 653 / 4 |

[图、文本栈及说明压缩包](../../xxu_nav_2026/log/cpp_perf_20261116/cpp_flamegraphs.zip)；原始二进制采样另保留为 [桥接 perf.data](../../xxu_nav_2026/log/cpp_perf_20261116/zlio_base_odometry.data)、[看门狗 perf.data](../../xxu_nav_2026/log/cpp_perf_20261116/cmd_vel_watchdog.data)。运行产物位于 Git 忽略的 `log` 目录，本文保存范围、方法和结论。

## 采样条件与符号

使用隔离 ROS 域 74、独立 Gazebo 分区，`tested_sim` 对应的 interval_mean IMU 和实验后端配置。完整导航/定位/扫描已就绪；机器人保持静止，没有下发目标、连接硬件或运行动态障碍。两个进程在相同时间窗各采集 15 秒，频率设置为 999 Hz、事件 `cpu-clock:u`、DWARF 用户栈 16384 字节。[采样命令与 PID](../../xxu_nav_2026/log/cpp_perf_20261116/capture.json)、[启动条件](../../xxu_nav_2026/log/cpp_perf_20261116/session.json)和[就绪证据](../../xxu_nav_2026/log/cpp_perf_20261116/sampling_ready.log)保留。

仅为两个目标所属的 `xxu_bringup` 包构建独立覆盖层：`build/perf_relwithdebinfo`、`install/perf_relwithdebinfo`，参数为 `RelWithDebInfo` 和 `-O2 -g -DNDEBUG -fno-omit-frame-pointer`。没有覆盖原 Release 安装，也没有修改业务代码。已核验库存在 `.debug_info`、`.debug_line` 和 `.symtab`，目标进程实际加载该覆盖层的组件库；[构建日志](../../xxu_nav_2026/log/cpp_perf_20261116/build.log)和[构建/单元核验](../../xxu_nav_2026/log/cpp_perf_20261116/build_verification.log)保留。

`Release` 本身不必然剥离所有函数符号；本次选择 RelWithDebInfo 是为明确保留 DWARF 和项目函数信息。`ZlioBaseOdometry::on_odometry()`、`CmdVelWatchdog::tick()` 已具名解析，部分预编译 ROS/DDS 库内部仍显示 `[unknown]`，不能声称依赖库完整符号化。[解析质量](../../xxu_nav_2026/log/cpp_perf_20261116/profile_quality.json)记录未知帧数量。

本机 `perf_event_paranoid` 原值 4，非特权探针失败；用户在终端将其设为 2 后，使用仅用户调用栈的采样成功。本任务未写永久 sysctl 配置，也未修改 kptr_restrict；该内核参数当前保持用户设置的 2。

## 读图口径

原始数据来自真实 `perf record`，没有生成或补造栈。每个 CPU 加权样本使用 `perf script` 头部的 `period` 纳秒数作为权重，保留调用顺序及线程身份；截图默认选择主线程，标题可切换 DDS 等其他线程。横轴累计的是采样 CPU 权重，**不是 15 秒墙钟时间轴**。全线程采样权重分别约 0.659 s 和 1.328 s，主线程约 0.117 s 和 0.654 s；这些是带采样开销的单轮估计，不作为无探针 CPU 占用验收。

原始 perf 文本可按 Speedscope 官方方法直接导入；其时间戳展示口径与本次按 CPU period 加权的 JSON 不同。比较 CPU 分布应使用本文 JSON/HTML，不能根据旧 py-spy 图的宽度宣称 CPU 降幅。当前为静止导航负载，不能替代运动、导航目标执行、压力测试或严格前后对照。

离线 HTML 内嵌 Speedscope 官方 CLI 的 base64 加载步骤、样式、字体和实际 JSON；没有修改其渲染器/导入器。压缩包附带 MIT 许可证，无需网络或上传采样文件即可查看。两张 PNG 已实际检查，浏览器未报告页面脚本错误；[最终检查](../../xxu_nav_2026/log/cpp_perf_20261116/final_checks.json)记录样本完整性与 HTML 摘要。

## 复现要点

先加载 ROS Jazzy、原工作区安装，再加载 profiling 覆盖层的 `local_setup.bash`。对同一节点启动后重新确认 PID；不要复用本次已退出的进程号。采样命令如下，`PID` 和输出名按目标替换：

```bash
perf record --no-buildid-mmap -F 999 -e cpu-clock:u \
  --call-graph dwarf,16384 --user-callchains -p PID -o node.data -- sleep 15
perf script --demangle -i node.data > node.perf
```

对照 [Speedscope 官方 perf 导入说明](https://github.com/jlfwong/speedscope/wiki/Importing-from-perf-%28linux%29)、[perf record 手册](https://man7.org/linux/man-pages/man1/perf-record.1.html)和[内核 perf 权限说明](https://docs.kernel.org/admin-guide/perf-security.html)。本次 [period 加权转换器](../../xxu_nav_2026/log/cpp_perf_20261116/perf_to_speedscope.py)与[离线 HTML 打包脚本](../../xxu_nav_2026/log/cpp_perf_20261116/build_speedscope_html.py)随运行产物保留。

采样结束已关闭临时导航及 Gazebo，[退出检查](../../xxu_nav_2026/log/cpp_perf_20261116/cleanup_check.json)确认该采样分区无剩余进程。原始 Release 工作区未重新启动。未执行 Git 提交或推送。
