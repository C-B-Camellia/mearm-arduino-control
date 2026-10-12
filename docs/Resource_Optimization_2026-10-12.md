# MeArm 资源优化结果 — 2026-10-12

分支 `assessment-sprint-2026-10-12`。先读取 `docs/Memory_Attribution_Audit_2026-10-12.md` 并复现基线，用 `2af8732` 保存优化前未提交源码及需求文件。最终源码 checkpoint：`d95b1f1`。

## 实测完整目标与逐项收益

每一行都实际运行完整合成 Uno 构建，不按源码行数估计。配置始终启用四轴、DistalLink、夹爪、HOME/两路点、三个完整8阶段抓放程序，以及 SoftwareSerial RX2/TX3/9600。数据全部为测试合成标定，禁止上传。

| 优化/证据目录标签 | Flash B | 本项变化 B | 静态 SRAM B | 本项变化 B | checkpoint |
|---|---:|---:|---:|---:|---|
| baseline | 32214 | — | 1433 | — | 2af8732 |
| compact-help：短 HELP，STATUS 不再复制完整 Pose | 32180 | -34 | 1433 | 0 | d1bea97 |
| compact-format：共用0/1/2位十进制诊断输出 | 32064 | -116 | 1421 | -12 | d1bea97 |
| packed-pick：PROGMEM 整数姿态表 | 31864 | -200 | 1421 | 0 | 2d34d43 |
| curve-frame：曲线 scratch 在 IK 前释放 | 31768 | -96 | 1421 | 0 | ac42a97 |
| parser-frame：串口解析独立栈帧 | 31564 | -204 | 1421 | 0 | ac42a97 |
| flash-joystick：配置迁入 PROGMEM，循环采样 | 31574 | +10 | 1393 | -28 | a2cf93e |
| shared-parser：共用严格数字参数解析循环 | 31512 | -62 | 1393 | 0 | 1f9f22a |
| shared-command-names：示教/图形共用 Flash 命令索引 | 31458 | -54 | 1393 | 0 | 1f9f22a |
| shape-table：与原坐标完全等价的16 B顶点表 | 31298 | -160 | 1393 | 0 | 1f9f22a |
| single-help：六行 HELP 合并为一次输出 | 31264 | -34 | 1393 | 0 | 1f9f22a |
| task-payload：互斥目标 Pose/Cartesian 参数共用存储 | 31264 | 0 | 1377 | -16 | 4325083 |
| compact-notices：启动与录制覆盖通知缩短，语义保留 | 31230 | -34 | 1377 | 0 | d95b1f1 |
| manual-frame：手动 IK scratch 不占主循环持久帧 | 31148 | -82 | 1377 | 0 | d95b1f1 |
| event-frame：BTN/HOME 转发不携带重业务帧 | **31154** | +6 | **1377** | 0 | d95b1f1 |

最终恢复 **1060 B Flash、56 B 静态 SRAM**；完整目标 Flash 余量 **1102 B**，SRAM 静态区外余额 **671 B**。event-frame 的6 B Flash代价换取取消路径去除47 B业务栈帧，保留。

实测不保留的尝试：单独整数化 STATUS 打印使完整 Flash 升至32286 B，撤回；独立 updateMotion 帧虽再省46 B Flash，却使当时绘图已知栈链420→472 B，撤回。未删功能、限位或 UART，也未缩减录制数组。

## 最终段、符号与可复现证据

| 配置 | .text | .data | .bss | Flash总计 | 静态SRAM |
|---|---:|---:|---:|---:|---:|
| 完整合成 | 30858 | 296 | 1081 | **31154** | **1377** |
| 真实源码默认未标定配置 | 29328 | 166 | 1084 | **29494** | **1250** |

完整证据保存在 `.pio/resource-optimization/<标签>/`：`firmware.elf`、`firmware.map`、`sections.txt`、`symbols.txt`、`disassembly.txt`、`build.log`、`metadata.json`。最终归档标签 `final` 的 metadata 包含 HEAD、全部生产源码哈希及 ELF SHA-256。

复现：

```powershell
& test/native/run.ps1 -Compiler 'C:/msys64/ucrt64/bin/g++.exe'
& 'C:/Users/12530/.platformio/penv/Scripts/python.exe' test/capacity_check.py --label final
& 'C:/Users/12530/.platformio/penv/Scripts/pio.exe' run -e uno
```

完整构建脚本在合成标定/抓放表注入标记变化时直接失败，防止将未完整启用的目标当成容量证据。默认生产 `.pio/build/uno` 与合成 `.pio/synthetic-capacity-check` 分开。

AVR 符号实测：录制 frames **480 B** 不变；MotionTask **120→104 B**；摇杆配置 **28 B 从 data 迁到 PROGMEM**；三个抓放表 **435→147 B PROGMEM**。抓放表减少288 B，但解包机器码增加，最终净收益以构建实测200 B为准。

## 栈：机器码证据与未验证边界

最终 ELF 序言实测，包含保存寄存器、不含2 B调用返回地址：

| 函数 | 优化前帧 B | 优化后帧 B |
|---|---:|---:|
| main | 149 | 102 |
| sampleGoal | 133 | 81 |
| centripetalSample | 内联在sampleGoal | 74，返回后才进入IK |
| solveIK3D | 87 | 87 |
| solveFK3D | 70 | 70 |
| solveFK | 22 | 22 |
| pointPose | 17 | 17 |
| pollSerial | 内联在main | 97 |
| dispatchAction | 47 | 0；BTN/HOME使用尾跳转 |
| dispatchServiceAction | 不独立 | 47，仅服务分支使用 |
| updateManualControl(now,input) | 内联在main | 79，串口/运动前释放 |

绘图已知链 `main→sampleGoal→pointPose→IK3D→FK3D→FK2D`，包含返回地址：**490→391 B，减少99 B**。额外的曲线计算帧与 IK 帧分时使用，不相加为同时峰值。

本轮还补查了旧审计未作为最大链列出的 PC `BTN 3` 绘图取消：旧链 `main→dispatchAction→onButtonPressed→cancelAndReturnHome→pointPose→IK3D→FK3D→FK2D` 的已知帧/返回地址约 **550 B**；新链多一个pollSerial，但转发不保留dispatchAction重帧，约 **553 B**。这条链的绝对帧总量没有降低；其静态区外余额增加56 B，已知余量约 **65→118 B**。

上述已知链尚未包含浮点库内部调用和中断。取消路径加入这些开销后，人工估计余量约 **70–90 B**，仍较紧。不是全程序控制流上界，也不是实机栈水位；因此**不宣称栈安全通过**。未新增 heap 或全局 math scratch。未来15秒录制方案必须继续把这条恢复路径纳入预算，不能把671 B全部视为可分配缓冲。

## 回归与冻结内容

最终 Native：**9598 次断言通过**，并保留原测试套件。新增对应验收：

- HELP 全部命令与六行输出；STATUS 数值字段；非法多参数解析保持零输出。
- 十进制0/1/2位、负数、进位、NaN/Inf/溢出标记；格式化只用于诊断，不参与目标计算。
- 整数抓放解包与安全量化一致；量化后组合限位不安全或NaN的测试配置拒绝启用。
- 四个ADC分别测试正负方向、死区边界和快速档；RX/RY映射不交换。
- 四种图形全部顶点逐点精确对照原坐标；向心曲线独立参考及交错调用不保留scratch。
- JOINT→CART→HOME→RUNLINE任务参数复用；CART积分、验证/暂停任务独占；PC按键取消后仍受控HOME。

默认和完整 Uno 均构建通过。`git diff 2af8732` 核实 `kinematics.cpp`、`arm.cpp`、`recording.cpp`、`recording.h`、硬件标定源/头文件均未改变；三维/二维运动学公式、安全限制和标定有效门禁保持原样。录制仍为 **80×6 B、150 ms**，15秒算法及ARMING/产品交互仍待下一轮。

## 新硬件信息边界

副板为独立ATmega328P-AU，SSD1306 I²C 128×64 OLED及四键由副板负责。主控没有加入OLED驱动或帧缓冲。本轮保留SoftwareSerial双向能力，尚未实现下一轮副板状态协议，也未将新硬件视为电气联调通过。

USB烧录与外部供电按电路组要求互斥；烧录断开板间杜邦线。UART/CH340G共线风险尚未解除，不能据本轮合成构建声称板间通信安全可用。没有上传或运行任何合成标定固件。
