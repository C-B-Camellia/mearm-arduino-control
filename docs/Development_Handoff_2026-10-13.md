# MeArm 开发冻结与独立审计交接 — 2026-10-13

本报告执行 STOP & SAVE：停止新增功能、重构与 Flash 优化，只提交已有源码、运行既有检查、保存证据和交接。没有上传固件、操作串口硬件、修改 Bringup、合并或同步标定。**这是可复现的开发冻结点，不是已通过实机验收的完整固件。**

## A. Git 状态与保存范围

- 工作目录：`D:\Develop\mearm-arduino-control`。
- 当前分支：`codex/emergency-fallback-2026-10-13`。
- 冻结前 HEAD：`d2d89202c8de9fced7a23eb102f5413a7d5594c1`。
- 最终源码 checkpoint：**`a57821b459c7a2b129403806399098e7afe565bf`**。
- 本报告随后单独提交，形成最终交接 HEAD；该文档提交不改变源码、配置或测试。以包含本报告的 Git commit 为最终文档 checkpoint，终端/最终回复记录其完整 SHA。
- 原 `assessment-sprint-2026-10-12` 分支保留。本阶段从 `2004de46b19dcf57f728b79aace1a55dc62e91fc` 开始，没有 reset、clean、强制 checkout、rebase 或合并。

### 本阶段全部新增源码 commits

| 完整 SHA | 内容 |
|---|---|
| `2455e3fddc03b12bc9197efa3b5bd2d5f3a60f0d` | Nano 消抖/启动保护/二进制按钮协议；Uno D2 中断采样 UART |
| `f50f0366c70676d157fd0bb8385276757560d494` | 八阶段运行时抓放示教、EEPROM 双槽保存、示教撤销及回归 |
| `8f795366b9c74f1ab60393c2d837e2882a283154` | 绘图独立 FEED；HOME、抓放、回放的速度隔离 |
| `d2d89202c8de9fced7a23eb102f5413a7d5594c1` | Nano 分页 OLED 本地提示；直接运行 Nano 主循环的模拟测试 |
| `a57821b459c7a2b129403806399098e7afe565bf` | 保存停止前已有的 TEACHZ、实体 UNDO、物体独立夹持角、SHORT/OVERWRITE 提示、运动辅助函数帧边界及测试 |

最后一个源码 checkpoint 只显式暂存下列 10 个已核对文件：`include/app_controller.h`、`include/teaching.h`、`src/app_controller.cpp`、`src/commands.cpp`、`src/manual_control.cpp`、`src/motion.cpp`、`src/pick_place.cpp`、`src/recording.cpp`、`src/teaching.cpp`、`test/native/sprint_tests.cpp`。没有使用 `git add -A`。冻结后所有跟踪的生产源码/测试/配置均已提交；本次没有为得到绿色构建删除代码，也没有继续修改业务逻辑。

### 有意保留的未提交文件与实验

| 路径 | 用途/状态 |
|---|---|
| `docs/MeArm_Kinematics_Derivation_2026-10-12.docx` | 冻结前已有的运动学 Word 材料，未提交，未修改 |
| `tools/build_kinematics_doc.py` | 已有 Word 生成脚本，未提交，未运行/修改 |
| `tools/render_kinematics_doc.ps1` | 已有 Word 渲染脚本，未提交，未运行/修改 |
| 根目录/Nano 的 `.pio/` | 忽略的本地构建、审计证据与实验，保留原始文件；不纳入生产 checkpoint |

未采用的 RAM 缓存/编译选项/返回通道实验留在 `.pio/cache_trial.py`、`.pio/flag_trial.py`、`.pio/function_trial.py`、`.pio/feedback_trial.py`、`.pio/feedback-capacity-trial/` 等。它们是历史实验，不是待上传候选；返回通道原型构建失败，不能当作正式功能。不存在尚未保存的跟踪生产 WIP；尚未完成的产品功能在 E 节列出。忽略目录不会随普通 Git clone 自动交付，后续审计若需要原始 ELF/Map，应保留本地目录。

### 已有 worktrees 与独立标定差异

| 工作区 | 分支 | 检查时 HEAD | 状态 |
|---|---|---|---|
| `D:\Develop\mearm-arduino-control` | fallback 分支如上 | 源码冻结 SHA 如上 | 本报告所在工作区 |
| `D:\Develop\MeArm-Bringup-2026-10-12` | `bringup-2026-10-12` | `2e78a3cdeddcb799531f4b0875876fa4ff0c80b5` | 正在独立实机调试；本轮只读 |
| `D:\Develop\mearm-master-test` | `master` | `b4e7de0f97e46a0b4911d7651bb308a6875f6baa` | 未修改 |

Bringup 检查时有四个未提交文件：`include/config.h`、`src/arm.cpp`、`src/joystick.cpp`、`test/native/sprint_tests.cpp`。实际只读 diff：

- JOINT 两档：Bringup **45/90**；fallback **40/80 °/s**。
- J2 启动/目标：Bringup **60°**；fallback **95°**；J1 均为 120°。
- Bringup ADC 正偏差取负方向，再乘轴符号；快档按 `abs(raw-center)>=500`。fallback 仍为正偏差正方向乘轴符号，快档触发为原始 ADC ≤8 或 ≥1015。
- Bringup 测试随上述修改调整，**本轮没有在 Bringup 运行测试或构建**。
- 用户说明 Bringup 正在做新的三维几何标定；检查时这些四文件之外没有出现在 Git 状态中的新几何配置。可能尚在进行或在外部记录，不能据此宣称标定不存在/完成。后续逐项审查同步，禁止直接覆盖任一工作区。

## B. 功能完成度与验收证据

调用流保留：PC/Nano 事件 → `dispatchAction/onButtonPressed` → 唯一 motion 任务或 manual → `arm.prepareOutputPose/requestPose` → 控制 tick → Servo/Timer1。Servo 仅由 arm 输出；浮点整 Pose 校验和整数截断后的组合校验都保留，失败不部分更新关节。暂停与预检仍占用 motion owner。

下表“自动测试通过”均指电脑模拟，不表示实体机械臂/EEPROM/按钮/OLED 已验证。Uno 的完整路径用合成标定验证；真实默认配置的门禁仍关闭未标定功能。

| 功能 | 当前源码入口 | 冻结后的自动测试/具体断言 | 完成状态与边界 |
|---|---|---|---|
| Nano 四键通信 | `nano/src/main.cpp`、`nano/include/button_scan.h`；Uno `button_link.cpp`/`pin_uart.cpp` | `testNanoInputAndLink`：非法编号/CRC、残帧、重复/跳号、HELLO、重启会话；Nano 实际 loop mock：启动按住不发事件、抖动、长按不重复、四键连续、TX 阻塞/有界队列、重启、序号回绕 | 扫描与协议已实现且模拟通过；Uno INT0/Timer2 指令时序及 Servo 兼容性仅构建通过，待示波器 |
| A/B/C 八阶段运行时示教 | `pick_place.cpp:capture/getPickStage/executePickCommand`；`motion.cpp:startPickPlace/validateOne` | `testRuntimePickPersistence`：仅 SRC/DST mask=34 不可运行；完整 mask=255；对象隔离、速度/等待解析、C 独立夹持角、搬运途中不得改变夹持角；末阶段不安全则零 Servo 输出并 HOLD | 已实现且模拟通过；八阶段全部用户明确示教，不盲目生成抬升；需真实安全路径、限位、HOME/夹爪标定 |
| EEPROM 持久化/断写恢复 | `pick_place.cpp:validSlot/restore/savePickConfiguration/updatePickPersistence` | 同测试：157 个写入边界重启恢复；唯一槽每个 155 字节损坏拒绝；新槽每个 155 字节损坏回退旧槽；版本/CRC/提交标记/字段非法拒绝；启动已有 pending Pose 仍能恢复；SAVE 时 GO/按钮/录制拒绝且零运动 | 双槽和显式保存已实现，模型测试通过；真实写字节中途掉电/BOD/磨损尚未验证 |
| 录制与回放/HOLD/GO | `recording.cpp:startRecording/updateRecording/compact`；`motion.cpp:startPlayback/executePlayback/pauseMotion` | `testAdaptiveRecording`：15 s 匀速、5 s 停顿后反向、20 ms 密集反向、四轴同时变化、80 帧附近、20 s 满时长、millis 回绕、首末帧/时间、误差界；回放先首帧、1700 ms 暂停时零输出、恢复时长、15007 ms 停止、timeScale=25 的限速披露；录制按钮取消/10 s 提示在相关测试中验证 | 已实现且端到端模拟通过；复杂轨迹允许明显精度下降；保存的是接受的指令，无位置反馈；严守关节速率时可能统一放慢原时间轴 |
| HOME | `motion.cpp:startHome/cancelAndReturnHome` | `testHomeAndPick`、`testFrozenTaskSpeeds`：未标定拒绝，完整预检；SPEED/H/L 两种设置下 HOME 时长相同 | 已实现、模拟通过；当前真实 `homeValid=false`，不能正式运动 |
| 五点示教/撤销/顺序/复用 | `teaching.cpp:beginMarking/markTeachPoint/undoTeachPoint/setTeachOrder` | `testTeachAndDraw`：未标定/缺 TEACHZ 拒绝进入存点；B4 UNDO 后 4 点且保留排列；满五点 MARK 拒绝；重复 ORDER 拒绝；反序数据对照 FK；CLEAR/EXIT 状态及存点复用 | 已实现、模拟通过；点集/纸面高度仅 RAM，断电不保留；TEACHZ 目前需 PC 捕获 |
| 折线、曲线、基础图形 | `motion.cpp:startPolyline/startCurve/startBasicShape/drawGoal`；`path_planner.cpp:centripetalSample` | `testCurveSampler/testCurveIndependentReference` 独立参考、端点/过点；`testTeachAndDraw` Poly/Curve/Line/N/Triangle/Z/V、暂停无输出、BTN3 抬笔 HOME、点附近整数指令 FK 距离<3 mm（合成值，非纸面实测精度）；`testFailureAndLatency` 不可达全路零输出 | 路径/预检/取消已实现、模拟通过；向心 Catmull–Rom 经过五点，采用采样段弦长计时；**没有整条轨迹加速度/前瞻规划或严格等弧长参数化** |
| JOINT/CART/四关节/串口 | `manual_control.cpp`、`kinematics.cpp`、`commands.cpp`、`arm.cpp` | `testSafety/testMath/test3DAndManual/testHoldAndJoystick/testManualFrameOwnership/testParser`：NaN/Inf、量化后组合限位、禁用轴、原子失败、方向/死区、3D 往返/不可达、倍率边界、任务独占、严格命令 | 已实现、模拟通过；当前生产只启用 J1/J2，CART/O/S/J0/J3 受标定门禁 |
| OLED 本地显示 | `nano/src/local_display.cpp`，`nano_oled` | Nano OLED loop mock 共 219 次断言：地址/时钟/timeout、分页继续扫描按键、文本长度、明确 LINK WAIT/TX!=accepted，不伪称主控权威状态 | 本地提示已实现、模拟及 AVR 构建通过；真实屏幕/地址/电平未验证；**主控返回协议与权威倒计时/任务状态页未实现** |

### 四键当前真实状态表

`BTN 1..4` 与 Nano 接收统一调用 `onButtonPressed()`。EEPROM SAVE 时优先 BUSY 拒绝；HOLD 保持脉冲目标，不切断 Servo 电源。

| 业务状态 | B1 | B2 | B3 | B4 |
|---|---|---|---|---|
| 普通空闲 | 执行下一 A/B/C；成功受理后推进选择 | 启动录制 ARMING | 回放已保存 take | 受控 HOME |
| ARMING/正式录制 | BUSY | ARMING 取消；正式录制保存 | BUSY | 丢弃 take，申请 HOME |
| 非绘图自动任务，普通上下文（含预检/暂停） | BUSY | BUSY | BUSY | 取消并申请 HOME |
| 示教设置 | 捕获 DRAWZ | 捕获 TRAVELZ | 进入存点，必须已有 TEACHZ | 退出示教 |
| 存点/五点已齐 | MARK | POLY | CURVE | UNDO 最后捕获点；PC CLEAR 返回设置 |
| 绘图（含预检/暂停） | HOLD | GO | 取消→抬笔→HOME | 无动作 |
| EEPROM SAVE | BUSY | BUSY | BUSY | BUSY |

B1 为每次按键执行一个对象，A→B→C→A；没有无人值守无限自动循环。抓放起始经 HOME/vias，八阶段结束停在 RETREAT，**不会自动再回 HOME**。绘图取消后任务变为 HOME 而示教上下文仍保留，此阶段四键全部 BUSY，PC HOLD/GO/CANCEL 仍可用；这是当前分发顺序的实际行为，需后续产品一致性审查。普通 CANCEL 取消任务并 HOLD；绘图 CANCEL 执行抬笔 HOME，失败保持。纸面模型是水平 z 平面；单一编译期 TCP 配置，不存在运行时 PEN/GRIPPER 双工具切换命令。

### 速度与控制时钟

- JOINT 40/80 °/s，CART 10/20 mm/s；H/L 仅手动倍率，默认 1.0，范围 0.75..1.5、步长 0.25。
- 普通串口关节/XYZ/RUNLINE 使用 SPEED（5..90 °/s）；H/L **目前不影响普通串口目标运动**。
- HOME 固定 60 °/s；抓放各阶段独立 PICK SPEED（5..90，捕获初值 60），等待 0..2550 ms / 10 ms 单位；这些默认值没有被认定为实测安全速度。
- 绘图 FEED 默认 10 mm/s，范围 1..40；采样段时间取弦长/进给及 60 °/s 关节上限的较大值。H/L、SPEED 不改变绘图，测试比较完整执行时长。
- 回放首帧预定位用 60 °/s，之后按记录时间戳；>120 °/s 的记录段统一放慢并打印 `PLAY timeScale=`，H/L/SPEED 不改变回放。
- 主控制 tick 20 ms、相位保留、漏周期不补发多帧；RUNLINE 等待下限 10 ms；Servo/Timer1 约 20 ms 脉冲周期独立。10 ms 不是 PWM 周期。

### 录制容量与误差的实际边界

`include/recording.h` 定义：`RecordedFrame { uint8_t angles[4]; uint16_t timeMs; }`，sizeof=6（static_assert），80 帧=**480 B**。模块静态元数据 15 B（status 6 + 时间戳 8 + 提示计数 1），录制模块合计 495 B。记录观察间隔 20 ms，ARMING 3000 ms 不计入 duration，配置最长 20000 ms。

每次 80 帧满时，有界扫描至多 78×4 个残差比较，移除一个内部点再追加；第一/最后端点保留，既不因密集动作提前耗尽缓冲，也不依赖平均压缩率。没有第二缓冲、局部帧数组或递归压缩；observe 单个局部 next=6 B，回放局部 a/b 共 12 B。因而**固定存储保证至少 15 s，配置支持 20 s**；这是数据结构容量保证，不是 AVR 20 ms 最坏执行时间证明。

先做严格时间共线合并，反向/停顿/夹爪转变端点优先保留；满容量后的有损重压缩可能损失这些细节，明确标记 DEGRADED。累计保守误差界最多 180°，不承诺复杂动作 ≤1°。最新测试对每个原始观察点重建，实测：15 s 匀速 0°；含停顿/反向 0.92308°；密集反向 11.9911°；15 s 四轴密集动作 16.1538°；20 s 四轴密集动作 19.95°。复杂场景报告界为 180°。记录持续时间超过 10 s 会提示，≤10000 ms 保存标记 SHORT；没有自动认证“明显真实运动”，也不强制阻止短 take 的调试回放。覆盖原 take 前 PC 提示 REC OVERWRITE。可选 12 s 单独提醒未实现。

## C. 最新构建与资源分析

### 本次检查证据（源码 a57821b；非历史 HEAD 冒用）

检查在 2026-10-13 01:55–01:56（Asia/Shanghai）执行，01:59:38 保存哈希快照。所有目标成功。编译/链接使用 AVR GCC 7.3.0、PlatformIO atmelavr 5.3.0、Arduino AVR 5.4.0；Uno Servo 1.3.0，Nano OLED U8g2 固定 2.36.17。完整合成在未变更的输入上增量检查成功，归档绑定当前源码 HEAD 和文件哈希；不是重新声明旧 commit 的构建。

| 配置/环境 | 状态 | .text B | .data B | .bss B | Flash (.text+.data) | 静态 SRAM (.data+.bss) |
|---|---|---:|---:|---:|---:|---:|
| 根目录 `uno`，真实默认未标定 | SUCCESS | 27350 | 282 | 987 | **27632 / 32256** | **1269 / 2048** |
| 根目录 `uno_fallback`，真实默认 + D2 按键 RX | SUCCESS | 28132 | 282 | 1025 | **28414 / 32256** | **1307 / 2048** |
| 隔离完整合成，脚本内 env `uno` | SUCCESS，禁止上传 | 31948 | 282 | 1025 | **32230 / 32256** | **1307 / 2048** |
| `nano/` 的 `nano`，四键最小固件 | SUCCESS | 2408 | 20 | 213 | **2428 / 30720** | **233 / 2048** |
| `nano/` 的 `nano_oled` | SUCCESS | 11296 | 236 | 734 | **11532 / 30720** | **970 / 2048** |

完整合成启用四轴、DistalLink、夹爪、HOME/两路点、运行时 EEPROM 抓放路径、D2 UART。标定数据全部是假测量，**不得上传 `.pio/synthetic-capacity-check` 或任何容量原型固件**。未注入可生产使用的 EEPROM 点位；有效标定使已实现数学/运动路径保留在链接结果中。默认生产的较小 Flash 来自未标定门禁可被编译器折叠，不能据默认余量宣称完整功能还有几 KB。

本次 Native：**PASS 1,404,106 checks**；Nano 不含 OLED **PASS 60**，含 OLED **PASS 219**。具体功能断言见 B 节，数量不替代逐项验收。

完整证据目录：`.pio/resource-optimization/freeze-2026-10-13/`。包含 build.log、production-and-synthetic.log、native.log、nano-tests-and-builds.log、firmware.elf、firmware.map、sections.txt、symbols.txt、disassembly.txt、metadata.json、freeze-manifest.json、各环境 sections、stack-frames.json。manifest 保存生产/测试/Nano/配置及有意未提交材料哈希。合成 ELF SHA-256：
`802a03bab94266feef3622a503262ef4ee31a33a7dd7ddb81be749200a6c62a3`。

复现命令（仅编译/测试）：

```powershell
# 根目录
& test/native/run.ps1 -Compiler 'C:/msys64/ucrt64/bin/g++.exe'
pio run -e uno -e uno_fallback
& 'C:/Users/12530/.platformio/penv/Scripts/python.exe' test/capacity_check.py --label NEW_UNIQUE_LABEL
# nano/ 目录
& test/run.ps1
pio run -e nano -e nano_oled
```

归档标签请使用新的名字，避免覆盖已有审计证据。没有为本轮检查修改生产逻辑。

### 静态对象与 Map 摘要（准确测量）

完整 Map：`.text=0x7ccc=31948`；`.data=0x11a=282`，SRAM 起始 0x0100，Flash 初始化镜像地址 0x7ccc；`.bss=0x401=1025`；`__bss_end=0x061b`，静态区外余额 **741 B**。调试段不计烧录容量。没有生产 heap/动态容器。

| 符号/对象 | 字节 | 段 | 声明位置 |
|---|---:|---|---|
| frames[80] | 480 | bss | `src/recording.cpp:8`；格式 `include/recording.h:7` |
| Serial（含 RX/TX） | 157 | bss | Arduino `HardwareSerial0.cpp`/`HardwareSerial.h` |
| hardwareConfig | 112 | data | `include/hardware_config.h:13`、`src/hardware_config.cpp:8`；const 并不意味着 AVR 自动存 PROGMEM |
| task | 104 | bss | `src/motion.cpp:14`/`:33`，AVR static_assert |
| joints[4] | 100 | data | `src/arm.cpp:9` |
| serialBuffer | 64 | bss | `src/commands.cpp:45` |
| points[5] | 40 | bss | `src/teaching.cpp:11` |
| Servo 库脉冲槽 | 36 | bss | `.pio/libdeps/uno/Servo/src/avr/Servo.cpp` |
| HardwareSerial 虚表 | 18 | data | Arduino 框架 |
| manualTarget / UART bytes | 各16 | bss | `src/manual_control.cpp`、`src/pin_uart.cpp:10` |
| TeachSettings | 15 | bss | `src/teaching.cpp:13`/`include/teaching.h` |
| cartTarget / 本项目 Servo 对象 | 各12 | bss | manual_control / arm |
| UART 解码 frame | 8 | bss | `src/button_link.cpp:8` |

Nano OLED 使用 128 B 的一页显示缓冲，不是 1024 B 全屏缓存；970 B SRAM 还包括 U8g2 对象、Wire/TWI 缓冲和 UART，不能把 128 B 当 Nano 总占用。Uno 没有 OLED 库/缓存。

### 主要 Flash 瓶颈（准确符号；非模块独占总和）

完整 ELF 最大代码符号：pollSerial 3486 B、main 2568 B、sampleGoal 1524 B、dispatchServiceAction 1400 B、inverse3D 1004 B、updateManualControl 998 B、execute 946 B、executePickCommand 934 B、centripetalSample 800 B、observe 794 B、solveIK 624 B、validateOne 578 B、solveFK3D 568 B、executePlayback 518 B。HELP 主文本符号 252 B；另有 PICK HELP 和分散日志常量。

main/pollSerial 经 LTO 含多个内联业务、解析/状态输出分支；不能把这个符号表直接解释为模块完整占用。数学库共享 sin/cos/acos/atan/atan2/sqrt、float 加减乘除和转换辅助：例如 __divsf3x=220 B、__mulsf3x=210 B、__addsf3x=204 B、sqrt=140 B、atan2=134 B，另有共享 __fp_* helpers。这些数字不是全部浮点库总占用，不能重复计费或假设删一个调用就释放全部共享库。

完整目标只剩 **26 B Flash**。本阶段从 32242/1380 B 基线到冻结 32230/1307 B，净 -12 B Flash / -73 B 静态 SRAM；同时增加 EEPROM、协议及交互功能。相关中间实测归档：fallback-buttons=31474/1291 B、fallback-pick-readonly=32190/1298 B、fallback-speed-final=32194/1302 B、当前=32230/1307 B；中间目录可能在提交前构建，准确绑定以各自 metadata 的 HEAD/源码哈希为准。

### 已尝试而未采用/不足的手段

- EEPROM 初次接入完整目标约 33540 B，超限；仅共用 EEPROM 读写/解析减少约16 B，不足。后续已采用编译期标定 const、共用日志薄函数及局部编译选项，没有改运动学公式或限位。
- 147 B RAM 抓放缓存原型只省约58 B Flash，静态 SRAM 达1445 B，未采用；正式抓放从 EEPROM 读取，没有整份 RAM 副本。
- `-fno-inline-functions-called-once` 等单独选项实验增长约1.3 KB；多种 IPA/调度/全局内联试验无效或变差，未采用。当前有效选项见 platformio.ini；禁止把试验脚本当可靠构建配置。
- 软件返回 UART 原型：33606 B Flash /1423 B SRAM，FAILED；相对当时32194/1302增加1412/121 B。位于 `.pio/feedback-capacity-trial/`，没有进入生产，且 SoftwareSerial TX 的整字节中断屏蔽会与 Servo/RX 时序冲突。这个结果只证明该原型不能容纳，不是所有返回协议实现的容量下界。
- 先前 d95b1f1 报告中的整数化 STATUS 单独尝试、独立 updateMotion 重帧尝试已撤回；旧报告是旧配置，不能当本冻结 HEAD 的资源结论。

### 栈风险：实测帧与推算边界

从冻结完整 ELF 序言读取的帧（局部空间+保存寄存器，不含2 B返回地址）：
main48、pollSerial99、onButtonPressed5、cancelAndReturnHome37、pointPose17、solveIK3D24、inverse3D34、solveFK3D66、solveFK22、sampleGoal85、execute68、validateOne44、centripetalSample70、curveAxis28、executePickCommand47、dispatchServiceAction46、observe31、updateRecording10。

两条重点链的**已知帧小计**，计入每层2 B返回地址、包含 main 返回地址：

- PC BTN3 绘图取消：main→pollSerial→onButtonPressed→cancelAndReturnHome→pointPose→solveIK3D→solveFK3D→solveFK：48+99+5+37+17+24+66+22+16 = **334 B**。
- 绘图执行：main→execute→sampleGoal→pointPose→solveIK3D→solveFK3D→solveFK：48+68+85+17+24+66+22+14 = **344 B**；预检相同链用 validateOne44 替代 execute68，小计320 B。

BTN/HOME 转发尾跳转，不携带46 B service 帧；inverse3D 返回后才进入 FK 核验；曲线 scratch 返回后才进入 IK，不能把这些互斥分支全部叠加。上述是机器码可见帧及人工调用链推算，**不是全程序栈峰值上界**；尚缺数学库内部调用、额外参数压栈、所有分支和中断同时预算。334/344 B 不能简单从741 B扣除就宣称剩余全可用。

手工复核 ISR 入口保存：Servo TIMER1_COMPA 17 B，UART TIMER2_COMPA 8 B，INT0 6 B，Timer0 overflow9 B，硬件 Serial RX10 B、TX15 B；还需硬件返回地址2 B及 ISR 内子调用（例如 digitalWrite）。stack-frames 自动解析器对 ISR 穿插 SREG 指令会低估，**ISR JSON 值不可作安全证明**。没有实机栈水位、库完整调用图或 ISR 延迟测量。历史报告中的部分“已知链”算术/边界需独立复算；这里仅按列出的当前帧重算，未沿用旧443/454 B标签。

### 下一位工程师最值得检查的三个方向（仅审计建议，本轮不实施）

1. pollSerial/main 的 LTO 内联、命令/日志常量和重复分支，建立当前符号/Map 归因，保留严格解析与诊断语义。
2. EEPROM LOAD/启动同步复制工作区的阻塞、编辑区写入磨损、保存/断电边界与双槽校验；审核 const/PROGMEM 数据放置是否真实释放资源而不绕过门禁。
3. 绘图/取消/IK 与浮点库/Servo/RX 中断的完整栈及延迟测量，先得到实机水位和逻辑分析仪数据，再决定局部调用边界及进给规划调整。

## D. 实机接口与操作速查

### 环境与接线（候选软件接口，尚未电气联调）

| 功能 | 接线/设置 |
|---|---|
| PC→Uno | USB HardwareSerial；115200 baud，换行提交命令；保持 Uno D0/D1 作为 USB 入口 |
| Nano→Uno 按键 | Nano **D1/TX → Uno D2/RX**，9600 baud，5 V TTL、共地；需 Uno `uno_fallback` |
| Uno→Nano | **没有启用返回线/协议**；Uno D3 未用；不得把外部 TX 接 Nano D0/RX 并忽略 CH340 共线 |
| Nano 四键 B1..B4 | D4..D7 分别经按键接 GND，INPUT_PULLUP、低有效；25 ms 稳定按下沿、长按无重复、开机按住先释放 |
| Nano OLED | Nano A4/SDA、A5/SCL；SSD1306 128×64，候选 I²C 0x3C、400 kHz；实际模块地址、VCC 和上拉确认后接入 |
| Uno Servo | J0/D9、J1/D8、J2/D7、J3/D6；独立 Servo 供电方案需电路组确认 |
| Uno 摇杆 ADC | J0/A0(center506)、J1/A1(519)、J2/A3(519)、J3/A2(515)，死区±20；方向差异见 Bringup 表 |

当前 Uno 使用 **INT0+Timer2 采样 RX**，已替代 SoftwareSerial 接收；不是原先要求中“直接复用库”的实现。只支持16 MHz、D2、9600，编译断言。Timer0 保留 millis，Timer1 保留 Servo，Timer2 独占 RX，禁止 Uno tone()/Timer2 PWM。16 B RX 环（可用15）及8 B帧，溢出/错停止位使解码失步等待 HELLO；PC 日志和 EEPROM 阻塞下可能丢按钮，未证明实体吞吐保证。

按电路组要求，USB 烧录与外部供电互斥，烧录断开板间杜邦线。本候选避开 Uno D0/D1 板间通信，但不等于全部供电/CH340 共线风险已解除。Nano D1 仍是板自身硬件 UART/USB 路径，下载时断开板间线。没有执行任何上传命令。

### 8 字节板间协议

`A5 type epochLo epochHi seq key crc8 5A`；CRC8 poly0x07，覆盖前6字节。HELLO type0/key0，按键 type1/key1..4。Nano 开机更新 EEPROM 0/1 的16位 epoch；事件 seq 递增，空闲约1 s HELLO。

Uno 首先收到合法 HELLO 才接受同 epoch 下连续 seq 的按键；重复忽略，非法/残帧/间隔>40 ms/跳号/缓冲溢出失步，直到 HELLO。HELLO 永不驱动机械臂。Nano 重启/断电不凭空产生事件；但链路掉线**不会主动取消正在执行的 Uno 任务**，PC HOLD 是独立入口。Nano 单向只能确认发出，OLED 固定显示 LINK WAIT、LOCAL KEY、TX sequence、TX!=accepted/KEY QUEUE FULL、PC=SYSTEM STATUS，不假称主控倒计时/状态。

### PC 指令速查（区分计算与运动）

| 指令 | 语义/注意 |
|---|---|
| HELP / STATUS | 只读；STATUS含 J、autoSpeed、feed、manualScale、mode、task、phase、held、cal、enabled、error、pickNext、workflow、REC、points/paper |
| IK x z / IK3 x y z | 计算解；IK3需真实三维标定；不直接运动 |
| J0..J3 angle | 目标任务，经完整安全预检；未启用轴拒绝 |
| x90,y120,z95 | 小写 x/y/z 分别表示 J0/J1/J2，原子目标；这不是 XYZ 坐标 |
| XY x z / LINE x z / RUNLINE x z | XY目标；LINE预检预览；RUNLINE原二维直线执行 |
| XYZ x y z | 三维目标运动，需 CART 标定 |
| MODE JOINT / MODE CART | 手动模式切换，CART未标定拒绝 |
| O / S / H / L | 全局标定夹爪开/闭；手动倍率提高/降低 |
| SPEED value / FEED value | 普通目标速度 / 独立绘图进给；任务中修改拒绝 |
| HOLD / GO / CANCEL / HOME | 暂停/继续/取消/HOME；CANCEL绘图特殊抬笔回HOME；HOME缺参保持拒绝 |
| A / B / C / BTN 1..4 | 指定物体一次抓放 / 统一按键事件 |
| REC / REC START / REC STOP / PLAY | REC切换ARMING/保存；PLAY安全首帧准备后回放 |
| TEACH START / DRAWZ / TEACHZ / TRAVELZ / MARKING / EXIT | 子命令都需 `TEACH ` 前缀；捕获当前已接受指令对应的笔尖高度 |
| MARK / UNDO / LIST / CLEAR / ORDER 1 2 3 4 5 | 五点录入/撤销/查看/清空/严格五点排列 |
| DRAW POLY / DRAW CURVE | 复用已有五点及指定顺序 |
| DRAW LINE x1 y1 x2 y2 | 当前纸面高度上的平面直线 |
| DRAW SHAPE N\|TRIANGLE\|Z\|V [xmin ymin xmax ymax] | 显式范围，或用前两点边界 |
| PICK SET A APPROACH\|SRC\|GRIP\|LIFT\|TRANSFER\|DST\|RELEASE\|RETREAT | 捕获当前接受的四轴整数量化指令；阶段也可用1..8 |
| PICK SHOW A / PICK CLEAR A | 查看/清除当前编辑区；B/C同理 |
| PICK SPEED A GRIP 40 / PICK WAIT A GRIP 200 | 示例：该阶段速度/等待；重新 SET 会重置该阶段默认60°/s、100 ms |
| PICK SAVE / PICK LOAD | 显式持久化/恢复；均保持 HOLD；SAVE完成输出 PICK SAVED; HELD，须显式 GO |

串口严格十进制，不接受 NaN/Inf、指数记法、未知额外参数；单行最大63字符，过长丢弃至换行并报错。每循环最多64字节、处理一条完整命令以给运动/按键服务机会。常见输出：`OK`、`ERR ...`、`PATH VALID`、`DONE(cmd)`；DONE仅说明指令序列结束，不能认定实体动作到位。枚举 TaskKind/phase 的数字见 `include/motion.h`。

### EEPROM 配置步骤与数据布局

先完成真实 J0/J3 限位/startup、全局夹爪开闭有效标记、HOME/vias。然后分别用串口或摇杆将机械臂引导至已确认安全的八阶段姿态，等待指令已接受、摇杆居中，逐次 `PICK SET A <stage>`，对 B/C 同样操作；不能只示教 SRC/DST。

1 APPROACH、2 SRC 是该物体打开角；3 GRIP、4 LIFT、5 TRANSFER、6 DST 必须保持同一该物体夹持角；7 RELEASE、8 RETREAT 保持同一释放角。夹持角须区别于打开/释放，每轴仍受真实安全限制。每对象可不同，不强行用全局 S 的同一角度夹所有物体。

用 PICK SPEED/WAIT 设置经过实机验证的各阶段参数；SHOW mask=255 表示八阶段齐全，**不等于障碍物路径已证明安全**。PICK SAVE 后等待 `PICK SAVED; HELD`；GO 才恢复手动。断电/LOAD只恢复最近完整提交槽，无自动运动；当前软件所有关节安全/禁用轴检查仍需通过。CLEAR仅清编辑区，需SAVE才持久化，LOAD可撤销未保存编辑。

每阶段6 B（4×uint8角度、1×deg/s、1×10 ms等待）；每对象49 B（mask1+8×6），总payload147 B。header8 B：magic0x4D、version1、generation16、CRC16、length147、commit0xA5。槽0地址0..154、槽1地址155..309、编辑区310..456，共457 B，主控与Nano各自独立EEPROM。CRC16 poly0xA001包含generation和payload；提交标记最后写，旧槽在新槽完整提交前保留。

SAVE在主循环逐字节、等待 eeprom_is_ready；编辑 SET/LOAD/开机恢复工作区为同步 EEPROM.update，最多复制147字节，真实耗时未测、最差可能数百ms。编辑区不是掉电恢复来源。没有写整个payload的RAM缓存。真实brownout期间单个字节撕裂、BOD及寿命需补硬件测试；Native仅模拟写入边界和损坏。

### 当前真实默认配置与缺失参数

`src/hardware_config.cpp` 的生产 `const HardwareConfig hardwareConfig{}` 所有新功能有效位为false；绝不复制容量脚本的假值。真实默认 J1=120° / J2=95°，启用；J1范围20..125°、J2范围40..130°、组合 J1+J2≥148°。J0/J3输出禁用，名义0..180不代表已标定安全限位。原注释“calibrated elbow startup(95)”不能替代当前 Bringup 的实测确认。

仍需：J0/J3 minimum/maximum/startup；3D yawSign/yawZero、肩轴径向/高度、toolModel和三轴TCP偏置；夹爪开/闭；HOME与最多两个安全via；对象八阶段完整姿态/速率/等待与障碍物间隙；DRAWZ/TEACHZ/TRAVELZ、纸面接触高度与真实进给；Nano屏地址/电平；ADC方向/中心及最终速度。两种工具TCP与倾斜纸面也未提供/未实现。

第一条只读 PC 检查可用115200发送 STATUS/HELP，但 **启动时先 attach J1/J2 并输出120/95脉冲**，所以“只发STATUS”不等于系统从未输出运动指令。未核对Bringup差异和安全物理初态前，保持Servo供电/信号隔离，只做USB控制器通信；预期当前fallback STATUS中 `J=90 120 95 50`、`enabled=6`、`cal=0`、`held=0`、`task=0 phase=0`，EEPROM空白时对象mask0。异常日志/复位/值与所选分支不符时停止接入运动硬件，检查实际源码、端口、供电。不要用合成固件来“解除”标定门禁。

## E. 当前限制、阻塞与接手结论

**可安全交由另一位 AI 从本冻结点独立审计；不能宣称完整四轴实机候选已经完成。** 本轮没有证据支持硬件已验证，Bringup才是用户独立标定区，不能自动修改。

阻止正式完整运动的条件：

- 当前fallback缺真实J0/J3、3D/TCP、HOME、夹爪及八阶段工装数据，门禁关闭相应功能；Bringup方向/启动角差异须人工审查，几何标定还在进行。
- 完整Flash仅26 B余量；后续有效标定/函数保留/编译器或库改变就可能超限。所有功能的真实最终标定组合仍需重建，默认产物不能替代完整容量证据。
- SRAM静态余额741 B尚无全程序栈上界/实机水位，尤其绘图、PC BTN3取消、库调用及中断。
- 自制UART采样接收与Servo实机时序未验证；串口日志/EEPROM同步操作可能导致按键丢失，协议失步恢复不等于实时可靠性通过。
- USB/外部供电及CH340相关连接遵循电路组互斥/断线要求；当前软件接口不替代电气风险结论。
- 所有姿态是已接受指令；无实际位置反馈，无碰撞/障碍物模型。全路关节限位预检不证明“任意物理初态进入安全”或工装路径无碰撞。

尚未完成或有产品差距：

- OLED高级权威系统页、Uno→Nano返回协议：原型超容量且时序风险，正式仅单向本地提示，PC承担所有权威状态/倒计时。按用户允许的备用范围保留，不牺牲官方算法。
- 全轨迹加速度/前瞻/严格弧长规划；现有为向心曲线采样和段级进给/角速率约束。
- PEN/GRIPPER运行时工具切换、独立双TCP、倾斜纸面模型；真实标定不足且未实现。
- 点集/录制持久化、可选12 s独立提醒、真实运动有效性认证：没有实现，不假称READY代表官方演示合格。
- 普通串口目标的H/L倍率：目前仅手动；该“建议”未扩展到串口目标。
- 前一轮单独 `docs/Tonight_Bringup.md` 未创建；STOP后本报告提供当前接口/安全首检边界，不恢复旧文档任务。
- Word材料、官方任务四推导最终排版、PPT均未继续，不以固件测试代替材料交付。
- Nano/按键/OLED实物、两板联调、真实EEPROM掉电、Servo热/供电、笔尖精度和抓放成功率全部待人执行验收。

冻结后只提交本交接文档并核对Git状态/源码哈希；不恢复开发，不合并Bringup，不上传固件。
