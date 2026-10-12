# MeArm 当前源码内存归因审计（只读）

审计对象：`assessment-sprint-2026-10-12`，HEAD `b3db827` 加审计开始时已存在的未提交修改。生产 `src/`、`include/`、`platformio.ini` 的 SHA-256 前后相同，见 `production-sha256.json`。未提交 Git、未上传硬件、未修改生产代码或录制容量。

`current-full` 是隔离的完整启用检查目标：四轴有效、DistalLink TCP、夹爪/HOME/两路点有效、三个 8 阶段抓放程序有效、SoftwareSerial RX=2/TX=3/9600。全部标定和外板引脚均为**合成测试数据，禁止上传**；生产配置仍未标定。没有假设 OLED/蜂鸣器存在。它覆盖当前已实现完整代码路径，不表示先前产品需求已全部实现，也不是实际硬件的最终配置。

## 1. 段大小：实测

全部按相同本机 PlatformIO/Arduino/Servo 工具链重建。AVR GCC 7.3.0，Uno 应用 Flash 上限 32256 B，SRAM 2048 B。审计 Map/栈信息选项未改变下面的代码大小。

| 目标 | `.text` Flash | `.data` Flash + SRAM | `.bss` SRAM | Flash 总计 | 静态 SRAM |
|---|---:|---:|---:|---:|---:|
| Batch 1 `6df83a1` | 21804 | 266 | 551 | 22070 | 817 |
| Batch 2 `fdc36ca` | 24846 | 298 | 1144 | 25144 | 1442 |
| Batch 3 `b3db827` | 30192 | 206 | 1100 | 30398 | 1306 |
| 当前默认配置 | 30302 | 206 | 1100 | 30508 | 1306 |
| **当前完整启用检查** | **31878** | **336** | **1097** | **32214** | **1433** |

完整目标 Flash 余量 **42 B**，SRAM 静态区以上 **615 B**。之前 32108 B 是旧工作区构建，本次当前工作区结果比其多 106 B。`.debug_*`、`.comment` 等 ELF 调试段不计入烧录 Flash/SRAM。`.data` 初始化镜像既占 Flash，又在启动时复制进 SRAM；不能只把 `.text` 当 Flash 总占用。

证据：各目标的 `sections.txt`、`symbols.txt`、`firmware.map`、`build.log`、ELF；完整目标 ELF SHA-256：`b865f6e502c68a1789a2a9ab432d06dfc7d65fd7879a87915fb83c2dddfa7835`。

## 2. 完整目标最大的 RAM 符号：实测

| 符号/对象 | B | 段 | 原生产声明 |
|---|---:|---|---|
| `frames[80]` | 480 | bss | `src/recording.cpp:6`，结构 `include/recording.h:5` |
| `Serial` HardwareSerial 实例 | 157 | bss | 框架 `cores/arduino/HardwareSerial0.cpp:70`；RX/TX 成员 `HardwareSerial.h:113`、`:114` |
| `task` MotionTask | 120 | bss | `src/motion.cpp:13`、`:26` |
| `hardwareConfig` | 112 | data | `include/hardware_config.h:13`、`src/hardware_config.cpp:5` |
| `joints[4]` | 100 | data | `include/arm.h:15`、`src/arm.cpp:9`；每项 25 B |
| `serialBuffer[64]` | 64 | bss | `src/commands.cpp:39` |
| `SoftwareSerial::_receive_buffer[64]` | 64 | bss | 框架 `libraries/SoftwareSerial/src/SoftwareSerial.cpp:51` |
| `points[5]` | 40 | bss | `src/teaching.cpp:10`，每点 XY 两个 float |
| Servo 库 `servos[MAX_SERVOS]` | 36 | bss | `.pio/libdeps/uno/Servo/src/avr/Servo.cpp:35`，Uno 12 个 3 B 槽 |
| `link` SoftwareSerial 实例 | 31 | bss | `src/button_link.cpp:8` |
| `joysticks[4]` | 28 | data | `src/joystick.cpp:4`；`JoystickAxisConfig` 在 `include/joystick.h:12`，包含 pin/center/deadzone/direction |
| HardwareSerial 虚表 | 18 | data | 框架 HardwareSerial 类 |
| SoftwareSerial 虚表 | 18 | data | 框架 SoftwareSerial 类 |
| `manualTarget` | 16 | bss | `src/manual_control.cpp:17` |
| `cartTarget` | 12 | bss | `src/manual_control.cpp:18` |
| 本项目 `servos[4]` | 12 | bss | `src/arm.cpp:16`，每个 Servo 对象 3 B；和库内脉冲槽不同 |
| `settings` TeachSettings | 10 | bss | `src/teaching.cpp:12` |
| `order[5]` | 5 | data | `src/teaching.cpp:11` |

其余计数、标志、时间戳和动作表见完整 `symbols.txt`。全部命名 bss 符号合计 **1097 B**，无未解释 bss 缺口。命名 data 符号合计 **298 B**，另有匿名浮点常量 16 B、普通字符串 21 B、对齐 1 B，合计 `.data=336 B`。

`task` 的 120 B 主要是 4 个 Pose=64 B、2 个 Point3D=24 B、3 个 uint32 时钟=12 B，加状态/索引等=20 B。四轴 Servo 对象和库内 12 个槽在默认构建已经存在；启用 J0/J3 本身不会再新增两套对象。

## 3. 录制实际格式与静态/局部存储

当前真实定义，不是 RDP/关键帧压缩：

```cpp
constexpr uint8_t RECORD_CAPACITY=80;
constexpr uint16_t RECORD_INTERVAL_MS=150;
struct RecordedFrame { uint8_t angles[JointCount]; uint16_t timeMs; };
RecordedFrame frames[RECORD_CAPACITY];
```

AVR 编译但不链接的 sizeof 探针实测：`sizeof(RecordedFrame)=6`、`sizeof(Pose)=16`、`sizeof(RecordingStatus)=4`，证据 `sizeof_probe.cpp`、`sizeof.txt`。探针没有加入固件，不影响段占用。

- 录制帧静态缓冲总量：**80×6=480 B**；没有第二套备份、压缩窗口、原始样本缓存或递归 RDP。
- 录制模块其他静态元数据：`status=4 B`，`startedAt=4 B`，`lastSample=4 B`；模块合计 **492 B**。
- 节拍是距上次实际采样至少 150 ms；主控制 tick=20 ms 时常见间隔为 160 ms。首帧 t=0，随后最多 79 个间隔；最短满容量时长 **79×150=11850 ms**，稳定 20 ms tick 下通常 **12640 ms**。当前 **不保证 15 s**。本轮按指示不调整算法或容量。
- `append()` 的局部 `RecordedFrame f` 在 C++ 层面为 6 B；停止用 `commandedPose()` 的 16 B 返回值。没有局部帧数组，无录制算法递归。`finalize()` 是两层有界循环。
- 回放 `sampleGoal()` 有单个 6 B 临时帧，`execute()` 有两帧 a/b=12 B；它们不与另一份录制缓冲并存。
- 录制切换经过 `dispatchAction()` 的有限自调用；不是递归压缩。BTN 经 Button→RecordToggle→RecordStart/Stop 最多 3 层 dispatchAction。禁止将“没有录制算法递归”解释为应用分发完全无嵌套。
- 编译后局部对象常被寄存器分配/复用/内联：不能把源码所有局部变量尺寸简单叠加为实际峰值。最终 `append` 栈帧为 11 B 局部空间+18 B 保存寄存器，不含调用返回地址。

全固件主要字节缓冲（互不重复计数）：录制 480 + PC 命令行 64 + PC 硬串口 RX/TX 128 + 外板软串口 RX 64 + 外板报文 3 = **739 B**。Serial 的 128 B 已包含在其 157 B 实例内，不能二次计入静态 SRAM 总量。另有点集 40 B、顺序 5 B 和上述控制数组；A/B/C 的程序表 **435 B 在 PROGMEM**，不占 SRAM。

解析期间局部数组：`commands.cpp:88` 的 `tokens[7]` 是 AVR 指针数组 14 B；应用解析 `order[5]` 是 5 B、`values[4]` 是 16 B。不是录制专用存储，但录制期间收到命令仍会执行；经过 LTO 后大部分包含在 main 的固定栈帧内。

## 4. 历史 SRAM 增减精确归因

Batch 1→2：`817+625=1442 B`。

| 变化 | B |
|---|---:|
| 新增 `frames[96]` | +576 |
| 录制 status/start/sample | +12 |
| MotionTask 115→118 | +3 |
| `nextObject`、`workflow` | +2 |
| RAM 字符串 94→127 | +33 |
| data 对齐 1→0 | -1 |
| **合计** | **+625** |

Batch 2→3：`1442-136=1306 B`。

| 变化 | B |
|---|---:|
| 录制 96→80 帧，576→480 | -96 |
| 新增 points40/settings10/count1/active1/marking1 | +53 |
| MotionTask 118→120 | +2 |
| workflow 改为计算得出 | -1 |
| 自写十进制解析替代 strtod，errno 移除 | -2 |
| 新增 order5/动作映射表8 | +13 |
| RAM 字符串 127→21 | -106 |
| data 对齐 0→1 | +1 |
| **合计** | **-136** |

当前默认→完整启用：`1306+127=1433 B`。SoftwareSerial RX64 + 实例31 + head/tail2 + active_object指针2 + 虚表18 + 外板解析状态10 = **127 B**。hardwareConfig 的 112 B 从 bss 移到 data，仅改变初始化与 Flash 镜像，**不增加总 SRAM**。生产配置尚未启用外板，所以必须用完整目标审计这些 127 B。

## 5. Flash 来源与 Map 摘要

完整目标 Map：

```text
.text  0x00000000 0x7c86 = 31878
.data  0x00800100 0x0150 =   336   load address 0x7c86
.bss   0x00800250 0x0449 =  1097
__bss_end         0x00800699
__data_load_end   0x00007dd6 = 32214
.noinit = 0; .eeprom = 0
```

AVR ELF 用 0x800000 区分 SRAM 地址；实际静态区 `0x0100..0x0698`，RAMEND=`0x08ff`，剩余 `0x0699..0x08ff`=615 B。

`.text` 31878 B 的 Map 分解（互不重叠）：LTO 合并代码 **27478**，应用/Arduino PROGMEM 常量 **1536**，构造析构指针4，libm（含浮点算术、三角函数、系数表）**2051**，libgcc（含共享序言/尾声和启动段）**494**，libc **188**，CRT/向量126，对齐1。之和=31878；再加 `.data` 初始化镜像336=32214。

PROGMEM 常量1536=字符串/选择器字面量879 + A/B/C程序435 + Arduino引脚表90 + 两套命令名表132。879 中 HELP 六行占325 B；这些已经不占 SRAM。Print 方法独立机器码合计 **752 B**，含 float 格式化；打印调用点的机器码还内联在 main 等函数中，不能只算字符串长度。

| 当前 ELF 的大代码符号 | B | 实际含义与原生产来源 |
|---|---:|---|
| `main` | 9900 | Arduino main + 内联 setup/loop、PC解析/状态输出、手动控制、轨迹推进；`src/main.cpp`、`commands.cpp:81`、`app_controller.cpp:78`、`manual_control.cpp`、`motion.cpp` |
| `sampleGoal` | 2838 | 所有任务采样 + 内联 drawGoal/centripetalSample；`motion.cpp:113`、`:69`、`path_planner.cpp:16` |
| `solveIK3D` | 1730 | `src/kinematics.cpp:85` |
| `dispatchAction` | 1350 | `src/app_controller.cpp:13`；部分业务已内联 |
| `solveFK3D` | 752 | `src/kinematics.cpp:67` |
| `drawVertex` | 682 | `src/motion.cpp:38`，含基础图形采样 |
| `solveIK` | 624 | `src/kinematics.cpp:30` |
| `startDrawing` | 396 | `src/motion.cpp:301` |
| `solveFK` | 384 | `src/kinematics.cpp:15` |
| Servo Timer1 ISR `__vector_11` | 376 | Servo `avr/Servo.cpp` |
| `Print::print(double,int)` | 362 | 框架 `Print.cpp`；AVR double 为 32 位 |
| `curveKnots` | 312 | `src/motion.cpp:50` |
| `strokeSteps` | 294 | `src/motion.cpp:56` |
| `append` | 266 | `src/recording.cpp:9` |
| `parseNumber` | 260 | `src/commands.cpp:18` |
| `onButtonPressed` | 250 | `src/app_controller.cpp:49` |
| Servo::write | 248 | Servo 库 |

重要边界：LTO 已把曲线公式内联到 sampleGoal，把 executeLine/executeAppCommand/updateMotion 等合并到 main。因此 **不能**把 2838 B 全称为曲线成本，也不能把 9900 B 全称为解析成本。曲线、解析、日志和状态机共享浮点库。准确独立边界是上述 ELF 符号及 Map；当前编译结果不支持无歧义的“每模块总占用”百分比。额外 debug 行号归因存在 LTO 信息失真，本报告不采用它。

libm 的 2051 B 为 Map 实际节贡献，不是把 atan/acos/sin/cos 别名大小重复相加。主要浮点算术成员 `addsf3x.o=192 B`、`divsf3x.o=204 B`、`mulsf3x.o=194 B`（最终 nm 的公开符号范围和对象节范围可能不同）；三角函数、sqrt、浮点转换与归一化均共享这些依赖。libc188 B 包含 strtok_r78、memcpy_P18、strcmp_P18、strchr22、strstr52；没有 strtod。

## 6. 运行时栈评估：已测量与推测分开

**结论：615 B 是静态区外余额，不是已证明安全的栈余量。当前较紧，尚不能签署栈安全通过。**

最终完整 ELF 的序言反汇编实测（含本函数保存寄存器，不含 2 B 调用返回地址）：

| 函数 | 局部空间 | 保存寄存器 | 帧合计 |
|---|---:|---:|---:|
| main | 131 | 18 | 149 |
| sampleGoal | 117 | 16 | 133 |
| pointPose | 13 | 4 | 17 |
| solveIK3D | 69 | 18 | 87 |
| solveFK3D | 52 | 18 | 70 |
| solveFK | 6 | 16 | 22 |
| append | 11 | 18 | 29 |
| dispatchAction | 29 | 18 | 47 |
| Print::printNumber | 33 | 8 | 41 |

绘图可达嵌套链 `main→sampleGoal→pointPose→solveIK3D→solveFK3D→solveFK`：149+133+17+87+70+22，加 6 个 2 B 返回地址，**已确认 490 B**。这不是最大栈上界，而是该链到 FK 时的确定消耗；尚需加浮点库内部 push/call 以及任意此刻到来的中断。

Servo ISR 保存17 B + 硬件返回地址2 B，调用 digitalWrite 又需2 B，共21 B（digitalWrite 此构建未再保存寄存器）。SoftwareSerial ISR 保存15+硬件2=17 B；Timer0 ISR 9+2=11 B；串口 RX ISR8+2=10 B。当前 ISR 不主动开放中断嵌套，故不简单把四种 ISR 相加。

依据浮点库反汇编的额外保存/调用，该数学深链叠加一层 ISR **估计约520–550 B**，余额约65–95 B。此区间是人工路径估计，不是经过全控制流证明的全程序最大值，更不是实机水位。其他恢复/解析路径、间接虚调用和有限事件自调用仍需要覆盖。源码和 ELF 未见 malloc/calloc/realloc/new 的运行时调用，无动态录制容器；没有 heap 增长不代表栈必然安全。

诊断限制：普通 `-fstack-usage` 的 LTO .su 输出为空；尝试把该选项传到最终 LTO 链接会触发本机 AVR GCC 7.3.0 internal compiler error。诊断失败只发生于隔离副本，不影响成功的完整目标。故本报告以最终 ELF 序言为局部帧证据，**没有伪造全程序栈上界**。

需要后续实际验证：独立诊断固件做栈填充/最低 SP 水位，覆盖曲线预检/运行/取消、录制时 STATUS/HELP、回放暂停恢复、PC持续输入且外板同时按键、Servo/串口中断负载；必须在已标定安全条件下进行。当前本轮只读，未加入该诊断代码。平台构建成功仅证明链接容量未溢出。

审计没有通过缩减帧数或删除安全检查改善结果；产品纠偏、录制15秒、Flash余量恢复继续保持暂停。
