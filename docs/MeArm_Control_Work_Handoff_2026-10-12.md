# MeArm 控制组完整功能冲刺：架构冻结与 Work 交接包 V1

**日期**：2026-10-12  
**官方提交截止**：2026-10-18 23:59（以《十二期二面考核及部分三面说明.pdf》为准）  
**内部目标**：10-16 晚功能冻结；10-17 联调与材料；10-18 复查并提前提交。  
**状态**：接口设计提案 / 待与当前仓库逐项核对，不是“所有代码已实现”的声明。

## 0. 必须遵守的优先级

当前实际硬件证据、官方器件资料 > 当前仓库源码/构建/日志 > 考核 PDF > 本文建议 > 旧聊天推测。

使用本文件时还请阅读以下三份项目内文件：
- `Electrical_Control_Tutor_Brief_V1.md`：教学角色、学生首次实现、提示阶梯及安全红线。
- `Electrical_Control_Foundations_Curriculum.md`：基础学习顺序（本次限于考核必需内容）。
- `Electrical_Control_S1_2026-10_to_2027-01.md`：长期主线；不要把其他新技术塞入这次冲刺。
- **考核原件**：`十二期二面考核及部分三面说明.pdf`，控制/算法方向第 4–6 页、提交说明第 11、13–14 页。

工作模式：Work 负责读真实仓库、查接口、建立任务、编译与自动化测试、提出精确补丁、协助文档；用户负责第一次核心控制算法、真实接线、实机测试、标定及停止危险运动。紧急情况用户可显式授权 Work 实现某项非安全关键代码，不能默认代替用户验证。

## 1. 已知现状与需要核对的事实

- 主控：经典 Arduino Uno / ATmega328P，PlatformIO、CLion、Arduino Servo 库；硬件资源有限（2 KB SRAM）。
- 已有模块：`main.cpp`, `commands.cpp`, `arm.cpp/.h`, `motion.cpp/.h`, `kinematics.cpp/.h`; 正在开发 `joystick.cpp/.h`, `manual_control.cpp/.h`。
- 已验证过：二维 J1/J2 的 FK/IK、五个实测点的误差大致在数毫米量级（按用户历史证据）；二维 `RUNLINE` 能运动；20 ms 控制节拍、`WAYPOINT_DELAY_MS=10` 时约 20 ms 指令更新的日志。**不等于三维 IK、纸面直线度已通过。**
- 旧二维模型（待对照实际仓库）：L1=L2=80 mm；肩部高度约 38 mm；`alpha=184-J2`、`gamma=J1+J2-116.9`、`beta=alpha-(180-gamma)`，`x=L1 cos(alpha)+L2 cos(beta)`，`z=L1 sin(alpha)+L2 sin(beta)`。应保留其既有符号、单位和 FK/IK 接口，直到新模块通过回归。
- 过去保护规则：J1 ∈ [20,125]、J2 ∈ [40,130]、J1+J2>=148。J0 和夹爪限制、笔尖偏置、坐标轴标定以**现在真实机械臂**为准。
- 用户最新确认：机械臂与夹爪已调好；电路组的控制板预计 10 月 13 日到；要求完成 **所有**控制/算法任务并实现通用 CART 模式。
- 不清楚且禁止猜测：仓库远端 URL、当前 branch/HEAD、实际 `platformio.ini`、J0/J3 是否已开启、摇杆真实轴映射、电路板主控型号/UART 电压/串口引脚、笔尖相对模型原点的真实位置、HOME 实际安全关节角。
- 注意旧版 `arm.cpp` 里可能把 `held` 同时用于“停止处理 pending”和“人工 HOLD”，`setControlMode()` 却拒绝 `isArmHeld()`，这可能让模式切换永远被挡住。**必须先审查真实源码的状态语义，不要直接套聊天中的版本。**

## 2. 考核范围及逐项可交付行为

### 任务一
1. 四关节分别控制和实时摇杆操作：JOINT 模式可控 J0/J1/J2/J3。
2. PC 固定命令：`O` 开夹爪、`S` 闭夹爪、`H` 提升整体运动速度、`L` 降低整体运动速度；大小写或英文别名按文件许可，需在视频说明。
3. 解析单行 `x10,y30,z20` 样式的**三个舵机角度**命令，推荐映射 x=J0、y=J1、z=J2，必须三参数完整解析、整体安全验证后一次提交 Pose，不能逐关节输出形成不安全中间态。文中示例角度不保证安全，超限必须拒绝。
4. GitHub 仓库、有效开发提交历史。

### 任务二
PC 发 `A`/`B`/`C`，对三种不同物体各自完成抓取及放置，放置位置三个明显不同，空间 x/y 都要明显变化；应具有 approach→descend→grip→lift→transport→lower→release→retreat 预设阶段。真实物体种类严格参照机械组最终任务成果与考核文件。

### 任务三
电路组自制 Arduino 作为四键控制板，发串口事件给机械臂 Arduino：B1 首次 A、二次 B、第三次 C、循环；B2 首按开始录制（需持续 >10 s 且明显运动）、再按停止并保存；B3 播放上次动作；B4 回初始状态。硬件组完成自制板则必须用其板，不能无故换现成替代板。

### 任务四
正、逆运动学推导必须为**打字排版的 Word 文档**，不能用手写照片或整页公式截图替代；不仅是软件源码。

### 任务五
纸面连续直线；从 N、三角形、Z、V 中任选至少一种；绘制过程用按键暂停/继续/取消，暂停保存进度、继续续画、取消终止并返回待机；五个随机点摇杆示教、实体按键记录、先回待机再抵达 P1，然后画经过五点的四段折线；同样五点示教与经过五点的连续平滑曲线，均支持绘图控制，三面临时换点和连接顺序不得重新烧录。视频需完整展示设置目标、示教、存点、启动、自动绘制。

**注**：原文件标题任务三写 20%，条目内 B1/B3/B4 有多个百分比；不自行修正评分矛盾，以完整满足要求为目标。

## 3. 架构冻结：层、所有者与调用约束

设计约束：沿用 Arduino superloop，单线程协作状态机；不能在业务函数中使用 `delay()`；不能并行创建两个运动所有者；最终所有舵机输出只经过 `arm` 的安全检查入口；串口命令和四键事件共用应用事件分发。

| 文件（已存在/计划新增） | 唯一职责 | 严禁 |
|---|---|---|
| `joystick.h/.cpp` | 采 ADC、死区/档位/方向、返回标准化四轴指令（物理轴定义） | 直接操控关节、解析 MODE、调用 Servo |
| `manual_control.h/.cpp` | `ControlMode Joint/Cartesian`、时间积分、JOINT 和 XYZ 手动点动、模式切换时目标同步 | 自行持有另一个独立 Servo 状态 |
| `kinematics.h/.cpp` | 纯数学 FK2D/IK2D、FK3D/IK3D、校准结构及有效性检查 | 读串口、驱动舵机、管理任务 |
| `arm.h/.cpp` | 权威 Pose/关节 limits/Servo PWM、待执行目标、安全输出 | 解析串口、生成曲线、管理录制 |
| `motion.h/.cpp` | 所有自动行为的**唯一**任务所有者：队列/逐点推进/暂停/继续/取消/回家/错误态 | 直接读摇杆、创建无界动态数组 |
| `teaching.h/.cpp` | 纸面高度设置、5个点的 XY 存储、示教状态、MARK/LIST/CLEAR | Servo.write、重复另一套 XYZ jogging |
| `path_planner.h/.cpp`（可内嵌 `motion`） | 折线、图形和插值曲线的**纯采样函数**；不保存大体积点列 | 直接写 Servo、管理按钮 |
| `recording.h/.cpp` | 固定内存记录关节指令帧、录制时间、播放数据读取 | 回放时绕过 motion 直接写舵机 |
| `pick_place.h/.cpp`（也可作 motion 的数据表） | A/B/C 的抓放步骤配置及状态推进，位姿/夹爪常量集中管理 | 复制通用运动调度器 |
| `button_link.h/.cpp` | 解析外部板串口原始事件并输出 ButtonId | 决定当前按键业务含义、写舵机 |
| `app_controller.h/.cpp`（轻量） | 当前 Workflow、按键上下文分发、忙闲仲裁 | 重新实现各业务算法 |
| `commands.cpp` | PC UART 命令文本解析→调用 app controller / service | 持有当前运动状态/抄一套解析器 |
| `main.cpp` | 有限轮询：poll PC、poll 按键、输入、record tick、motion tick、apply output | 加阻塞长循环、临时状态重置 |

若真实仓库已有相同职责函数，**优先原地扩展，不为了表格强行创建十几个模块**。路径规划纯采样函数可留在 motion，不是必须抽成单独文件。

## 4. 状态的所有权（必须分离）

- `ControlMode { Joint, Cartesian }`：输入设备含义。JOINT：左摇杆 J0/J1，右摇杆 J2/J3；CART：左 XY、右 Z/J3。正常抓取可使用 CART。
- `WorkflowMode { Normal, TeachSetup, TeachMark, TeachReady }`：交互上下文。进入 TeachSetup 自动切 CART；标记阶段锁定 z=travelZ；退出后恢复原手动模式，但必须安全检查。
- `MotionState { Idle, Validating, Running, Paused, ... }` 与 `TaskKind { Move, Home, Pick, Line, Shape, Poly, Curve, Playback, ... }`：**自动运动唯一 owner**，不同自动任务互斥。
- `RecorderState { Empty, Recording, Ready }`：录制数据生命周期；播放是 Motion 的任务，而不创建第二个 Servo 输出线程。
- `ArmState`：有无 pending Pose、是否显式 HOLD、软件下发姿态；不要混用“姿态已执行”和“用户暂停”。

禁止：手动摇杆与自动运动任务同一时刻写不同 Pose；切换模式/进入示教时复用旧目标导致跳动；取消任务只清空计数器却没有执行考核要求的安全回 HOME。

## 5. 冻结接口草案（实际签名可按仓库小幅调整，必须记录差异）

```cpp
// joystick.h
struct JoystickInput { int8_t lx, ly, rx, ry; }; // -2,-1,0,+1,+2 档，参数可调
JoystickInput readJoysticks();

// manual_control.h
// ControlMode, getControlMode(), setControlMode() 已在开发，先审查真实版本
void updateManualControl(uint32_t nowMs); // 非任务和非 hold 时运行

// kinematics.h：保留现有二维 API
struct Point3D { float x, y, z; };
struct IKResult3D { bool valid; float j0, j1, j2; };
struct BaseCalibration { float servoAtWorldZeroDeg, sign, shoulderRadialOffsetMm, shoulderHeightMm; };
Point3D solveFK3D(float j0, float j1, float j2);
IKResult3D solveIK3D(const Point3D& target);

// teaching.h：示教平面存 XY，Z 用全局的 draw/travel 高度
struct TeachPoint { float x, y; };
struct TeachSettings { float drawZ, travelZ; bool drawValid, travelValid; };
bool beginTeach();
bool setTeachHeightFromCurrentPose(bool forDrawing); // 实际 FK3D 取笔尖坐标
bool beginMarking();
bool markTeachPoint();
void clearTeachPoints();
uint8_t teachPointCount();

// recording.h：容量以 SRAM 实测决定
bool startRecording(uint32_t nowMs);
bool stopRecording(uint32_t nowMs); // >10 秒、帧数有效
void updateRecording(uint32_t nowMs, const Pose& commanded);
bool hasRecording();

// motion.h：优先复用旧 startLine()/updateMotion()/pauseMotion() 等
bool startHome();
bool startPickPlace(uint8_t objectIndex); // 0/1/2 = A/B/C
bool startPolyline();
bool startCurve();
bool startBasicShape();
bool startPlayback();
bool cancelAndReturnHome();

// button_link.h / app_controller.h
// ButtonEvent 按键不附带业务，应用层按当前 Workflow/Task 分发
void onButtonPressed(uint8_t buttonNo); // 1..4
void pollButtonLink();
```

**代码样式只是建议接口，不代表已实现；当前 `IKResult` / `Pose` 等以仓库实际结构为准。** Work 应先汇总接口对照与冲突，不得一口气改写所有模块。

## 6. 关键行为与异常契约

### 6.1 通用手动模式
- 模式切换必须在无活动运动任务、摇杆回中且无未处理危险运动时完成。`Cartesian` 目标由**当前已下发**关节姿态 FK3D 初始化，避免切换瞬移；该姿态仍不是真实反馈。
- 摇杆输出可以使用 ±1/±2 档，但速度值属于 `manual_control` 的配置；JOINT 单位 deg/s，CART 单位 mm/s，不能直接沿用 45/90°/s 当作 CART 速度。
- 用经过的时间 dt（秒）积分；首次运行、模式切换、暂停恢复清理 dt 基准；对连续停顿设置最大 dt 防止跳变。无效 IK/碰撞/关节越界时拒绝候选点，保持最近合法目标。
- J0/J3 在已确认电源、机械限位/夹爪行程、零点和方向后才开启。三维 IK 的几何模型应由 J0 yaw + 已标定的二维模型构成，但必须核实肩部绕底座轴的偏置以及工具笔尖偏置；`r = sqrt(x²+y²)` 只在约定条件下直接适用。检查 `r≈0` 奇异和 IK 分支一致性。

### 6.2 录制 / 回放
- 录制**关节指令姿态**而非假装有真实角度反馈；只在手动模式录，记录所有四个关节及起止时间；超过 10s 有明显位移才算有考核证据。
- AVR SRAM 2KB：建议初始静态 `Frame{uint8_t joint[4]}` * 160 帧 = 640B，100ms 采样可覆盖最多约 16s；这是候选上限，Work 必须检查编译器 RAM 占用与 stack 余量，必要时降低容量/更改保存策略。不能使用无界 vector / String 拼接。
- 固定节拍采样用 `millis()`；遇到缓冲区满明确自动停止/报错，禁止覆盖内存。重放先以受限速度、安全路径回到第一帧，再按原始采样间隔重放；可做点间插值，但需经过安全约束和关节速度限制。

### 6.3 五点示教 / 绘图
- `TEACH START` 进入 Cartesian + Setup；手动确定 drawZ/travelZ（真实笔尖坐标），`TEACH MARKING` 锁 Z=travelZ，仅左摇杆 XY 定位；用户按实体键记录 5 个 `(x,y)`。需要 `MARK/CLEAR/LIST` 的 PC 命令等效测试入口。
- `DRAW POLY` / `DRAW CURVE`：HOME→抬笔安全移动 P1 上方→受控下降到 drawZ→沿五点轨迹→抬笔；自动绘图期间 `HOLD / GO / CANCEL` 有确定行为；取消后必须**受限轨迹返回 HOME**，不能突然瞬移。
- 折线每段端点依次通过 P1..P5。平滑曲线必须经过 P1..P5，而不是仅靠近控制点；可用 centripetal Catmull–Rom 或稳健三次插值，必须测试重复点、端点切线、曲线过冲、IK 可达性及 J0 连续性。
- 画线/图形用相同轨迹执行器；预检整条任务的每个离散路径点，实际发送的关节角步长还要考虑安全速度上限。命令一次只请求一个任务；后台增量校验，不大批动态分配。
- 纸面是否水平、笔尖接触力及刚度、末端笔尖几何偏置需要实际硬件标定；不能由软件编译推断绘图真实通过。

### 6.4 A/B/C 自动搬运
- 为每种物体保存独立抓取/放置姿态序列和夹爪开/闭角（以机械组现实测量为准）。需要 x/y 明显位移、三种不同最终位置、动作可重复。
- 使用与绘图相同 Motion Task owner，保留步骤和受控中止；抓放姿态不要猜测机械极限。

### 6.5 统一事件与四键映射（建议，等硬件到后确认）
- 传输层：PC `pollSerial()` 和控制板 `pollButtonLink()` 各自将输入解码为**同一种内部 Action/ButtonEvent**。测试支持 PC 输入 `BTN 1` 等模拟按键。未来硬件板发送建议短帧 `B1\n` … `B4\n`（最终格式须与电路组协议核对）。
- **Normal**：键1 A→B→C 抓放循环；键2 录制 toggle；键3 PLAY；键4 HOME。
- **TeachMark / TeachReady**：键1 MARK；键2 启动 POLY；键3 启动 CURVE；键4 清点/返回上级（须在测试时固定语义），也保留 PC 明确命令。
- **Drawing Running/Paused**：键1 HOLD，键2 GO，键3 CANCEL 并受控 HOME，键4无操作或按协议定义；三动作必须为不同按键。
- 教学配置阶段 `TEACH DRAWZ` / `TEACH TRAVELZ` 通过 PC 命令捕获当前姿态；若现场禁止使用 PC，则 Work 与用户提前决定物理按钮长按或短按组合方案。**不可默认考官允许 PC 键盘配置。**
- 控制板 UART 连接、5V/3.3V逻辑电平、共地、串口速率、RX/TX 针脚、是否同时需要 Uno USB 串口均待实体板验证；Uno D0/D1 和 USB 串口共用硬件 UART，不能默认与 PC 和第二板同时独立收发。软件 UART 可能和 Servo/中断相互影响，必须实测后选型。

## 7. 单一状态机运行规则（Work 必须核验）

1. 任何活动 Motion Task 占用输出所有权；此时手动控制不调用 `requestPose()`，录制启动拒绝，新的自动任务拒绝或按明确取消策略处理。
2. HOLD/PAUSE 停止后续轨迹指令，`GO` 从任务步骤和进度恢复，不突然从头开始；应用层操作不改变已设的示教点。
3. `CANCEL`：绘图语义必须取消绘图并回待机，若回待机不安全则明确失败并保持当前安全命令姿态；不能以软件“已安全回家”谎报。
4. `HOME`：统一使用已物理验收的安全初始 Pose 和**渐进**运动；禁止将 Servo 直接瞬时写 90° 作为回中。
5. 姿态目标必须先通过 NaN/范围、组合限位、三维 IK 分支、路径采样的边界检查；数值错误不触发舵机动作。
6. `DONE` 仅指所有软件命令已下发；需留出落位等待时间，不能声称舵机有实际位置反馈。
7. 对外输出统一状态：控制模式、工作模式、任务种类/阶段、进度、错误原因、录制时长/帧数、五点数量、已确认的校准有效性。

## 8. 命令协议提案（大小写兼容按考核允许范围）

- 保留：`IK x z`、`XY x z`、`LINE x z`、`RUNLINE x z`、`J1 angle`、`J2 angle`、`HOLD`、`GO`、`CANCEL`、`STATUS`、`HELP`、现有 `SPEED`（调整为真正生效或保留别名）。
- `MODE JOINT|CART`；`XYZ x y z`（空间 mm）；`IK3 x y z`（仅查询）；`J0 angle`、`J3 angle`（标定/限位正确后启用）。
- `O`, `S`, `H`, `L`；`x90,y85,z95`（三个**舵机角度**，与 XYZ 严格区分）。
- `A`, `B`, `C`; `REC`（toggle，也可 `REC START|STOP`）, `PLAY`, `HOME`。
- `TEACH START`, `TEACH DRAWZ`, `TEACH TRAVELZ`, `TEACH MARKING`, `MARK`, `LIST`, `CLEAR`, `TEACH EXIT`。
- `DRAW LINE`, `DRAW SHAPE N`（也可选 TRIANGLE/Z/V）, `DRAW POLY`, `DRAW CURVE`。
- `BTN 1|2|3|4`：仅作为无板联调的事件模拟入口，必须调用和外部四键相同的分发逻辑。
- 所有命令明确成功/失败和 Busy/invalid/unreachable/full/un-calibrated 等错误；允许现场改五点和顺序，禁止靠硬编码五点后烧录。

## 9. 开发顺序（保证所有任务 + 两天缓冲）

| 日期 | 主功能 | 最小可观测证据 |
|---|---|---|
| 10/12 | Work 读仓库；状态/接口审计；命令全覆盖；模式与 joystick 控制初版；单元/解析测试骨架 | build、git checkpoint、命令测试表 |
| 10/13 | 四轴 JOINT/CART 完成首次实机验证；外部按钮板联调；录制≥10s、回放、HOME | 操控视频、按钮事件日志、录制/回放视频 |
| 10/14 | 三维 FK/IK + J0/笔尖标定；五点存储和示教；A/B/C 抓放参数联调 | FK/IK 对照表、五点 MARK 日志、至少一种抓放 |
| 10/15 | A/B/C 三类全跑通；五点折线；纸面直线/基本图形；HOLD/GO/CANCEL 回 HOME | 三种物体视频、实际纸面轨迹和暂停视频 |
| 10/16 | 五点平滑曲线、长时间重复运行、异常处理；冻结最终演示固件 | 完整曲线示教视频、稳定 tag / release |
| 10/17 | 视频补拍、三面模拟（随机点、不烧录）、任务四 Word 推导、说明文档、个人报告、小组汇总 | 可提交材料齐全 |
| 10/18 | 回归/复查/提交；建议 18:00 前首次完整上传 | 网址、文件清单、版本、备份、提交凭证 |

以上任务可并行拆分：用户优先核心运动/硬件；Work 做代码审查、命令 parser、测试、自动文档、架构迭代方案；电路组按键固件和机械组搬运点位需同步对接。**不因为所有功能都要求完成就假设风险不存在。** 10/14–16 进度若延误，优先保留可运行版本和证据，未成功必须如实标注。

## 10. Work 的第一轮任务：先审计再改代码

请先完成这些明确输出，不要直接进行全项目重写：

1. 读取 GitHub 仓库 **当前默认/指定工作分支** 的完整源码、README、`platformio.ini`、git status/log，建立文件及函数索引。
2. 表格列出本文件各接口在仓库中的真实存在位置、是否已实现、类型不一致、是否与当前编译冲突；尤其审计 MODE 幂等、`isArmHeld()` 和 `posePending` 状态逻辑。
3. 单独列出目前能 `pio run -e uno` 的版本及编译输出；不要声称未连接的实机测试已通过。
4. 检查 Uno SRAM 及引脚资源图：Servo D6/D7/D8/D9、ADC A0/A1/A2/A3、串口 D0/D1、可能的电路组 UART 口。实际资源以仓库和实物为准。
5. 形成最小 PR/patch 分组与先后依赖：① parser+EVENT ② manual JOINT/CART ③ button link ④ record/playback ⑤ IK3/calibration ⑥ pick/place ⑦ teach/poly/curve ⑧ docs/video checklist。每组具备 build+回归测试，不能把所有功能塞到一个大 commit。
6. Work 可以先完善**无舵机输出的可自动测试模块**、接口骨架及文档；任何 J0 舵机映射、外部供电或写入新运动行为必须等待用户批准和实机标定。第一次核心实现留给用户，除非用户明确逐项授权代写。
7. 输出给用户：当天最小可执行动作（10–45分钟）、所需材料、通过标准、应保存的证据。每次只给一件马上做的任务。

## 11. Work 请求用户提供的资料（不要请求密码或令牌）

必需：
- **GitHub 仓库 URL、当前 branch、最新 commit hash**，并授予 Work 可读取该仓库的访问权限（私有仓库需要在连接器里授权）；或者直接上传完整工程 ZIP（包含 `platformio.ini`, `src/`, `include/`, `lib/`, `test/`），不要只贴聊天中的旧函数。
- 本考核 PDF、Tutor Brief、Curriculum、S1 四份文件，若 Work 所在新会话不能自动访问本 Project 则必须重新附加。
- 今日当前能正常上传并运行的版本、J0/J1/J2/J3 引脚表、可动安全范围、夹爪开/合及 HOME 实测值、舵机外部电源数据。
- 明日四键板：实物/原理图、主控型号、UART TX/RX 引脚、逻辑电平、波特率、按键报文协议；暂时未知可以标“待到货”。
- 五点绘图：夹笔方向、笔尖/肩部/底座旋转轴的参考位置、纸面高度、已测量的连杆尺寸、可达且无碰撞的绘图区范围。
- 三个待抓取物体及放置位置的照片/尺寸；谁负责物体摆放与夹爪机械调试。

推荐：当前 `git diff`、已有串口运行日志/视频、CAD/SolidWorks 可测关键尺寸、工具链版本、已知异常（卡滞、摩擦、发热）、10/13 电路板可到场时间、你每天可投入的净工时。

## 12. Work 需要协助准备的提交材料

- 机械臂 Arduino 工程；ESP8266 工程（考核提交文本如此写，但当前副板据说是自制 Arduino，**必须跟师兄确认具体是否另有 ESP8266 要求**，不得捏造 ESP8266 代码）；必要的按键板固件。
- 算法任务一/三/五实际演示视频；五点演示必须包含目标设置→摇杆示教→按键记录→自动绘制，三面临时点无需烧录。
- Word 打字排版的正逆运动学公式推导；说明文档（各功能模块、示教、轨迹、速度、异常处理）；个人报告；GitHub 仓库网址 txt 和开发提交历史。
- 小组组长：整机照、各方向调试视频、机械组装照片、分工/完成情况表、最佳整体视频及提交清单。

---

## 可直接粘贴给 Work 的启动指令

> 请接手我 2026-10-18 23:59 截止的 MeArm 控制/算法二面考核的软件工程协作。我的目标是 **全部任务完成，包括 JOINT 和通用 CART 双摇杆控制、O/S/H/L、三舵机角度指令、A/B/C 自动抓放、四键串口联动、超过10秒录制与完整回放、HOME、三维 FK/IK、实际纸面直线/基础图形、暂停/继续/取消回中、现场随机五点示教折线和平滑曲线，无需重新烧录**。机械臂与夹爪目前已调试好，电路组板子预计 10 月 13 日到。
>
> 请首先阅读随附的本《MeArm_Control_Work_Handoff_2026-10-12.md》、考核 PDF、Tutor Brief、Curriculum、S1；**第一轮不要直接改仓库**，先读取我分享的真实 GitHub 仓库/工程 ZIP，确认当前分支、代码和硬件条件，提交一份差异审计、依赖图、RAM/引脚风险检查、精确接口冻结表、能实施的最小 PR 顺序以及首个 30–45 分钟任务。要以考核原文为验收准则，以真实代码/实测优先于聊天假设。
>
> 这也是我的嵌入式学习项目。我负责核心函数首次实现、硬件接线、实测与初始排错，你负责设计契约、函数骨架、协助跨文件实现、构建/测试、文档和 Git 流程；对非关键重复代码或紧急实现可以在征得我同意后代写。不得编造运行成功，不得直接启用未经标定的 J0/J3，不得绕过安全 Pose 检查。遵守一次一个小范围任务、明确时间/验收/证据的合作方式。
>
> 为保证余量，目标 10/16 晚功能冻结、10/17 视频文档与三面演练、10/18 提交。请现在开始第一轮真实仓库审计，并列出缺失的具体信息。
