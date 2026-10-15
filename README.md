# MeArm Bringup：Fast70 交互补丁（2026-10-15）

本工程保留已录入的 Bringup 标定，完整 D10 目标为 **`uno_nano_five_fast`**。
当前补丁：TEACH MARKING 自动定位、DRAW 自动准备 J3=85、S=85、带命令上下文的 ACK、短错误码；HELP 已删除，FEED 不恢复。
本轮只通过软件构建/模拟回归，未烧录、未操作机械臂。软件角度、FK、READY 和 DONE 都不是物理位置反馈。

详细提交、逐项资源数据、日志和实机复测见 [交付报告](docs/Interaction_Patch_2026-10-15.md)。旧 docs 是历史证据；与本页冲突时应核对当前源码，不沿用旧命令说明。

## 构建与回归

```powershell
pio run -e uno_nano_five_fast
& ./test/native/run.ps1 -Compiler C:/msys64/ucrt64/bin/g++.exe -ExtraFlags '-DMEARM_FIVE_FAST','-DMEARM_POLY_HOLD_MS=80'
& ./test/native/run.ps1 -Compiler C:/msys64/ucrt64/bin/g++.exe -Suite interaction_tests.cpp -ExtraFlags '-DMEARM_FIVE_FAST','-DMEARM_POLY_HOLD_MS=80'
& ./test/draw_t3/run.ps1 -Suite interaction_draw_tests.cpp -ExtraFlags '-DMEARM_FIVE_FAST','-DMEARM_POLY_HOLD_MS=80'
& ./test/nano_link/run.ps1
```

`uno` 不含当前 D10 链路，不能代替完整目标容量验收。`uno_nano_link` 不启用五点 Fast 实验策略。Native 的 `test_main.cpp` 是历史旧夹具，当前主回归入口为 `sprint_tests.cpp`。

当前完整目标 Flash **32254/32256 B（剩2 B）**，静态 SRAM **1295/2048 B**。SRAM 的753 B差额还需容纳栈和中断上下文，不等于已测栈安全。不要把 `.pio/synthetic-*` 的合成标定产物用于实机。

## 连接与时间参数（本补丁不修改）

| 连接 | 用途 |
|---|---|
| Nano D1/TX → Uno D10 | 9600 baud 单向按键 UART |
| Nano GND → Uno GND | 共地 |
| Uno 5V → Nano 5V | Nano 未连接 USB 时使用既定供电方案 |
| Uno USB | PC 115200 baud 命令与反馈 |
| Uno D9/D8/D7/D6 | J0/J1/J2/J3 Servo |
| Uno A0/A1/A3/A2 | LX/LY/RY/RX 摇杆；CART 为 X/Y/Z/J3 |

Nano 原有四键与 OLED 固件、接线不变。D10 接收使用 PB2/PCINT0 和 Timer2；Servo 使用 Timer1。主控没有增加 OLED 驱动或返回链路，不向 Nano D0 接线。原有烧录/外部供电互斥、烧录断开板间线等电气约束继续遵守。

D10 保持8字节二进制帧：`A5 type epochLo epochHi sequence button CRC8 5A`；Hello=0、Press=1；CRC覆盖前6字节，多项式0x07。保留会话同步、序号、重复过滤、40ms残帧超时和溢出恢复。不是 `B1\n` 文本协议。

- 控制周期20ms；Servo库负责脉冲；RUNLINE最小等待10ms不是PWM周期。
- 手动 JOINT 25/90°/s，CART 25/100mm/s；H/L只改变手动倍率，范围0.75..1.5，每次0.25。
- LINE/SHAPE描线35mm/s；五点POLY/CURVE描线70mm/s；落笔/抬笔空间参数25mm/s。数值是规划目标，非实测速度。
- Fast只对POLY/CURVE的STROKE绕过额外60°/s关节规划；其余阶段照旧。POLY三个中间点保持80ms，CURVE无此停顿。
- SPEED默认60、范围5..90°/s，作用于原有普通目标移动/TEACH自动定位等；HOME、DRAW及PICK分阶段策略独立，回放也不随H/L或SPEED变化。
- 原限位、148..250°组合约束、量化输出复核、IK/路径预检不变。软件标定有效位并不证明现场笔尖/工装已经校准。

## PC 命令速查

115200 baud，每行以换行结束；接受CRLF、空格/Tab。命令区分大小写，仅普通十进制数（不接受nan/inf/指数）。最长63字符，坏字节/超长行丢弃到换行。每轮最多处理64字节和一个命令。STATUS等同步串口打印仍会消耗时间，运动中不要持续刷屏。

| 命令 | 行为 |
|---|---|
| `STATUS` | 当前指令角、模式、任务、HOLD、错误、录制、点集和高度；不是物理位置反馈 |
| `MODE JOINT` / `MODE CART` | 保留原有模式切换回中/空闲条件；CART需要标定 |
| `J0 90` / `J1 120` / `J2 60` / `J3 85` | 单轴目标，经统一任务和完整Pose检查 |
| `x90,y120,z60` | 原子三轴角度指令：x/y/z=J0/J1/J2，保留J3 |
| `O` / `S` | O保持配置开爪40°；S目标85°；没有CLOSE命令 |
| `H` / `L` | 仅手动摇杆倍率增加/减小 |
| `SPEED 60` | 普通自动角速度配置；忙时拒绝；不改变固定绘图速度 |
| `IK x z` | 二维IK查询，不运动 |
| `XY x z` | 旧二维肩肘目标移动（命名XY，实际是径向x/高度z） |
| `LINE x z` / `RUNLINE x z` | 二维路径仅预检 / 预检并运动，4mm几何采样 |
| `IK3 x y z` / `XYZ x y z` | 三维工具点IK查询 / 目标移动 |
| `HOLD` / `GO` | 暂停/继续；保留任务进度，不积累暂停时间 |
| `CANCEL` | 通常停止并HOLD；DRAW含夹爪准备阶段则按原逻辑抬笔、受控HOME |
| `HOME` | 取消当前任务，预检并执行HOME；绘图取消恢复保留抬笔 |
| `A` / `B` / `C` | 执行对应的完整八阶段配置，先预检；示教中拒绝 |
| `REC` / `REC START` / `REC STOP` | 录制切换/开始/结束保存；ARMING中停止为取消 |
| `PLAY` | 预检、受限返回首帧、按时间轴回放 |
| `BTN 1`..`BTN 4` | 与实体四键共用业务分发 |
| `TEACH START` | CART示教设置；不清点集/高度；摇杆J3冻结 |
| `TEACH DRAWZ` / `TEACH TEACHZ` / `TEACH TRAVELZ` | 捕获当前已接受指令姿态的FK高度，分别为描线/对点/移动高度 |
| `TEACH MARKING` | 验证高度，等待回中，再保持当前XY自动到teachZ；READY后才可MARK |
| `MARK` / `UNDO` / `LIST` | 记录点 / 撤销最后记录的点（不是ORDER末尾点）/ 列出当前顺序 |
| `ORDER 1 3 2 5 4` | 必须是全部五点的无重复排列 |
| `CLEAR` | 清点集，返回设置；保留纸面高度 |
| `TEACH EXIT` | 退出，保留点/高度；若自动定位正在等待/移动，取消并HOLD |
| `DRAW LINE x1 y1 x2 y2` | 自动准备夹爪、HOME/移动/落笔/描线/抬笔/返回 |
| `DRAW SHAPE N x0 y0 x1 y1` | 边界矩形内N；还支持TRIANGLE/Z/V；宽高均需≥5mm |
| `DRAW SHAPE N` | 用前两个顺序点形成边界矩形，退化仍拒绝 |
| `DRAW POLY` / `DRAW CURVE` | 按ORDER经过五点的折线 / 向心Catmull–Rom曲线 |
| `PICK SET A 1` | 捕获当前已接受指令Pose为A的第1阶段；A/B/C各自独立 |
| `PICK SHOW A` | mask和各阶段J0 J1 J2 J3、速度、等待ms |
| `PICK SPEED A 1 60` / `PICK WAIT A 1 100` | 已记录阶段的5..90°/s速度 / 0..2550ms等待（10ms整数倍） |
| `PICK CLEAR A` | 清对应工作配置，不自动SAVE |
| `PICK SAVE` / `PICK LOAD` | 显式持久化 / 恢复有效快照；保持HOLD，完成后需GO |

`HELP`、`FEED` 已不提供，返回 `ERR E05`。未新增裸坐标、PAUSE、RESUME、CLOSE等命令别名；暂停/继续用HOLD/GO。

### ACK、任务完成和错误

普通成功变更命令返回 `OK <命令及参数>`，例如 `OK MODE CART`、`OK S`、`OK DRAW POLY`。直接复用原串口缓冲，解析器NUL分隔符还原为空格；多余空白可能保留，角度三元组的逗号会变为空格。上位机应按token解析，不再只匹配精确的 `OK`。

STATUS、IK、IK3查询保留数据输出；LIST/PICK SHOW输出数据后还有上下文OK。实体按钮仍输出 `OK BTN`，PC模拟输出如 `OK BTN 2`，Nano二进制协议不变。

异步任务的 `OK` 只表示接受；之后可能预检失败。成功完成输出 `DONE DRAW POLY` / `DONE DRAW CURVE` / `DONE DRAW LINE` / `DONE DRAW SHAPE` / `DONE HOME` / `DONE PLAY` 等。所有普通关节目标（含S/O/角度三元组）共用 `DONE MOVE`；ABC共用 `DONE PICK`，没有额外保存原始输入。`LINE`仅预检完成为 `DONE LINE`。失败/取消不输出旧任务的DONE。取消DRAW后可能输出 `DONE HOME`。

TEACH MARKING当前输出顺序：

```text
TEACH WAIT
OK TEACH MARKING
TEACH MOVING
PATH VALID
TEACH READY
```

WAIT等待四轴回中；MOVING表示已开始自动定位流程（包含尚待完成的预检），PATH VALID之后才运行输出。READY表示该任务的指令序列已提交完成，不是测得物理到位。

PICK SAVE完成保留 `PICK SAVED; HELD`；录制保留 `REC ...` 生命周期。回放若安全预检要求时间轴整体放慢，输出 `PLAY x=<倍率>`；该字段只是原 `PLAY timeScale=` 的短标签，时间策略未改。

### 错误码速查

输出只有 `ERR <短码>`；STATUS.error保留内部枚举编号。PATH ERR的 `waypoint/substep` 是失败采样位置，不是另一个原因码。不能仅凭内部错误号判断物理故障。

| 原因码 | STATUS.error | 触发条件 | 操作者处理 |
|---|---:|---|---|
| E01 | 1 | 任务/录制/保存/待输出占用，或原有MODE回中/示教互斥条件不满足 | STATUS定位状态；等完成、回中，或明确取消/退出后重试 |
| E02 | 2 | HOLD状态不允许启动新任务 | 查明停止原因；安全后GO或受控HOME |
| E03 | 3 | 所需标定/高度/轴使能/HOME配置无效 | 核对真实配置和STATUS；不通过扩大限位解决 |
| E04 | 4 | 命令轴禁用 | 核对轴配置 |
| E05 | 5 | 通用非法输入/当前条件不满足；无法细分的错误 | 查语法、参数、重复示教点等；不要推断成IK失败 |
| E06 | 6 | 完整Pose或路径采样未通过输出合法性检查 | 记录PATH ERR/STATUS与输入，修改目标/路径；不取消安全检查 |
| E07 | 7 | IK无法给出有效解（也可能涉及标定/模型或安全条件） | 核对坐标、高度及标定，选择有效路径 |
| E08 | 8 | 点/路径容量上限 | 检查已有点，按需UNDO/CLEAR；不要默默丢点 |
| E09 | 9 | 无录制、点不足、无有效EEPROM等数据缺失 | 按当前命令补数据；PICK LOAD失败后保持HOLD |
| E10 | 10 | 图形边界矩形退化（宽或高<5mm） | 指定有效矩形或调整首两点 |
| T01 | 11 | MARKING所需高度配置无效；要求drawZ<teachZ<=travelZ且相关标定有效 | 在TEACH START设置中重新捕获三高度，STATUS核对 |
| T02 | 12 | 不在示教设置中捕获高度，或未开始示教即请求MARKING | TEACH START；等待现有任务完成；已标点时先回设置 |
| T03 | 13 | 空闲但未进入TeachMark时MARK | TEACH MARKING并等READY；等待/移动期间因占用返回E01 |
| D01 | 14 | DRAW的HOME/via夹爪不为85，或初始夹爪准备Pose预检失败 | 核对夹爪及完整Pose约束；自动准备不能绕过配置检查 |
| P01 | 15 | A/B/C对应八阶段mask不完整 | PICK SHOW，补齐缺失阶段并SAVE |
| P02 | 16 | 已有阶段但采样时速度/夹爪阶段语义等不合法 | 核对PICK SHOW的八阶段速度与开/夹/放角；重新示教对应阶段 |

STATUS.error=0表示无错误；若内部未知编号被打印则通用兜底E05，不伪造精确原因。E03/E06/E07也可能先于专用码发生；专用码仅用于源码能确定的分支。历史error可在成功查询后仍保留，不能当作当前活动任务必然失败。

## TEACH/DRAW 交互契约

三高度均来自**已接受的整数关节指令的FK**，不是摇杆积分目标或传感器实测。进入TEACH START仍遵守原模式切换条件。Setup锁摇杆J3；Mark锁摇杆Z和J3，保持固定teachZ；显式PC J3仍允许安全调试。

MARKING接受后由唯一MotionTask占用运动权。未回中只等待，不累积手动运动、不返回INVALID。回中时取得当前XY，设目标Z=teachZ，走原Cartesian分段、安全预检和输出；预检失败则HOLD，不设READY，保留五点和高度。等待/移动期间TEACH EXIT、B4、CANCEL、HOME会取消请求，之后GO不会复活请求。普通CANCEL保持示教设置；TEACH EXIT/B4同时退出。EXIT取消定位后要先检查状态再GO。HOLD/GO是暂停继续，不是取消。

DRAW在J3不为85时，先规划仅改变J3的非阻塞准备段，再走原绘图阶段；**准备段及整个DRAW一起预检，通过前不会先关爪**。若原来就是85，没有额外准备段。准备段按原60°/s关节策略、20ms输出节拍执行。HOLD/GO保留同一任务；CANCEL/B3沿用原绘图抬笔并受控HOME，恢复路线失败则保持停止。无任何“笔已实际夹牢”的自动判定。HOME/via仍须保持85，ABC独立八阶段角度不被改为85。

## 四键状态表

| 上下文 | B1 | B2 | B3 | B4 |
|---|---|---|---|---|
| 普通空闲 | A/B/C接受成功后循环 | 录制切换 | 回放 | HOME |
| ARMING/录制 | 拒绝 | 取消准备/保存录制 | 拒绝 | 清录制并HOME |
| TeachSetup | 捕获drawZ | 捕获travelZ | MARKING | 退出 |
| MARKING等待/自动定位（含暂停） | 拒绝 | 拒绝 | 拒绝 | 取消定位并退出/HOLD |
| TeachMark空闲 | MARK | POLY | CURVE | UNDO |
| DRAW含预检、夹爪准备及暂停 | HOLD | GO | 取消、抬笔、HOME | 无动作 |
| 普通其他运动任务 | 拒绝 | 拒绝 | 拒绝 | HOME（示教上下文另受原互斥限制） |

TEACHZ用PC `TEACH TEACHZ` 捕获。点集跨TEACH START/EXIT、DRAW保留；CLEAR才清点，断电仍丢失RAM点集和高度。

## 录制与EEPROM

录制仍为80×6 B关键帧（四角度字节+uint16相对ms），约20ms观察已接受指令，固定容量有界重压缩；3秒ARMING不计时，10秒提醒，20秒自动保存，也可B2主动保存。复杂动作允许DEGRADED并报告误差界，不以平均压缩率保证精度。回放先预定位首帧，再按时间轴重建；必要的既有安全时间倍率独立于H/L/SPEED。短录制可显示SHORT，不代表官方时长验收已通过。

PICK阶段1..8分别为APPROACH、SRC、GRIP、LIFT、TRANSFER、DST、RELEASE、RETREAT（数字和名称均接受）。SRC=2、DST=6，不能只录这两点代替安全八阶段。阶段1/2同开爪角，3..6同夹持角，7/8同释放角；夹持必须区别于开和放。捕获重设该阶段速度60和等待100ms，必要时随后重新设置。

配置步骤：在安全可达位置用串口/摇杆移动 → `PICK SET A 1`，依次完成八阶段 → `PICK SHOW A`检查mask=255和每个姿态、速度、等待 → `PICK SAVE` → 等`PICK SAVED; HELD` → 安全确认后`GO`。B/C同理。运行时先预检，不具备碰撞传感器或自动推导安全工装路径。

EEPROM为两个155 B提交槽+147 B编辑区，总457 B地址占用；版本1、CRC、最后提交标志保留。重启仅恢复有效提交快照；未SAVE编辑不视为有效恢复点。空白/损坏/版本不符不运行。LOAD覆盖工作编辑区，SAVE/LOAD不自动运动。

## STATUS 字段

phase：0空闲、1预检、2执行、3暂停、4等待回中。task：0无、1 LINE、2 RUNLINE、3关节、4 Cartesian、5 HOME、6 PICK、7 PLAY、8 DRAW LINE、9 SHAPE、10 POLY、11 CURVE、12 TEACH自动定位。空闲可保留最后task和完成计数。

workflow：0普通、1设置、2标点。REC：0空、1 ARMING、2录制、3已保存。Z依次drawZ/teachZ/travelZ，末尾有效位1/2/4；mask=7不保证高度顺序有效。paper仅表示draw/travel基本有效，不代替MARKING的三高度检查。enabled为轴位图，当前配置四轴应为15。

固件不再打印开机HELP或横幅；连接后主动发送STATUS。当前源码硬件配置已录入有效位，但部分旧注释未更新；本补丁没有改动任何物理参数。所有新行为仍待操作者实机验收。
