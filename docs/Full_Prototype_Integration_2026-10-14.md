> 历史阶段记录：其中 FEED 调参步骤已退役；当前固件固定35mm/s描线，见 [本轮接口与测试说明](TEACH_AXIS_FIXED35_2026-10-14.md)。下文保留用于追溯当时的源码与构建证据。

# MeArm 全功能原型集成交付 — 2026-10-14

本阶段已完成 Uno 软件功能集成与回归；没有上传、连接串口设备或驱动实体机械臂。软件通过不等于纸面精度、抓放成功率、UART 时序或栈峰值已实机通过。停止于本阶段，不开展最终性能优化。

## 版本与保护

- 工程：D:/Develop/MeArm-Bringup-2026-10-12
- 集成分支：codex/full-prototype-integration-2026-10-14
- 完整干净基线：09a7d5c98767e22b770bc173bcfc16ee938e8b64；保留于 bringup-2026-10-12 和 baseline/bringup-2026-10-14-09a7d5c。
- 参考开发分支 HEAD：e76bf95d869588739b9979ce5f805ed30fb53789。未更改其工作区；nano/platformio.ini 的用户修改 nanoatmega328new、未跟踪 Word、nano/include.zip、两个 Word 工具均保留。
- e4630669fc678f0f3369dca2a3df86eea10bdb1b：八阶段运行时抓放、EEPROM、任务互斥及对应回归。
- 9cfedabd1ad56d07ade9cb34afa798109d2c230a：TEACHZ、实体撤销、独立绘图进给、录制状态提示。
- 19738c69f00c1b60ff0c974089c9677ec6389d87：重复 HOME 保留绘图恢复抬笔；任意顺序、曲线过点、错误恢复的集成测试。以下构建/测试绑定该代码提交；后续交付提交仅保存文档和证据。

已阅读官方 PDF 算法任务一至五（第4–6页）、10月12架构交接、V2及V2.1规范、10月13 D10报告，并检查两套 Git 状态。运行时 PICK 遵循用户后续明确授权的八阶段配置要求；工装只需示教并 SAVE 一次，重启恢复，不要求每次演示重新示教。

保护核验见 integration_evidence_2026-10-14/manifest.json：
arm、arm.h、config.h、joystick、kinematics、path_planner、D10驱动、ButtonWire协议、button_link 与基线逐字一致。hardware_config 只增加生产 const 限定，整个实测初始化值一致。manual_control 只将标记阶段 Z 锁定由 travelZ 改为显式 teachZ，XY/Z 方向和速度不变。不是用旧文件覆盖新标定。

当前真实配置：J1 20..125°，J2 40..150°，组合限位 **148..250°**（源码标注临时上界，并非旧聊天中的255°）；J0/J3 有效、范围0..180°；启动指令80/120/60/50，HOME 90/120/60/85；夹爪 O=40、S=90。FixedPlanar，yawZero79.67、sign+1、肩偏置14.75/63、工具偏置53/-4.65/-6；HOME无中间点。原注释中“未标定/暂不启用”与有效位存在历史不一致，本轮按真实数值保留，不擅自调整。

## 官方功能矩阵

所有“通过”均为软件构建/电脑替身测试；具体物体、纸张、几何及接线仍须现场验收。

| 项目 | 实现文件/入口 | 测试及关键断言 | 当前结果 |
|---|---|---|---|
| 任务一：四轴/双摇杆 JOINT、额外 CART | arm::requestPose/prepareOutputPose；manual_control::updateManualControl；joystick::readJoysticks | testSafety：非法/量化后组合越界零输出；testHoldAndJoystick、test3DAndManual、testManualFrameOwnership：当前映射、积分、互斥 | 保留；软件通过，实机标定未改变 |
| 任务一：O/S/H/L、三角度原子命令、PC状态 | commands::executeLine/angleTriple/pollSerial | testParser、testIndependentSpeedAndClock：严格解析、完整Pose、速度隔离、STATUS公平性 | 通过；H/L只作用手动，自动任务按独立策略 |
| 任务一：Git历史 | 独立基线/集成分支与3个功能提交 | manifest源码hash、changes.patch | 已保存；未推送远端 |
| 任务二：A/B/C 三物体抓放 | pick_place::executePickCommand/getPickStage；motion::startPickPlace/sampleGoal/validateOne | testHomeAndPick、testRuntimePickPersistence：SRC/DST不完整拒绝、对象隔离、夹持阶段一致、最后阶段非法整条零输出 | 已集成；各物体实际八阶段姿态/速度待示教 |
| 任务三：B1 A→B→C循环 | app_controller::onButtonPressed/dispatchAction | testHomeAndPick：忙时拒绝、成功接受后递增；错误不排队 | 通过，机械落点及位移待验收 |
| 任务三：B2准备/录制/保存 | recording::startRecording/updateRecording/stopRecording | testAdaptiveRecording、testRecordingAndButtons：3秒不计时、15/20秒复杂动作、停顿/反向/四轴、时间回绕、首尾时间 | 保留并补 SHORT/OVERWRITE 提示；80×6B，最长期限20秒 |
| 任务三：B3完整回放 | motion::startPlayback/executePlayback/pauseMotion/resumeMotion | 同上及 testMotionPayloadTransitions：先回首帧，暂停不丢逻辑时间，播放至末帧，限速时报告timeScale | 通过；真实复现精度待测 |
| 任务三：B4 HOME | motion::cancelAndReturnHome/startHome | testHomeAndPick、testFrozenTaskSpeeds、testIntegrationContracts：固定速度、未标定拒绝、重复HOME保留抬笔 | 通过；HOME及进入路径无障碍仍须现场确认 |
| 任务三：实体板UART | button_link::feedButtonByte/pollButtonLink；pin_uart PCINT0/Timer2 | D10寄存器3273断言；真实Uno PC循环35断言：CRC、重复、epoch、断帧、溢出、Hello、USB STATUS | 驱动/协议原样保留；本轮未做实体联调 |
| 任务四：运动学推导 | kinematics::solveFK/solveIK/solveFK3D/inverse3D | testMath、test3DAndManual：正逆解往返、非法输入 | 算法保留；官方另需可编辑Word，已有开发目录未提交Word本轮未审核/改写，不能以固件替代材料 |
| 任务五：自动直线、基础图形 | motion::startDrawLine/startBasicShape；path_planner::shapeVertex | testTeachAndDraw：直线、N/三角形/Z/V都运行完；非法远点保持拒绝 | 通过；保留RUNLINE；纸面轨迹待实机 |
| 任务五：五点/任意顺序/折线 | teaching::markTeachPoint/undoTeachPoint/setTeachOrder；motion::startPolyline | testTeachAndDraw、testIntegrationContracts：满5拒绝第6点，排列重复拒绝，3-1-5-2-4不改存点，完成/退出后复用 | 已集成，通过 |
| 任务五：平滑过五点曲线 | path_planner::centripetalSample；motion::curveKnots/drawGoal | testCurveSampler、testCurveIndependentReference：独立向心参考、端点/重复点；testIntegrationContracts：重排后实际输出指令对应FK经过5点，模拟误差<3mm | 向心Catmull–Rom通过；真实笔迹精度/速度平稳性待测 |
| 任务五：三键暂停/继续/取消 | onButtonPressed → pauseMotion/resumeMotion/cancelAndReturnHome | testIntegrationContracts：运行中暂停1.2秒无新输出、继续完成、取消抬笔HOME、重复HOME不掉抬笔、保留点集 | 通过 |
| V2.1：PC优先、近纸面对准 | TEACH DRAWZ/TEACHZ/TRAVELZ、MARKING，STATUS/HELP | 未设TEACHZ拒绝进入；drawZ<teachZ<=travelZ；当前高度/摇杆中位检查 | 已集成；纸面三高度须由用户捕获 |
| V2补充：EEPROM断写恢复 | pick_place::savePickConfiguration/updatePickPersistence/restore | 157个写入中断边界、155字节逐项损坏拒绝/回退、重启有pending启动帧仍恢复 | 通过；版本1双槽、CRC16，EEPROM总用457B |

## 交互契约

| 上下文 | B1 | B2 | B3 | B4 |
|---|---|---|---|---|
| 普通空闲 | 下一个A/B/C | ARMING/录制切换 | PLAY | HOME |
| ARMING/录制 | BUSY | 取消准备/保存录制 | BUSY | 丢弃本次录制并请求HOME |
| 非绘图自动任务 | BUSY | BUSY | BUSY | 普通模式中取消并请求HOME |
| 示教设置 | 捕获DRAWZ | 捕获TRAVELZ | 进入MARKING（先PC设置TEACHZ） | 退出示教 |
| 标记/五点就绪 | MARK | POLY | CURVE | UNDO最后捕获点 |
| 绘图（含预检/暂停） | HOLD | GO | CANCEL→抬笔HOME | 无动作 |
| EEPROM保存中 | BUSY | BUSY | BUSY | BUSY |

- PC BTN 与实体按钮共用 onButtonPressed。Nano Hello、重启、CRC错误不会作为按钮。
- 单一任务owner覆盖预检、运行、暂停；没有排队抢占。HOLD维持指令姿态，绘图暂停不自动抬笔；GO续原进度。
- UNDO按捕获顺序删最后点，即使已ORDER；保留余下排列。CLEAR清空并回设置，EXIT、成功完成、取消和错误保留点集；点集仅RAM，重启清空。EXIT后保留CART。
- 取消恢复阶段属于HOME；示教上下文中四键BUSY，避免再触发绘图。此时PC HOLD/GO仍有效；PC HOME重发仍保留抬笔。PC CANCEL对非绘图任务是停止并HOLD，不保证回家。
- 路径预检失败保持最后合法指令、HOLD并报告错误；修正点/高度或路径后GO，再明确启动任务。没有自动错误重试。
- H/L：手动倍率0.75..1.5。JOINT45/90°/s、CART50/100mm/s保持现有配置；SPEED控制普通目标/RUNLINE；HOME及回放首帧准备固定60°/s；抓放各阶段独立速度；FEED控制绘图（默认保留25mm/s，1..40范围）。H/L不改变回放时间轴或绘图。
- 回放若原速度超过安全约束，会统一时间缩放并输出 PLAY timeScale；不是无条件承诺原速。
- 录制固定480B，满时有界非递归合并，保证接受至20秒上限（不含3秒准备），随后自动保存；复杂动作可降精度并标记degraded/errorBound。测试最大重建误差示例：匀速0°、停顿0.92308°、频繁变化11.9911/16.1538°、20秒复杂样本19.95°；保守界可能180°，这些样本误差不是所有动作的精度保证。

## 实际构建、容量及风险

命令：
~~~powershell
cd D:\Develop\MeArm-Bringup-2026-10-12
pio run -e uno -e uno_nano_link
& test/native/run.ps1 -Compiler C:/msys64/ucrt64/bin/g++.exe
& test/nano_link/run.ps1
~~~

| 阶段 | uno Flash/SRAM | uno_nano_link Flash/SRAM |
|---|---:|---:|
| 基线09a7 | 30742 /1253 | 31534 /1291 |
| 抓放初次合入（超限，未作为成品） | 构建超限 | 33778 /1298 |
| e463066抓放模块 | 30906 /1260 | 31732 /1298 |
| 9cfedab示教/绘图 | 30882 /1269 | 31704 /1307 |
| 最终代码19738c6 | **30902 /1269** | **31724 /1307** |

最终 D10：.text=31442，.data=282，.bss=1025；Flash余532B，静态SRAM外余741B。uno：.text=30620，.data=282，.bss=987。这是实际Bringup生产配置，四轴、D10和运行时抓放全部链接；未把合成标定固件作为交付。未重新跑旧开发工程的合成容量脚本（其注入标记针对空标定，不适用当前实测配置）。

首次超限先保存符号归因，再采用fallback已验证的const标定声明和三个编译选项，使D10由33778降至31732。const在AVR仍有112B数据对象；收益主要是常量传播后减少无关分支机器码，并非声称全部迁入PROGMEM。后续紧凑日志封装/独立进给及TEACHZ合入净减少28B Flash、增加9B SRAM。未删除官方能力、限位、UART或录制容量，未继续最终性能优化。

最终大符号（链接/LTO后的函数大小，不能当作互斥模块总量相加）：pollSerial3486B、main2586B、sampleGoal1480B、dispatchServiceAction1400B、manual998B、motion execute944B、PICK解析934B、centripetalSample800B、record observe794B、inverse3D646B、solveIK624B、validateOne578B、curveAxis314B。完整符号及map摘要随证据提交；原始map位于 .pio/build/uno_nano_link/firmware.map。

主要静态对象：frames480B、Serial157B、hardwareConfig112B、MotionTask104B、joints100B、PC命令缓冲64B、五点40B、Servo库数组36B、D10 ring16B、manualTarget16B、TeachSettings15B。录制/轨迹无新增无界容器；EEPROM没有147B RAM镜像。电脑测试中的数组/容器不进入固件。

栈不能仅由741B余量宣布安全。重点仍为 PC pollSerial→BTN3→cancelAndReturnHome→pointPose→IK3/FK，以及运动采样/IK路径叠加Servo/UART ISR。保留 noinline 服务边界，本次恢复修复仅一个条件，无新局部大数组或递归；未测实体高水位或最坏ISR嵌套。Timer0计时、Timer1 Servo、Timer2 D10 RX保持分工。15字节可用UART环约15.6ms容量，长日志和EEPROM LOAD/启动恢复阻塞可能丢事件，等待Hello重同步；不能宣称密集按键实时性已保证。

完整Native：**1,412,846断言通过**；其中容量、EEPROM、曲线过点、暂停恢复、顺序/点集互斥等证据按上表映射，而非以总数替代验收。D10寄存器模型3273、协议+真实PC循环35通过。基线Native曾在旧5mm CART预期失败；测试改为当前50mm/s下5个20ms tick的5mm目标，避开原25mm合成工作空间边界，仍严格断言方向/误差/正交分量；未改真实速度或放宽安全限位。

## PC速查与抓放配置

- 只读：STATUS、HELP、IK x z、IK3 x y z、LIST、PICK SHOW A（B/C同理）。
- 控制：MODE JOINT/CART；J0..J3 angle；x80,y120,z60（角度）；XYZ x y z；O/S；H/L；SPEED n；FEED n。
- 任务：A/B/C、REC [START/STOP]、PLAY、HOME、HOLD/GO/CANCEL、BTN 1..4。
- 示教：TEACH START；TEACH DRAWZ/TRAVELZ/TEACHZ；TEACH MARKING；MARK；UNDO；ORDER 3 1 5 2 4；CLEAR；TEACH EXIT。
- 绘图：DRAW POLY/CURVE；DRAW LINE x1 y1 x2 y2；DRAW SHAPE N/TRIANGLE/Z/V [xmin ymin xmax ymax]；原 LINE/RUNLINE 保留。
- 单行最多63字符，严格十进制；未知/额外参数拒绝。DONE(cmd)只表示已下发指令。

抓放必须分别示教 APPROACH/SRC/GRIP/LIFT/TRANSFER/DST/RELEASE/RETREAT（也可编号1..8），不能仅SRC/DST两点。每次在用户确认姿态安全并停止摇杆后 PICK SET A <阶段>，捕获已接受的四轴整数指令，不是舵机反馈。阶段1/2保持打开角，3..6保持该物体夹持角，7/8保持释放角；闭合与打开/释放须不同。PICK SPEED A GRIP 40、PICK WAIT A GRIP 200 是配置语法示例，不是本轮新标定。

PICK SHOW A mask=255表示阶段齐全，不证明避障。入口HOME路线和八阶段全部预检通过后才能输出；关节线段限位预检不包含桌面/物体碰撞模型。PICK SAVE时HOLD，逐循环保存，等待 PICK SAVED; HELD；用户决定GO。PICK LOAD恢复最近完整保存并HOLD。CLEAR修改编辑区，SAVE才持久化。EEPROM版本1：双155B提交槽+147B编辑区=457B，CRC16和最后提交标记；断写保留旧提交，无效/损坏拒绝，不保存虚构硬件参数。

## 实机接线与逐步验收（由用户执行）

| 功能 | 接线/参数 |
|---|---|
| Nano单向发按键 | Nano D1/TX → Uno D10，9600 baud |
| 共地/供电 | GND↔GND，按已确认方案 Uno5V→Nano5V；Nano运行时不插USB |
| PC | Uno USB，115200 baud，使用 uno_nano_link 固件 |
| 四键/OLED | Nano现有D4..D7/A4SDA/A5SCL不改；主控不含OLED驱动 |
| Servo/摇杆 | Servo D9/D8/D7/D6；ADC A0/A1/A3/A2，全部保留 |

本轮不提供自动烧录动作。用户按电路组已确认的供电互斥/烧录断线要求自行操作；上电会执行既有启动Servo输出，不应把“只发STATUS”理解为上电无运动。

1. 先确认当前机械姿态、供电和工作区，再由用户选择上传D10生产环境。第一条PC查询 STATUS：应看到四轴 enabled=15、cal=1、mode=JOINT、feed=25.0、autoSpeed=60.0、manualScale=1.00；通常启动J=80 120 60 50，phase=0。task枚举及状态定义见 include/motion.h。不符则HOLD、保存输出，停止运动验收，不改标定来凑通过。
2. 受控HOLD下重启Nano并查询STATUS；Hello不应引发任务。OLED本地TX不代表Uno已接受；本协议没有返回ACK或主控真实状态显示。实体按键会触发任务，不能当成无运动探针。
3. 确认运动空间后分别验收JOINT/CART原方向、HOME及夹爪40/90、夹笔HOME85，不同时测试多项。任何方向、限位、供电异常按现场停止方案处理。
4. 按八阶段配置A，再完成B/C；保存并冷启动确认SHOW恢复。先逐阶段人工确认障碍间隙，再执行A/B/C及B1循环，录制三个不同物体和x/y明显不同落点的视频。
5. B2：3/2/1→REC START，摇杆演示停顿/反向/四轴，超过10秒提示后录至15秒以上，B2保存。B3检查首帧准备、末帧、HOLD/GO；如degraded明显，记录实际误差而非声称精确复现。
6. 夹笔并独立实测drawZ/teachZ/travelZ。TEACH START后在各高度捕获；到teachZ并居中后MARKING。实体B1存五点，B4撤销并重存，ORDER任意排列；B2折线、B3曲线复用同点。绘图时实体B1暂停、B2继续、B3取消回HOME。视频必须覆盖设置、示教、记录、启动及笔迹。
7. 再验收直线和至少一种基础图形。分别记录过点误差、直线度、速度停顿及取消抬笔轨迹；超出预期立即HOLD，保留日志和参数。
8. 回退用独立目录/worktree检出基线分支构建，勿reset当前工程。基线文件可由保存分支恢复；EEPROM新增抓放数据不改变基线源码标定。先保存后来新增的实机修改再切换版本。

## 明确未完成/阻塞

- 官方任务四Word及视频/GitHub提交材料不由固件通过替代，本阶段未审核或生成。
- A/B/C实物姿态、各阶段速度/等待、HOME及从当前位置的避障路线、纸面三高度/笔尖实际精度仍依赖用户实机确认；本轮没有填猜测值。
- 只保留当前一个实测TCP/FixedPlanar模型，没有新增未经标定的PEN/GRIPPER双TCP切换、倾斜纸面模型。
- 曲线是真正向心Catmull–Rom并通过五点，按离散弦长计时及关节速度限制；尚无整条曲线加速度前瞻/严格弧长查表。这属于后续轨迹品质工作，不在此阶段继续性能优化，不能声称已达到V2全部品质增强。
- Nano单向协议无权威OLED倒计时/任务返回；保留现有本地显示，不新增返回线或主控显示库。
- 栈高水位、ISR抖动、EEPROM阻塞期间丢按键、15–20分钟稳定性待实机；Flash只余532B，下一阶段新增功能须先重新测容量。
- 当前组合上限250°及宽J0/J3范围照原配置保留，不代表本轮重新验证过机械碰撞边界。

## 修改文件与证据

新增生产模块为已有接口的功能增量，没有新增Servo所有者；未修改Nano源文件。修改文件：include/{app_controller,hardware_config,pick_place,teaching,control_settings,serial_format}.h；src/{app_controller,commands,hardware_config,main,motion,pick_place,recording,teaching,control_error,control_settings,manual_control,serial_format}.cpp；platformio.ini；test/native/{EEPROM.h,nano_link_tests.cpp,sprint_tests.cpp}。

完整差异见 integration_evidence_2026-10-14/changes.patch；构建、测试、符号、map摘要、ELF分段、源码/HEX SHA256见同目录。不提交.pio构建产物。此文档和证据提交之后停止本阶段开发。
