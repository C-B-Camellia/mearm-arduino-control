# DRAW T3 ROOT CAUSE — 定向诊断（2026-10-14）

审计对象：D:/Develop/MeArm-Bringup-2026-10-12，分支 codex/full-prototype-integration-2026-10-14，HEAD aef334b234f7b67a953f708e9f438eab0cf5c66e。开始时工作区干净。本轮只新增本报告、计时报告和桌面诊断测试/证据；不修改 src/、include/、platformio.ini，不改标定、不上传、不访问硬件。没有切换分支或实施下面的修复。

## 结论与确定性

| 发现 | 证据/位置 | 确定性及与现场现象的关系 |
|---|---|---|
| FEED 限制所有绘图阶段，包含 HOME、TRAVEL、落笔、抬笔和返回 | src/motion.cpp:229–249 的 isDrawingKind 分支不区分阶段 | 确定；显著拖慢落笔前后过程，也违背“HOME固定安全速度”在绘图内部阶段的预期 |
| 0.75mm一个样本、每tick最多推进一个输出，实际描线最高约37.5mm/s；FEED又只允许1..40 | motion.cpp:60、229、284；control_settings.cpp:6；config.h控制20ms、手动CART50/100 | 确定；现有实现不可能达到用户新提出的“至少比普通CART手操50mm/s快”。只把FEED从15改25或调到40无法解决 |
| 每段耗时向控制tick取整，超出的逻辑时间不带到下一段；自动运动dt还截到40ms | motion.cpp:237清elapsed、250–251；与现有回放executePlayback有意保留overshoot的行为不同 | 确定；0.75mm在FEED15/25下常变为60/40ms，实际约12.5/18.75mm/s；控制迟到时还会进一步拖长 |
| 相同笛卡尔点的FK→IK→整数量化可能改变1°输出；目标Z下降时量化后Z并非严格单调 | arm.cpp:132 static_cast<int>；kinematics.cpp:124仅验证浮点逆解 | 桌面可复现，源码机制确定；能引起局部高度反向、台阶和抖动。具体数值为Native模型结果，尚非AVR实测；不能单独归因所有“明显悬空” |
| TEACH DRAWZ在标记/忙碌/待输出状态被拒绝，保留旧drawZ；STATUS不显示三高度 | teaching.cpp:15、38；commands.cpp:54；app_controller::printApplicationStatus | 确定；提供的日志有workflow=2、paper=1和大量ERR INVALID，符合风险条件，但输入未回显，不能认定某一ERR就是那次捕获 |
| 浮点IK模型中没有发现DRAWZ被取负或用teachZ/travelZ替换的错误 | teaching::captureHeight→drawGoal→solveIK3D；两次有效捕获测试 | 对已测可达点自洽；若成功捕获且真正进入描线，单纯Z符号反了的推断不成立。真实毛笔TCP和舵机位置仍无反馈验证 |

详细计时、阶段表、公式及模型运行结果见 [DRAW_T3_TIMING.md](DRAW_T3_TIMING.md)。

## 用户日志能证明什么

用户明确补充：描线和画形状全过程都没接触纸面，约10秒，要求至少比普通手操快。因此“只是没等到落笔”不能作为唯一解释，必须保留完整高度问题待查。

日志包含：
- 五点：P1(144.77,32.18)、P2(169.62,-6.63)、P3(191.05,19.89)、P4(135.51,-40.28)、P5(115.85,12.26)。
- workflow=2（标记状态）、后续workflow=0、points=5/paper=1。
- J3=90时的多次UNSAFE，调整85后出现PATH VALID/DONE；本轮不重新引入这个已解决问题。
- task=9、phase=0、move=172/172对应已完成的DrawShape指令任务，不是“尚在预检”。
- 多处ERR INVALID，但没有对应输入行和时间戳；没有两次TEACH DRAWZ成功确认及其高度值，没有完整DRAW LINE参数。
- 最后task=9的STATUS关节为96/92/115/89；当前源码绘图返回HOME应90/120/60/85。因为这是完成后的状态、期间允许手动继续运动且没有逐命令记录，不能据此断言RETURN失效，也不能把它当作PEN_DOWN姿态。它说明仅凭事后STATUS不能重建整条路径；还应确认实际烧录版本。HELP吻合接口，不能证明HEX版本完全相同。

按当前真实模型换算已提供的STATUS（不是断言它们就是DRAWZ捕获瞬间）：
| 指令J0/J1/J2/J3 | 模型X/Y/Z mm |
|---|---|
| 80/92/63/90 | 101.30842 / -4.06657 / 97.03436 |
| 88/85/78/90或85 | 115.84592 / 12.26247 / 96.46655 |
| 96/92/115/89 | 165.55849 / 43.66134 / 103.14738 |

这几组数据不能替代丢失的drawZ/travelZ/两次捕获命令记录。纸面真实高度仍未知，不填猜测值。

## DRAWZ的真实数据流

1. PC TEACH START → executeAppCommand → dispatchAction(TeachStart) → beginTeach。mutableNow拒绝活动运动、pending Pose、录制和EEPROM保存。设置CART、active=true、marking=false，同步手动目标。**不清空已有三高度和点集**。
2. 手动CART中右杆ry更新 point.z -= axisVelocity(...)*dt。Z为模型世界坐标，正向向上；不是Arduino引脚高低，也不是三角度指令里的z=J2。
3. requestPose保留浮点goalAngle；applyPendingPose通过整套安全检查，输出整数角度并更新commandedAngle。没有编码器或实际笔尖高度反馈。
4. TEACH DRAWZ → captureTeachHeight(true) → captureHeight(settings.drawZ, settings.drawValid)。currentPoint取得commandedPose，再FK3D，保存其p.z。捕获的是**已经下发的整数量化关节指令的预测高度**，既不是浮点目标、也不是实测笔尖高度。舵机仍可能未实际到位。
5. 捕获要求 active且非marking，且无任务/pending/录制/保存；Z必须有限。被拒绝时原drawZ及drawValid保持不变。PC成功只打印OK，没有新Z；失败打印ERR。TEACH START成功也不等于下一条DRAWZ一定成功。
6. drawSettingsValid只检查cartesian有效、draw/travel有效且travelZ>drawZ。paper=1不证明最近一次捕获成功，不检查teachZ有效，不代表接触纸张。
7. MARKING额外要求drawZ<teachZ<=travelZ、当前模型高度与teachZ相差<=0.05mm、摇杆居中。在MARKING时CART锁teachZ，仅允许XY定位；此时TEACH DRAWZ拒绝并保留旧值。
8. startDrawLine显式保存两点XY；不从五点列表推断纸高。drawGoal第4目标为P1的drawZ，之后所有STROKE点使用drawZ，抬笔使用travelZ。每次采样读取settings，**无旧drawZ缓存**；正规设置接口在任务忙时被禁止，所以成功捕获的新值会供下一任务使用。
9. solveIK3D求浮点解并以FK验证XYZ每轴误差<0.05mm。然后执行器做关节插值，requestPose/applyPendingPose再次完整检查，最后整数截断交Servo。浮点模型通过不等于整数输出仍在0.05mm内。
10. PEN_DOWN是两端IK姿态之间的关节插值，不保证整段垂直笛卡尔直线；STROKE的笛卡尔离散点间也是短关节插值。段末应到达量化后的目标，不能将插值中间的短暂高度当成最终纸高。

## 数学自洽与量化证据

当前FixedPlanar模型：
- alpha=184-J2，beta=J1-112.9（单位度）。
- Z=80sin(alpha)+80sin(beta)+63-6。
- Yaw和toolTangential只影响XY；FixedPlanar工具偏置53/-4.65/-6原样保留。
- 解的安全分支、NaN/Inf、限位和组合148..250°都在；本轮不改。

在同一XY下，IK成功意味着FK(IK(X,Y,Z)).z在目标Z的0.05mm内。两目标差大于0.1mm时，该验证界足以保证浮点FK高度顺序；更小间隔不能只靠0.05界作全局严格单调证明。Native在同一XY扫描2501个目标、每步降0.01mm，所有浮点结果按预期下降。

实际量化不具备上述单调保证：
- HOME 90/120/60 经FK→IK为90/120.00002/59.99998。
- static_cast<int>得到90/120/59。
- 模型Z从133.21112变成132.42027：几乎零长度的HOME_TO_TRAVEL_Z仍能产生1°目标差和约0.79mm高度变化。
- 同一XY(101.57516,13.78767)，目标113.44112→113.43112mm；量化后FK高度111.53661→112.32745mm，**目标下降0.01mm，整数输出反而升约0.79085mm**。扫描有2480个平台和1个反向跳变。
- 两次更大幅度有效捕获：A=130.43454，B=127.64931；相同XY的PEN_DOWN目标J1/J2分别118.13623/59.75812和116.23315/59.58595；量化Z分别129.64371、126.85846，依然下降。可见不能把任何更低DRAWZ都描述为反向。

以上为按当前生产几何运行的桌面数学测试；虚拟姿态118°/116°是明确的测试输入，不是真实纸面标定，未写入生产配置。AVR浮点/三角库可能给出不同末位和跳变位置，不能冒充实物量测。

对当前固定偏置模型，单个整数截断造成的Z误差保守界为2×80×pi/180≈2.793mm（每个关节不足1°）。两个目标的误差差值可更大，不能以单次0.79mm代表最大值；明显厘米级高度异常若出现，不能仅用此量化机制解释。

## 软件之外尚未确定的高度原因

- FixedPlanar假设工具偏置不随末端俯仰旋转。实际毛笔笔尖若随夹爪俯仰变化、夹持长度改变、与之前标定铅笔不同，模型内部自洽仍可能在不同XY高度偏差。需要实测TCP/多姿态误差，不能直接改成DistalLink或改偏置凑接触。
- commandedPose没有舵机反馈。机械负载、供电、死区、背隙、刷毛形变或尚未到位，都可能令真实高度与预测不同；不能从Native排除，也不能从当前日志确认。
- 捕获失败沿用旧drawZ仍是需要优先核实的路径。日志workflow=2及INVALID支持调查这一方向，但不足以绑定具体捕获。
- PATH VALID之后确有多个空中阶段，且它们被FEED拖慢；这会放大前期悬空观察，但与用户“描线全过程未接触”的补充不完全相符。

## 最小修复提案（待用户批准后另建分支）

**A. 先修确定的计时与可观测性，保留现有曲线及空间采样公式。**
1. 给已有索引范围增加轻量阶段判断。HOME/TRAVEL/RETURN不再共用纸面FEED，使用已冻结的60°/s关节安全速度；PEN_DOWN/PEN_UP先保持当前保守规则，不凭空增加垂直速度参数。FEED专门定义STROKE进给后，再由用户确认落抬笔profile。
2. 让描线携带段间剩余逻辑时间，消除每0.75mm强制重新对齐tick的耗时损失；沿现有已预检路径做有界的时间游标推进，每tick仍只提交一帧。必须保留过点/落笔/抬笔/HOME强制端点和单帧关节限速，不能直接把dt裁剪去掉后跨大步跳目标。
3. 普通CART50mm/s已高于FEED上限40及执行器37.5上限。若要求STROKE至少达到手操速度，批准后需同时修执行器推进和FEED允许范围；用相同已验证路径对比有效进给，不保证任何任意路径都能超过关节安全约束，也不以修改物理标定换速度。仅提高FEED上限无效。
4. TEACH捕获成功回显新值；STATUS按需显示drawZ/teachZ/travelZ及有效位，绘图回显HOME/TRAVEL/PEN_DOWN/STROKE/PEN_UP/RETURN。只在阶段切换/查询输出，避免20ms刷日志。保留失败时原数据，但明确说明未更新。需实测Flash/SRAM开销，当前Flash仅余532B。

**B. 量化单独最小处理，禁止本轮直接全局改舵机输出。**
- 首先针对“整数附近浮点误差造成凭空少1°”提出数值稳定化：安全验证后，仅将距整数极小容差内的数值规范为该整数，量化后仍重新检查全部限位/禁用轴；容差及位置必须在新分支用AVR/Native验证后定，不改模型参数。
- 普通小数截断的高度台阶不会因此消失。不要承诺此修复保证Z全局单调；若进一步评估最近整数/脉宽精度策略，须作为独立行为变更测试，不悄悄替换所有Servo输出。
- 修复验收至少包括HOME FK→IK→输出恒等、合法/非法边界、相同XY两次捕获、扫描量化误差、原安全测试及Uno容量。禁止为通过单调测试放宽限位。

**C. 仍须实测的部分。**
先得到同一XY处新旧捕获的成功ACK及三高度/实际下发关节、PEN_DOWN末/描线阶段记录；若浮点/整数预测已更低但实体仍偏高，再测真实TCP和舵机实际位置。不得直接减小toolVertical或翻转Z符号。此部分不以“换算法”代替几何诊断。

## 回归与交付边界

新增 test/draw_t3/draw_t3_tests.cpp、run.ps1。测试直接在桌面编译包含未修改motion.cpp，以观察其内部目标和duration；脚本从其他编译源排除motion.cpp以避免重复定义。这些文件不进入PlatformIO固件。

运行：
~~~powershell
cd D:\Develop\MeArm-Bringup-2026-10-12
& test/draw_t3/run.ps1
& test/native/run.ps1 -Compiler C:/msys64/ucrt64/bin/g++.exe
~~~

- 定向诊断6714断言通过：两次捕获、pending/忙碌/marking拒绝保留旧值、最新drawZ进PEN_DOWN、固定XY量化扫描、FEED上限/手动速度冲突、全阶段目标和耗时、20/100ms服务间隔、端点确认频率对比。
- 原Native回归1,412,846断言通过。
- “诊断通过”包含对现有缺陷的复现断言，不表示已修复生产问题。
- 完整输出见 draw_t3_evidence/diagnostic.log、native.log；生产源码hash核验见source_manifest.json。用户输入日志未带命令和时间戳，现场真实耗时与纸面高度仍不能精确复算。
- 本轮不重编/上传生产固件；既有HEAD资源为uno_nano_link Flash31724/SRAM1307B，引用既有构建证据，未声称是新固件结果。本轮生产文件内容保持原样。

等待用户批准最小修复范围后，才在独立分支实施。阶段二绘图缺陷尚未修复，不能核心冻结。
