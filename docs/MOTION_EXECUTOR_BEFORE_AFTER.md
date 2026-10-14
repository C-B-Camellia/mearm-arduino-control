# 自动运动执行链与时间对照（2026-10-14）

## 证据与可复现范围

RC0=80e1e32；R1=b7d9606；R2=本独立分支的代码checkpoint，最终SHA见manifest.json/code_commit。生产配置使用当前Bringup实测硬件数值，不注入合成标定。所有运动数据均为桌面模拟输出命令，**没有舵机位置反馈、没有烧录/实机执行**。

重跑当前测试（项目根目录PowerShell）：

```powershell
& C:/Users/12530/.platformio/penv/Scripts/pio.exe run -e uno -e uno_nano_link
& ./test/native/run.ps1 -Compiler C:/msys64/ucrt64/bin/g++.exe
& ./test/draw_t3/run.ps1 -Suite minimal_tests.cpp
& ./test/draw_t3/run.ps1
& ./test/draw_t3/run.ps1 -Suite draw_t3_tests.cpp
& ./test/nano_link/run.ps1
& C:/Users/12530/.platformio/penv/Scripts/python.exe -X utf8 test/minimal_architecture/run_legacy.py
```

RC0对照：将git archive 80e1e32中的src/include/test解到独立目录；仅复制当前minimal_tests.cpp和rc0_minimal_tests.cpp到其test/draw_t3，然后运行该目录run.ps1 -Suite rc0_minimal_tests.cpp。后者只取消新增的“旧版不存在的帧去重”断言，所有几何/路径断言和测量保持一致。现成目录 `.pio/minimal-rc0-80e1` 可重跑。不要在其上传固件。

## 三代调用链

- 历史20335f2：startLine→逐waypoint IK预检→controlDue时executeNextWaypoint→IK→立即写Servo→计算完成后的millis开始独立等待。没有连续关节限速；浮点姿态被当commanded，Servo再截断。此版本安全能力落后，仅作历史执行链参照，不能回退到生产。
- RC0：PC/按钮→单MotionTask→sampleGoal/drawGoal→整路径逐关节子段预检→execute最多跨16细结点→每段IK、两端FK求进给距离、max(关节时长,进给时长)→requestPose→主循环一次applyPendingPose→Servo。每个细结点0.75mm尺度，曲线还有保守过采样。
- R2：入口/owner/预检/暂停/取消相同→共同drawGoal生成5mm尺度几何结点并给出空间弦长→一个execute连续时间游标和关节/空间共同约束→requestPose→一次最终量化校验→仅当整数整帧变化才写Servo；不变也接受float目标并完成确认。IK复核FK仍在，计时不再做两次FK。

这次没有新建通用轨迹框架、第二队列或第二套按钮状态；也没有假装删除本来就只有一份的owner。Playback独立的时间戳采样函数仍由同一个MotionTask执行：它不能被改成DRAW按FEED计时。

## 历史RUNLINE与真实更新间隔

git中的20335f2实际配置是5mm/1000ms，后续667047d是4mm/10ms。用户实测5mm/10ms与10mm/20ms不能伪称由某个已找到commit原样重建。
`run_legacy.py`使用20335f2原始motion/arm/kinematics源码，只在隔离桌面配置中分别设置用户给出的两组步长/等待；测试起姿J1=120,J2=60，平面X+20mm，不是下面世界Y+20mm DRAW的同一空间路径。注入的5ms是可控计算开销假设，**不是AVR实测**。

|参数情景，控制20ms|模拟IK耗时|实际目标输出时刻ms|DONE ms|
|---|---:|---|---:|
|5mm/10ms|0|20,40,60,80|100|
|5mm/10ms|5|25,45,65,85|100|
|10mm/20ms|0|20,40|60|
|10mm/20ms|5|25,65|100|

旧5mm四点float J1/J2：117.888/63.962、116.042/67.8733、114.461/71.7289、113.139/75.5273；整数分别117/63、116/67、114/71、113/75。旧10mm为后者第2/4点。完整输出见legacy-trace.log。
这证明“等待20ms”不等于“20ms更新一次”：若等待从计算结束开始，下一控制tick可能尚未满足等待。旧实现也会一次跳数度；不能为了复现其速度而删除当前关节速度限制。历史RUNLINE没有五点曲线，不给它编造五点经过误差。

## 共同DRAW阶段与时间公式

n=homeViaCount，penDown=n+4，penUp=totalWaypoints-n-1。所有LINE/SHAPE/POLY/CURVE使用同一套定义：

|阶段|目标|约束|
|---|---|---|
|HOME/via|已标定HOME路线，夹笔角不变|max(10ms,ceil(max关节差/60*1000))|
|HOME_TO_TRAVEL|HOME的XY、最新travelZ|同上，不受FEED|
|TRAVEL_TO_P1|第1点XY、travelZ|同上，不受FEED|
|PEN_DOWN|P1XY、最新drawZ|max(10ms,关节时长,ceil((travelZ-drawZ)/FEED*1000))|
|STROKE|统一drawZ、对应路径XY|每几何段max(1ms,关节时长,ceil(空间弦长/FEED*1000))|
|PEN_UP|最后点XY、travelZ|与PEN_DOWN相同|
|RETURN|HOME/via|固定60度/s，不受FEED|

关节上限没有提升；不同约束取最大值，不是把两段等待相加。elapsed只按控制时间积累一次，跨非关键结点保留余时；真实拐角/五点/HOME/抬落笔须经输出确认。每tick至多16段、有效dt至多40ms；触顶会减速，不能保证任意病态曲线都达到FEED。整数目标可以不变，仍推进连续轨迹，不能等待“角度必须变了”才前进。
PATH VALID表示整条预检完成，尚未开始HOME/落笔；DONE(cmd)表示最后目标命令已接受，不是物理到位。

## 20mm直线定量对照

使用实际FK/IK模型和当前HOME；测试纸面由仅用于模拟的J1=116姿态捕获，travel由HOME捕获；起点是HOME世界XY，终点Y+20mm。此高度不是用户现场纸面，不能复制进生产参数。输出所有float/整数关节值在rc0-trace.log、r2-trace.log的FRAME行：
`FRAME,feed,t_ms,completed_before,stroke_flag,floatJ0,J1,J2,J3,intJ0,J1,J2,J3`。

20mm理想描线时间：FEED15=1333.333ms，FEED25=800ms；HOME/抬落笔的现场总时间必须给实际起姿和drawZ/travelZ才可计算，不猜测。

|版本/FEED|描线ms|全任务ms，不含预检|接受帧数|真实Servo整帧写入次数|整任务整数目标变化次数|
|---|---:|---:|---:|---:|---:|
|RC0/15|1360|2420|121|121|36|
|R2/15|1340|2400|120|37|37|
|RC0/25|820|1600|80|80|37|
|R2/25|800|1580|79|35|35|

描线期间整数目标变化：RC0 FEED15/25为14/15次，R2为15/13次，平均约11.19/16.25Hz（R2）。这才是目标真正变化的频率；50Hz控制/Servo.write不能制造50Hz的1度分辨率运动。R2保留Servo整数角语义，未引入微秒PWM插值，所以不能承诺已消除肉眼所有停顿。

20mm线浮点横向误差RC0约0.00066mm、R2最大0.03041mm；浮点Z误差RC0约0.00011mm、R2最大0.00388mm。舍弃密集IK结点的代价被测出，而非隐藏。此界限只覆盖这组实测模型/测试路径，不是全工作空间证明。

## 曲线、五点和高度一致性

五点由相同实测模型下的虚拟关节姿态生成，不写入EEPROM/生产配置。保留向心Catmull–Rom公式(alpha=1/2)、相同点序、边界反射和全部五个结点；样本FK逐点误差<0.05mm。执行时整数目标到五点的最近距离<3mm，这是关节整度量化下的软件断言，不等于真实笔尖误差。

FEED25/100分别验证LINE、N、POLY、CURVE，所有STROKE目标都取最新drawZ；四类执行浮点Z最大误差0.01424mm。测试曲线以每段256细分的离线参考比较：R2浮点XY最近参考误差<=0.01671mm，整数FK曲线偏差<=0.95105mm；参考离散本身存在约0.02mm级采样误差，因此不是解析Hausdorff上界。

30mm补充速度回归：FEED15/25/80/100的R2描线2020/1200/380/300ms；五点POLY FEED80描线480ms、CURVE560ms。曲线几何样本288→44。100ms迟到tick测试曲线描线1200ms，正确减速，不突发补帧。

先前LINE/POLY悬空、CURVE正常的现场现象，不能由这些软件测试宣布解决。四入口高度路径源码一致，仍需排查捕获是否OK、是否观察TRAVEL阶段、整度量化、TCP模型与真实笔尖差异。没有改Z方向、工具偏置或机械标定。

## 计算量、资源与栈

仪器包装仅统计motion/测试层直接调用IK/FK；**IK内部FK复核不计入directFK，始终保留**。

|场景|RC0最大directIK/directFK每tick|R2|
|---|---:|---:|
|20mm LINE FEED25|1/2|1/1（1次FK发生在HOME_TO_TRAVEL，描线计时为0）|
|CURVE FEED25|5/10|1/1|
|CURVE FEED100|11/22|3/1|

原曲线每样本算1次Catmull–Rom；R2为弦长算当前/前一参数2次，但样本数大幅下降。此项还没有AVR微秒证明；它不改变公式，避免引入弧长缓存数组。

最终完整D10 Flash32162/32256、SRAM1283/2048；相对RC0 Flash多4B，静态少24B。单纯计时/稀疏化+去重复配置曾32106B；整帧输出去重另花56B，实际写次数下降见上表。没有声称Flash大幅改善。
execute自身栈70→53，sampleGoal85→87，main48→51；绘图所列帧合计净少12B。PC BTN3取消链未增加调用层，但main帧多3B，静态少24B，已知部分净余量多21B。完整运行栈含libm/ISR未知，不能凭765B静态余量宣布安全。noinline输出函数尝试超Flash6B，已撤回并保留失败日志。

最坏控制周期耗时：尚无实机测量，不填估算微秒作为事实。每tick仍可能最多16个IK及每IK内部FK，串口打印可等待硬件TX，录制压缩/EEPROM LOAD有独立成本。后续实机应测controlMaxUs和gapMaxUs/串口负载，串行命令应与绘图分时操作。当前构建通过不代表WCET、Servo/Timer2中断抖动或供电已验收。

## 最短现场复测（仅由用户手动执行）

1. 留存当前可用固件与分支；此候选位于独立分支，未覆盖原集成。先不换标定。用户自行构建 `pio run -e uno_nano_link`；本任务没有执行上传。
2. 上电后仅STATUS：确认J0..3、CART标定cal=1、组合限位仍148..250、held/error与预期相符，PC命令正常；确保实际笔已夹紧且J3=85。若状态异常先停止，不发送运动。
3. 使用原已验证方法建立纸面：TEACH START，在真实笔尖落纸姿态执行TEACH DRAWZ并确认OK；实际抬到安全高度后TEACH TRAVELZ并确认OK，STATUS核对Z=draw/teach/travel且travel>draw。不要把报告虚拟Z当真实值。保留五点，必要时LIST/ORDER核对。
4. 选择现场已验证可达且安全的20mm线段，明确给出两端XY。FEED15、DRAW LINE x1 y1 x2 y2；记录PATH VALID、实际落笔、开始描线、结束描线、DONE时间。必须等到真正落笔阶段再判断高度。预期描线约1.34s，关节速度约束主导时可更慢。
5. 同一线段FEED25重试，描线约0.80s。观察真实落纸及抖动；若路径位置/高度异常、发热或失步，HOLD并停止后续验收。不要用增加FEED掩盖抬笔高度错误。
6. 复用原五点DRAW POLY/CURVE：B1暂停、B2继续、B3取消；确认先抬笔再HOME、夹笔不松。再运行N图形。暂停期间不得自行重启/抓放。
7. 录制B2准备3秒后实际记录>=15秒，B2保存，B3回放并中途HOLD/GO；D10重复/重启测试须在安全静止状态，由人观察是否无误触。通过这些实机结果后才评估CORE-FROZEN。

失败回退：停止候选运动，保留串口原始命令/时间/STATUS；使用已保存80e1e32或原实机已验证分支，由用户选择恢复，不能reset/覆盖新标定。若只是FEED慢但控制周期稳定，先判定是否60度/s关节约束或整数分辨率，不能直接删限速。
