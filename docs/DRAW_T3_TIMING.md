# DRAW T3 TIMING — 当前执行器的实际计时（2026-10-14）

对应HEAD aef334b234f7b67a953f708e9f438eab0cf5c66e。以下分为源码确定公式、当前几何下的桌面调度测试、用户现场数据可推导部分。没有为未知纸面高度或真实DRAW LINE起点填值。

## 调用链与时钟

PC DRAW LINE x1 y1 x2 y2
→ commands.cpp:89 executeLine
→ app_controller::executeAppCommand/executeDrawCommand
→ dispatchAction(DrawLine)
→ motion.cpp:373 startDrawLine
→ motion.cpp:355 startDrawing
→ motion.cpp:166 begin：取得commandedPose作为origin，进入Validating
→ motion.cpp:173 validateOne：sampleGoal→drawGoal→pointPose→solveIK3D，逐关节小段完整预检
→ PATH VALID，切Running
→ motion.cpp:284 updateMotion
→ motion.cpp:229 execute：加载目标、计算duration、关节插值
→ arm.cpp:98 requestPose（完整安全检查并设置pending）
→ main.cpp:105 applyPendingPose
→ arm.cpp:111整数输出（量化后再检查）→ Servo.write
→ Servo库Timer1持续输出约20ms脉冲周期。

主控制tick=20ms，WAYPOINT_DELAY_MS=10ms只是每个运动小段duration下限，**不是10ms Servo PWM，也没有每段delay(10)**。Timer0/millis与Timer1 Servo不同。pollSerial、button_link、EEPROM服务在控制之前；错过tick只输出一帧，不突发补帧。

## DRAW LINE阶段与目标

记H=homeViaCount+1；当前真实配置homeViaCount=0，所以H=1。
记A=(x1,y1)、B=(x2,y2)，L=|B-A|，N=ceil(L/0.75)，当前有效线段还要求L>=0.1mm且N<=4096。
总目标数M=2H+4+N；当前为N+6。

| 当前H=1时目标索引 | 阶段 | 目标和坐标来源 | 当前FEED是否限制 |
|---|---|---|---|
| 1 | HOME | 实测HOME关节90/120/60/85；FK为HomeXYZ | 是 |
| 2 | HOME_TO_TRAVEL_Z | HomeX/HomeY/travelZ，IK求角 | 是 |
| 3 | TRAVEL_TO_P1 | x1/y1/travelZ，IK求角 | 是 |
| 4 | PEN_DOWN | x1/y1/drawZ，IK求角 | 是 |
| 5..4+N | STROKE | x1+(x2-x1)k/N，y1+(y2-y1)k/N，drawZ | 是 |
| 5+N | PEN_UP | x2/y2/travelZ | 是 |
| 6+N | RETURN_HOME | 实测HOME；有via时先依定义经过via | 是 |

多HOME via时每次HOME/RETURN均展开H个关节目标。图形、五点折线和曲线也使用同一前后三高度结构；曲线样本数量来自保守长度估计而非固定0.75mm真实弧长。本文的等距0.75讨论主要针对DRAW LINE/直线stroke。

每个采样pose先复制origin，因此除HOME/via外夹爪继承任务开始时的指令；startDrawing要求它与HOME/via夹持角一致。J3=85可解决这道门禁；不会改变Z模型或使FEED提速。

## 实际duration公式

第i个目标的浮点关节向量为Q_i，开始关节为C_i=commandedPose（**量化整数**）。
代码用于进给距离的前点为上个浮点目标FK(Q_{i-1})，不是FK(C_i)。首目标之前Q_0=origin。
令：
- delta_i=max_j |Q_i[j]-C_i[j]|（度）。
- ell_i=|FK(Q_i)-FK(Q_{i-1})|（毫米）。
- v_joint=DEFAULT_MOVE_SPEED=60°/s，对所有drawing kind强制此值。
- F=drawingFeed（FEED15或25等）。

实际整数毫秒duration：
~~~
D_joint_i = ceil(1000 * delta_i / 60)
D_feed_i  = ceil(1000 * ell_i / F)
D_i      = max(10, D_joint_i, D_feed_i)
~~~

没有单独HOME/TRAVEL/PEN_DOWN/STROKE速度字段；所有阶段都通过同一个drawing kind分支，故即使HOME动作被60°/s限制，还会被纸面FEED二次拖慢。SPEED、H/L不会改变这条drawing速度公式；GO/HOLD也不改FEED。

无暂停、严格20ms服务、正常端点确认时，每段实际占用约：
~~~
T_i = 20 * ceil(D_i / 20) ms
T_total ≈ sum_i T_i + preflight_CPU_time + final_ack_loop_latency
~~~
新段把elapsed清0，上一段最后一个tick超过duration的时间丢弃。
0.75mm的理想feed时间在F15为50ms，在F25为30ms，实际一般为60/40ms；浮点ceil偶尔使50成为51、30成为31，仍落在60/40ms档。
10ms最小段时间受20ms控制tick覆盖，不能产生100Hz舵机目标提交。

关节整数目标常有小于1°的重复：即使输入t增加，Servo.write可能连续多帧写同一整数角。下一段又从当前整数姿态重新插值。因此纸面运动可能呈现台阶/停滞；这不是CPU必然停了，也不是实际位置反馈已到位。两个相邻样本间的角度差来自整数起点误差，可能使D_joint反过来主导D_feed。

## 端点确认与重复计算

- execute到elapsed==duration时设置endpointQueued。
- 同一控制loop随后applyPendingPose，pending清掉。
- 后续任意一次updateMotion都可确认端点并completedWaypoints++；**确认不要求再等20ms，也没有额外的固定dwell**。
- 若确认发生在非控制loop，下一目标仍要等下一个controlDue才能execute。若只每20ms调用一次updateMotion，确认和下一段执行可在同一次调用完成；最终DONE可能晚一个tick。
- 主要确定性开销是每个样本独立的时间取整与不能在一个tick消费多个短样本，并非每点必加一个“额外20ms确认延时”。
- 预检和正式执行各自会重新计算drawGoal与IK，无轨迹缓存。
- 正式每个新段执行previous FK、sampleGoal内IK及IK的FK校验、目标FK，再计算时长；一个新段通常含3次FK3D以及1次IK主求解。每段每tick还做关节插值，requestPose和apply再检查完整Pose。
- LINE每次strokeSteps只是常量长度计算；多段曲线的drawGoal还会重新查前段长度/节点和曲线求值。具体AVR CPU时间未测，不能把桌面运行时间当作Uno耗时。

控制迟到时：
~~~
dt_i = min(now-lastUpdate, 40ms)
elapsed = min(D_i, elapsed + dt_i)
~~~
若控制实际每100ms才服务一次，每次最多推进40ms逻辑时间，丢掉60ms。小段D<=40仍至少花100ms；D=50/51花两个100ms；长段时钟近似慢到40%。这是负载下的确定性退化机制，**没有证据表明现场tick确实为100ms**。

## 速度上限与手操要求冲突

每个0.75mm样本至少占一个20ms tick，不能一tick完成多个stroke样本：
~~~
effective stroke speed <= 0.75 / 0.020 = 37.5 mm/s
~~~
实际步长L/ceil(L/0.75)<=0.75，因此这还是上界，关节限速、整数误差及CPU迟到只会降低。
当前普通CART慢档50mm/s、快档100mm/s；FEED允许范围1..40。故当前无法实现“STROKE至少比普通手操快”，更不能保证包含强制HOME往返的总任务比只沿线手操更快。验收应在相同合法路径上分别记录STROKE进给和整任务时间。

修复必须同时处理现有路径的时间推进和FEED范围，且继续满足关节安全速度；单独提高FEED、降低采样密度或改20ms tick，都不能作为已验证解决方案。本轮均未实施。

## 桌面模型测量（不是用户纸面参数）

测试保留生产几何；只在Native替身里：
- 将已接受指令置为HOME，捕获travelZ=133.21112。
- 将虚拟肩关节置116°、其他用HOME，捕获drawZ=127.64931。
- A=HOME的模型XY=(101.57516,13.78767)，B=(101.57516,43.78767)，30mm直线，N=40。
- 这些是可重复的诊断输入，不是建议实机设置，也不代表用户当前纸面/起点。
- 预检70次validateOne调用；无Servo目标输出。预检耗时取决于实际CPU/主loop频率，本测试不测其墙钟。

| 阶段 | F15的ell/delta/D | F15实际20ms服务 | F25的D | F25实际20ms服务 |
|---|---|---:|---:|---:|
| HOME | 5.58392mm /4° /373ms | 380ms | 224ms | 240ms |
| HOME→travelZ | 约0.00003mm /0.00002° /10ms | 20ms | 10ms | 20ms |
| 空中→P1 | 约0mm /约1° /17ms | 20ms | 17ms | 20ms |
| PEN_DOWN | 5.56180mm /3.76685° /371ms | 380ms | 223ms | 240ms |
| STROKE40段总计 | 每段约0.75mm、D50/51ms | 2400ms（12.5mm/s） | 每段D30/31ms | 1600ms（18.75mm/s） |
| PEN_UP | 5.56180mm /4.70782° /371ms | 380ms | 223ms | 240ms |
| RETURN | 30mm /15° /2001ms | 2020ms | 1201ms | 1220ms |
| 总计，不含预检 | — | **5600ms** | — | **3580ms** |

同一模型改为100ms服务间隔：FEED15整任务16300ms、stroke8000ms；FEED25整任务9100ms、stroke4000ms。这是人工施加的调度压力测试，不是现场计时。
20ms下关闭“同tick追加一次非控制确认”：F25总计3600ms，仅最终确认多20ms，stroke仍1600ms。确认机制不应被错误计算成每点必多20ms。
重复相同整数四轴输出帧：F15=224帧，F25=123帧，说明连续细样本的整数台阶效应，不能当作硬件完全静止的实测帧数。

HOME→travelZ近零移动仍出现J2 60→59，来自IK的59.99998被截断；下一零距离段因真实指令已变又出现约1°角差，解释表中“ell近零但delta非零”。详见ROOT_CAUSE。

## 用户日志可推导的时间

1. 用户给出“画线约10秒”的体感，但没有完整DRAW LINE命令、捕获高度和起始姿态时间点。无法精确还原该线的阶段长度及总时长。
2. 实际五点P1→P2距离46.08403mm，N=62。如果那次LINE恰好取它们，纯stroke名义时间为3.07227s（F15）或1.84336s（F25），20ms分段取整常变约3.72s/2.48s，再加HOME/空中/落抬/返回。**没有证据那次LINE用了P1/P2**，只作已有数据的明确条件示例。
3. 另有DrawShape完成记录move=172/172。按当前H=1，stroke样本数=172-6=166；N/Z/三角形为3段、V为2段，名义总描线路程约(122.25,124.5]mm（V下界123mm）。因此F15纯stroke名义已约8.15..8.30s，F25约4.89..4.98s，tick取整通常进一步接近9.96s/6.64s，再加进出场。没有shape类型/尺寸及高度，不把这些近似当成其真实耗时。
4. 原日志中DONE后才有STATUS，不能据此给HOME/PEN_DOWN标注真实持续时间。

## 事件和操作者可观察阶段

- 发DRAW命令的OK：请求被接受，进入预检；尚未输出路径运动。
- PATH VALID：**整条离散任务及每段关节安全预检完成**，进入Running；接下来先HOME。
- 正式落笔：第4目标PEN_DOWN在其duration内关节插值；完成后move=4/M（H=1），仍只是指令到位。
- 开始描线：move从4/M继续，目标5开始STROKE；源码没有PEN_DOWN或STROKE文字事件。
- PEN_UP：move=4+N/M后处理目标5+N；RETURN：其后。
- DONE(cmd)：最后HOME目标已被applyPendingPose并软件确认；不等待物理舵机位置反馈/笔尖实测到位。
- STATUS的phase=2只表示Running，不能区分HOME/TRAVEL/STROKE；phase=0是Idle。task=8为DrawLine、9为DrawShape、10折线、11曲线。move反映已确认目标数，不是纸面描线百分比。

## 精确计算现场耗时仍需要的数据

完整DRAW LINE x1 y1 x2 y2、发令时已下发J0..J3、成功捕获的drawZ/travelZ、HOME/via配置（本源码已有）、FEED；加上PATH VALID/开始描线/DONE的时间戳及控制tick间隔分布，才能区分纸面轨迹时间、空中时间和CPU迟到。现有STATUS不输出高度，下一修复提案包含最小只读查询；本轮不加入生产代码，也不要求用户通过猜测纸高补齐。

证据：test/draw_t3/run.ps1；docs/draw_t3_evidence/diagnostic.log和native.log。Native定向6714断言及原回归1,412,846通过；这些是软件诊断结果，尚未实施修复或实机验证。
