# 最小充分架构决策（2026-10-14）

## 基线、范围与结果

真实目录 `D:/Develop/MeArm-Bringup-2026-10-12`，独立分支 `codex/minimal-motion-2026-10-14`。
开始时工作树干净；本报告 RC0 指最新已提交源码 `80e1e32c834ecc4da5a39f73d6884fc345c4f27b`（代码来自 bd1fee7），不是把更早的 aef334b 当作当前源码。原集成分支 `codex/full-prototype-integration-2026-10-14` 未移动。原始源码仍可由该 SHA 恢复，RC0 重建、测试和输出序列保存在 `minimal_motion_evidence/rc0-*`；隔离历史构建在 `.pio/minimal-rc0-80e1/`，不能上传。

R1 checkpoint：`b7d96067703f6c9ee29ad12bd5289e7bb67ab600`。
R2 的代码、回归和本报告为下一个独立 checkpoint；最终准确 SHA 见 Git log 与 evidence/manifest.json 的 code_commit 字段（文档封存提交不改变生产源码）。

|版本|uno Flash/SRAM B|完整 D10 Flash/SRAM B|相对上一步 D10|
|---|---:|---:|---:|
|RC0|31338 / 1269|32158 / 1307|基线|
|R1 保留版本|31306 / 1269|32128 / 1307|-30 / 0|
|R2 路径计时/结点|31310 / 1269|32132 / 1307|+4 / 0|
|R2 去重复关节配置|31286 / 1245|32106 / 1283|-26 / -24|
|R2 输出层去重复整数帧，最终|31342 / 1245|32162 / 1283|+56 / 0|

最终相对 RC0：完整 D10 **Flash +4 B，静态 SRAM -24 B**；Flash 余94 B，SRAM 静态之外765 B。没有为了数字删除功能/放宽限位。D10 ELF `.text=31904, .data=258, .bss=1025`；Flash=text+data，SRAM=data+bss。默认 uno 不启用 Nano D10；实机有 Nano 必须使用 uno_nano_link。

本轮是降低运行计算与重复状态的候选，不是 CORE-FROZEN，也没有获得硬件可靠性结论。

## R1 删除和拒绝删除

- D：从未被任何生产调用设置的 TooShort/NoMovement/LinkUnconfigured 错误项及字符串。保持其前所有错误编号；越界错误打印 INVALID。旧测试直接强制枚举这些死值，已改为测试全部可达项及255越界，不删除运动异常测试。
- C→精简：manual 被 HOLD/pending/owner 阻塞时只更新时间并清 armed；回中重新武装时才从接受姿态同步/FK。避免每轮重复计算一个不能使用的目标。完整 JOINT/CART 与暂停、恢复、回中测试保留。
- 撤回：把 RecordToggle 的公共分发递归改成直接分支。不凭源码判断：实测完整 D10 为32212 B，比RC0多54 B；撤回该项后32128 B。对应 `r1-build.log` 为失败方案的构建证据，`r1-retained-build.log` 才是R1保留版本。初次Native失败是死错误表断言未更新，最终 `r1-native-final.log` 通过。
- 暂停删除：N/TRIANGLE/Z/V、数字 IK 旧入口、REC START/STOP、PICK 阶段名称、默认环境的旧文本按键分支均有外部接口影响。本轮未删。图形仅16B常量表；删除不能解决主要运动计算瓶颈。用户若明确冻结为仅N，可另行改变HELP和验收合同，本轮不擅自选。
- TeachPath 是未实例化的旧声明，D类，运行时0B；不为代码变短删除。resetControlSettings 等仅测试引用由链接裁掉，同理保留测试接口。

## R2 实际改变

1. DRAW 仍使用一个 MotionTask、一个执行器、一个共同 drawGoal 路线，没有增加四套任务。
2. 直线/POLY/图形按 `ceil(length/5mm)` 取几何结点，代替0.75mm；每个顶点强制输出确认。曲线保持原向心 Catmull–Rom 公式及端点反射，使用原 `2*(三条弦长)` 保守取样公式、分母改5mm。并非曲线每5mm输出一次：曲线测试实际平均结点间距更小，Servo仍每20ms最多一帧。
3. 相邻结点之间仍按原关节线性插值；预检检查的也是这些实际执行关节段，<=1度预检子段未删除。结点变稀增加少量几何近似误差，见对照报告；不宣称全工作空间误差已获证明。
4. drawGoal 直接提供两生成点的弦长；PEN DOWN/UP 距离为 travelZ-drawZ。执行器不再对同一段两端做FK来重新求距离。IK内部的FK正确性复核仍保留。
5. segmentDistance 与只在预检期间有效的两个uint16计数复用同4B。MotionTask保持104B。它只在首次加载运行段时消费；预检调用sampleGoal后立即重建验证计数，二者不并存。
6. JointControl 删除从未有生产读取者的 joystickPin/center/directionSign，4轴节省24B。真实ADC配置一直由 joystick.cpp 的PROGMEM表读取（A0/A1/A3/A2、506/519/519/515、方向1/-1/1/1、死区20），该文件逐字未改。Servo引脚9/8/7/6、所有角度值、组合限位148..250、HOME和夹笔配置不变。旧test_main仅同步内部结构断言，活动Native仍测试真实ADC方向。

7. 唯一输出层在更新原有float目标时比较整数目标；整帧都相同就不重复Servo.write，仍完成pending确认。Timer1继续输出先前脉冲；没有新增轴状态缓存。测试覆盖亚度目标持续累积及跨整数边界。该项付出56B Flash，换取实际重复写调用消除，不声称容量下降。

## 逐模块与逐函数分类

完整186条函数/条件实现位置、分类、链接符号字节数与工作界限见 [FUNCTION_AUDIT.csv](minimal_motion_evidence/FUNCTION_AUDIT.csv)。涵盖19个生产CPP、头文件内联实现及两个ISR；Native专用与条件编译函数也列出，不冒充都存在于AVR二进制。CSV逐函数关联下列具体机制。LTO内联/合并函数只能报告承载符号，不把无法独立归因的尺寸写成0或把共享库重复相加。

|模块|分类与决定|B：删除会发生的故障；更简单替代/重复判断|运行成本及SRAM所有者|
|---|---|---|---|
|arm|A姿态接口；B最终输出、量化、HOLD|删除量化检查可能float合法而整数组合越界；删pending清除会HOLD后仍输出。请求与应用检查处于两个时刻，允许期间状态变化，不整体合并|每次最多4轴；joints76B，Servo对象12B，held/pending各1B；无IK|
|joystick|A映射；B死区/回中|删除死区导致ADC噪声运动；删除回中会自动结束时被偏杆接管。唯一PROGMEM配置替代关节中的重复字段|28B Flash表，4次ADC/采样，输入4B局部|
|manual_control|A两模式；B连续目标/武装/dt上限；C阻塞FK|去连续目标会低速整数截断而不动，去时间清理会恢复跳跃。阻塞时不用FK已替代|manualTarget16、cartTarget12、时间4、mode/armed各1；每有效CART tick至多1IK|
|motion|A任务；B owner/预检/确认/恢复；C取样计时|删owner会互相覆盖目标；删预检会动到半路才发现不可达；删最终确认会跳过未真正提交的必经点；取消必须先抬笔|task104B；预检每loop1子段；执行上限16几何段/tick，不递归|
|kinematics|A FK/IK；B数值/解支/FK复核|错误解支或工具变换误差可能产生另一姿态；没有等效的“只检查关节角”替代空间正确性复核|base最多3候选；solveFK3D自身栈58B，inverse3D59B；数学库共享|
|path_planner|A曲线/图形|公式不改；CurveWeights一次算XY共用，已经避免重复权重；不可用直线代替过五点曲线|16B形状表；CurveWeights32B局部，centripetalSample自身栈70B|
|teaching|A五点/撤销/顺序；B高度与互斥|否则运行途中改Z/点序，或缺纸面参数就画；XY+统一高度已是最小副本|points40/order5/count1/settings15/active+marking2；最多5点扫描|
|recording|A时间轴；B容量有界/误差界|去压缩最坏动作会不足15秒；去误差标志谎报精度；去时间戳回放失时。无第二数组|80*6=480B；status6、时间8、reportedSecond1；压缩最多78*4残差比较+79帧搬移|
|pick_place|A八阶段编辑/执行；B双槽/CRC/互斥|断写会损坏唯一轨迹或未示教数据被执行；固定常量是替代，但R3条件未满足|EEPROM457B含工作区，RAM8B状态；SAVE每loop至多1字节；LOAD同步147字节写的阻塞未修|
|app_controller|A上下文四键；B优先级/互斥|不区分绘图/录制/示教会让B3取消变回放；共用分发避免第二套按钮状态机|nextObject1B，Action2B局部；noinline隔离取消深栈|
|commands/command_text|A命令/状态；B严格解析与公平性|坏数字、超长输入成为运动或PC吞掉整个循环；统一parser已共享|serialBuffer64B+索引/丢弃2B；每轮最多64字节、7 token；pollSerial自身栈99B|
|button_link/button_protocol|A Nano入口；B会话/CRC/序号/超时|噪声/重启/重复事件误触任务；CRC与epoch各解决不同故障，不合并|8B帧+9B元数据；每loop16字节，CRC48位迭代；无第二业务状态机|
|pin_uart|B D10接收/溢出|漏采样或溢出后旧队列被当新会话；Timer2不替换Servo Timer1|16B环+5B状态；ISR每次1位；PCINT0向量78B，Timer2向量164B（完整链仍须硬件测）|
|hardware_config|A实测值；B有效门禁|空白/非法配置启动运动；常量折叠比新增缓存状态更简单|hardwareConfig112B data；最多2HOME中间点；不改实测值|
|control_settings|A独立速度；B有限/范围|零/NaN破坏时长计算；手动、纸面进给、自动任务有不同单位和时间合同，不能合成一个旋钮|3*float=12B，常数比较|
|control_error/serial_format|A必要诊断；B溢出检查|删错误会无法判断拒绝原因；float转整前溢出检查防无定义转换|error1B；共享格式化，非printf；无周期姿态刷屏|
|main|B非阻塞调度/唯一提交|去phase保持会叠加计算时间，补帧突发会多次输出；现有逻辑已最小，不新增调度框架|lastControlTime4B；每轮一帧上限；TimingStats仅诊断宏启用|

以上B的独立Flash尺寸见CSV；总数据与库符号见r2-symbols.txt。CPU给的是源码工作上界，不是虚构的微秒测量。并非所有B都有独立符号或独立全局数据；共享位置已明确。

## 逐状态与结构体决定

- MotionState：Idle(A可接新任务)，Validating(B预检占owner且不输出)，Running(A)，Paused(B保留游标和resumeState)。不能把Paused当Idle或把HOLD硬件标志当任务状态。额外resumeState仅1B，区分恢复预检与执行。
- TaskKind：None；LinePreview/LineExecution；JointMove/CartesianMove；Home/PickPlace/Playback；DrawLine/DrawShape/Polyline/Curve全部A。后四只决定取点方式，不创建独立执行状态机。LinePreview已确认诊断，不擅自删除。
- DRAW共同阶段由index派生：HOME(+via)、HOME抬至travelZ、TRAVEL至P1、PEN DOWN、STROKE、PEN UP、RETURN。未增加一个和index重复保存的phase字段。
- MotionTask：status/resumeState B；origin/segmentStart/segmentTarget A/B；target与cart union C已合并；elapsed/lastUpdate/duration B保留进度、时钟和一段限速；validation counters/segmentDistance C互斥复用；dwellRemaining A抓放等待；objectIndex/drawCheckpoint互斥复用；shape A；recoveryLift B重复HOME仍抬笔；segmentLoaded B不重复初始化进度；endpointQueued B必须输出确认。
- Workflow：Normal/TeachSetup/TeachMark全部A；由active/marking派生而非再存状态。AppAction所有已支持动作A，Action为2B局部命令，不是第二任务。
- RecordState Empty/Arming/Recording/Ready均A/B；ARMING不计入duration；status的errorBound/degraded为B，明确压缩误差。RecordedFrame=6B（编译断言），RecordingStatus=6B（AVR字段布局）。
- JointId、JoystickAxis、ControlMode、ShapeId为A索引/模式；ControlError前10值A诊断，尾3死项D已删除且不重排前10值。
- ToolModel Uncalibrated/FixedPlanar/DistalLink：有效性B，生产FixedPlanar物理值不改；另一个模型在const配置下可折叠，不冒险删除已测公式。
- JointControl由25降19B（4轴nm100→76）；Pose16、Point2D8、Point3D12、FKResult20、IKResult21、IKResult3D13、JoystickAxisConfig7、JoystickInput4、JointCalibration13、CartesianCalibration27、HardwareConfig112、TeachPoint8、TeachSettings15。除nm/static_assert注明者，其余为AVR的1B对齐字段推算；不能套用桌面sizeof。
- TeachPath D：未实例化61B类型，本身0运行时成本，留存不作为收益；CurveWeights C已共享XY权重；TimingStats C诊断专用，生产没有实例。EEPROM双槽header/payload是显式字节格式而非可能受padding影响的struct。

## 安全与回归证据

- Native `r2-final-native.log`：1,376,687 checks，覆盖JOINT/CART映射、限位/量化、NaN/Inf、四键上下文、ARMING/20秒复杂录制、回放时间轴/暂停、EEPROM所有断写边界与损坏拒绝、绘图取消和重复HOME。计数随输出帧减少，不能用数量作为逐项证明；看test/native/sprint_tests.cpp中的独立场景断言。
- `minimal_tests.cpp`：20mm FEED15/25完整float/整数输出序列，四种DRAW在FEED25/100的统一Z、五点精确采样经过(<0.05mm FK容差)、实际输出曲线偏差、调用次数。5026断言。
- `speed_tests.cpp`：5789断言；每tick float变化受60度/s约束、最多一帧、端点确认、暂停5秒、millis回绕、延迟tick、取消抬笔HOME、动态限位失效立即HOLD。
- `draw_t3_tests.cpp`：6102断言；两次捕获、忙/marking时拒绝保留旧Z、整数化高度反向扫描及实际路径阶段。没有通过改变模型解决物理高度误差。
- D10寄存器3273 +协议/真实PC loop35：正确帧、CRC、重复/错序、重新同步、超时/溢出和USB STATUS。Nano代码未改；不是实机通信验收。
- 未运行过期 `test/capacity_check.py`：它要求已不存在的空标定注入标记和旧固定pick表，不是当前Bringup有效构建入口。本轮完整配置是实际 `uno_nano_link`，四轴/EEPROM/绘图/D10均保留。

## 栈与未完成实测

[stack-frames.csv](minimal_motion_evidence/stack-frames.csv) 来自两个实际AVR ELF的共享函数序言，单位包含局部+保存寄存器，不含返回地址/后续callee/ISR。
execute70→53，sampleGoal85→87：execute+sampleGoal小计减少15B；main由48增至51B（输出去重引起编译器保存寄存器变化），故同一绘图链已列帧净减少12B。pollSerial99、cancelAndReturnHome37、inverse3D59、solveFK3D58、centripetalSample70不变。PC BTN3→取消→IK未加调用层，但其常驻main帧增加3B；静态SRAM节省24B，已计部分净余量改善21B，不把这当完整栈证明。静态空余765B不是已证明的栈上界；还需ISR打断和libm临时栈/实际水位测量。
尝试将applyPoseImmediate标为noinline来隔离主循环栈，完整D10达到32262B（超限6B），已撤回，记录在r2-rejected-noinline-build.log。

最坏控制周期微秒数**未验证**。源码仍有最多16几何段/控制tick保护、40ms有效积分上限；极端短曲线可能触顶减速，未删此防失控机制。所有Native耗时不等于16MHz Uno耗时。现有MEARM_TIMING_DIAG可用于单独测周期，但D10诊断固件是否装得下尚未构建验证，不能声称有完整硬件WCET结果。

## R3 标定后退役方案（未实施）

1. 在当前校准分支实际验收A/B/C各8阶段：位置、速度、等待、夹爪阶段、从允许起态进入、暂停取消。现没有用户提供的全部已验收表，阻止退役。
2. PC PICK SHOW A/B/C导出24条四轴整数角、速度、10ms等待和完整mask；保存人工标定记录/来源commit/CRC。不要导出Native合成EEPROM。
3. 创建保留运行时编辑/双槽的标定专用分支；新运行分支用PROGMEM只读147B表与有效标记，实现相同getPickStage/speed/dwell接口。先运行相同预检与按钮回归。
4. 明确用户同意删除PICK SET/SAVE/LOAD/CLEAR/SPEED/WAIT后，才删解析和EEPROM读写；保留SHOW/状态诊断或给固定表版本。未获同意不改变命令合同。
5. 分别实测Flash/SRAM与栈；常量表增加Flash、可减少EEPROM阻塞和持久化状态，不能预先承诺净节省值。撤回时只切回校准分支，不覆盖实测文件/EEPROM。
