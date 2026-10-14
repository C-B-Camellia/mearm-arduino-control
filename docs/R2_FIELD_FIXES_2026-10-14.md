> 历史阶段记录：其中 FEED 调参步骤已退役；当前固件固定35mm/s描线，见 [本轮接口与测试说明](TEACH_AXIS_FIXED35_2026-10-14.md)。下文保留用于追溯当时的源码与构建证据。

# R2 实机故障修复与速度实验交付（2026-10-14）

## 版本及证据边界

工作区 D:/Develop/MeArm-Bringup-2026-10-12；独立分支 codex/r2-field-fixes-2026-10-14。
原分支 codex/minimal-motion-2026-10-14 保留在 08b1078ea8beb44018a2e0986ccb65baaa25ba68，R2 代码检查点7513888ba97bf0b4af395f65c4d52611205da937。
本次生产代码最终提交 c0efb82ae81450f7b983b7e7bf908dc5b45f8a79；其后交付提交仅测试和记录。用户实际烧录版本未知，不能把源码复现等同于现场根因确认。
没有烧录、硬件操作、标定改变或 CORE-FROZEN；开发工程未修改。

## P0 根因与修复

|问题|源码证据及结论|最小处理、提交|测试与未解决项|
|---|---|---|---|
|B2二次按下不保存|按钮业务已允许 Recording 下B2；pin_uart.cpp:10旧16字节环仅容15字节，周期HELLO8字节紧邻PRESS8字节时可溢出，事件丢失|91c7893：RX环32字节、容量31；CRC、epoch、sequence、超时同步不变|寄存器模型3610断言，协议/真实PC入口38断言；含正式录制12秒后HELLO+PRESS停止、重复序号过滤。证明软件可丢帧及修复，不证明唯一现场原因；电气/ISR延迟仍待实测|
|回放及异常后HOME|arm.cpp:135 prepareOutputPose独立向下量化；安全端点(74,42,106,58)→(75,43,105,58)，101个插值中原99个量化成J1+J2=147|3558a72：先原浮点合法检查，量化若低于148则补足Elbow，最后完整整数合法检查仍执行|修复后0/101失败；15秒边界录制压缩/回放、5秒暂停恢复；多组148边界从held经HOME成功。日志姿态在原20ms步长也可成功，5ms首步揭示问题，不能声称所有HOME拒绝已复现|
|N看起来一条线|无参数SHAPE以有序P1/P2为矩形对角点，实际宽40.73、高4.25mm；N顶点顺序本身正常|ee9ee1a：startBasicShape(motion.cpp:424)任一边<5mm返回新ERR GEOMETRY(枚举10)，任务未启动，无输出；没有自动扩大图形|实际输入薄矩形拒绝；显式五点包围框N可预检并执行。5mm为明确软件退化门槛，不是已实测的分辨率；现场只画线的机械原因不能仅凭源码证明|
|CURVE UNSAFE|相同五点及Z=82.53/86.71/94.29，当前源码CURVE179waypoints、276预检调用通过；现场264不吻合，可能版本、顺序或输入不同|229ddac：失败时输出PATH ERR waypoint/substep，substep0表示采样/IK阶段，之后为预检插值子步；不更换向心Catmull–Rom、不放松预检|偏移到不可达工作区测试PATH ERR 3/0且零Servo输出；现场故障尚未复现，因此没有虚构曲线修复。需复现时LIST、STATUS、具体命令、PATH ERR|
|TEACH掉笔|manual_control.cpp:85原只屏蔽TeachMark，未屏蔽TeachSetup|059ea32：teachingActive期间不积分摇杆J3；显式PC J3仍按原安全接口允许；EXIT恢复|101断言：Setup/Mark冻结、PC J3更新后保持、EXIT恢复。无新增夹爪跳变|

录制格式、容量、误差上界算法未更改。err<=149 DEGRADED是保守压缩误差上界，不能当作每个实际输出误差149度，也不能因此认定录制保真足够。安全合法性修复不等于恢复复杂录制细节。帧删除不修改保留角度；本次定位的非法姿态发生于量化插值。

## 提交及资源变化

下表均为实际链接的uno_nano_link，单位字节。日志在r2_field_evidence，非源文件行数估算。

|提交|变化|Flash|静态SRAM|
|---|---|---:|---:|
|08b1078|原R2|32162|1283|
|059ea32|TEACH夹爪|32162|1283|
|3558a72|边界量化|32228|1283|
|91c7893|RX缓冲|32228|1299|
|305d458|精简HELP及匿名双数字IK快捷入口|31980|1299|
|ee9ee1a|N几何门禁|32044|1299|
|229ddac|预检失败定位|32120|1299|
|62c2439|独立五点STROKE实验|32168|1299|
|c0efb82|仅降低手动低档|32112|1299|

HELP现在列出命令组，原命令功能仍保留。唯一移除的解析能力是匿名“42.7 51”IK入口，使用显式“IK 42.7 51”。FEED暂保留用于A/B；没有实机验证之前不删除。EEPROM编辑器保留，R3退役继续等待ABC实测。

最终构建（final-build.log）：

|环境|Flash/32256|SRAM/2048|Flash余量|
|---|---:|---:|---:|
|uno（不启用Nano链路）|31294|1245|962|
|uno_nano_link（标准D10）|32112|1299|144|
|uno_nano_five_fast（独立五点实验D10）|32046|1299|210|

标准D10 ELF：.text31854+.data258=32112；SRAM .data258+.bss1041=1299。
HEX SHA256：标准711236B577BD6B88F33AB51D56740CA1C01AA636E100F2E7CC8DCAF2780BC489；实验20C7DF11C6B0D5FB79171D7471180AF234475D9EDC0FE57FCF71ADC77E805A5B。
Map：.pio/build/uno_nano_link/firmware.map及uno_nano_five_fast/firmware.map。
较大链接符号：pollSerial3586B、main2668B、sampleGoal1770B、dispatchServiceAction1454B、execute986B、updateManualControl958B、executePickCommand934B、centripetalSample800B、observe794B、inverse3D646B。内联/共享/LTO存在，符号大小不等于模块独占成本。

静态余量749B，比R2少16B，来自UART环。没有新增局部大数组、递归或第二个执行器；沿用dwellRemaining计时。PC BTN3→按钮分发→cancelAndReturnHome→采样/IK→安全输出仍是深链；此次未测AVR栈水位，不能断言749B足够。新PATH ERR仅错误路径输出，但串口输出也有调用开销。Timer2接收、Servo Timer1、Timer0节拍配置未改；中断叠加、长ISR屏蔽、31字节队列能否覆盖最坏延迟仍需实测。

## 回归结果

- 普通及MEARM_FIVE_FAST/80ms完整Native各1375670断言通过（manual-native.log、fast-native.log）。覆盖原录制、回放、任务互斥、暂停、取消及安全检查。
- field_tests：3631断言，实际几何五点、边界回放、held HOME、N拒绝与有效N、不可达曲线零输出。
- teach_grip_tests：101；minimal_tests：5026；draw_t3_tests：6100；D10寄存器3610、协议+PC入口38，全部通过。
- 快速保持60/80/100ms分别6281/6293/6305断言；POLY在保持期间暂停1000ms恢复，CURVE运行中暂停恢复，整数目标安全且任务完成。
- 旧draw_t3参数断言曾因低档改25而失败；仅更新其已过期的低档关系断言为25/100，未删除高度/数学测试。详细日志保存。
- 没有用桌面执行耗时声称16MHz WCET。固定循环有界不等于已证明20ms期限。

## 手动参数

JOINT低45→25deg/s、高90不变；CART低50→25mm/s、高100不变；H/L倍率、死区、连续目标累积、模式同步不变。自动DRAW没有借用手动参数。低速整数台阶客观存在，重复相同整数不会生成新Servo写入。

## 尚未解决及进入下一步的条件

现场CURVE失败点、B2实际丢帧原因和已烧录二进制身份仍不确定。全部软件测试通过不能声明现场通过；N的新门禁是明确拒绝薄矩形，非保证真实N质量。完整D10余量很小，栈峰值与控制周期需硬件测量。低速抖动、负载、笔尖实际误差仍以纸面实测为准。等待用户批准烧录，不进入CORE-FROZEN。
