# TEACH 轴向调查与固定绘图速度（2026-10-14）

基线：codex/r2-field-fixes-2026-10-14，c9643e71101bb4715cb438d6bcc7816fad26462b；生产代码c0efb82。初始工作树干净。原HEX哈希记录于teach_fixed35_evidence/baseline-binaries.txt，备份在.pio/teach-fixed35-baseline。没有回读实机Flash，用户烧录固件身份不能由本地HEX单独证明。

## 轴映射结论

当前源码没有TEACH特有的轴交换，故不修改ADC或方向标定。

|ADC|逻辑通道|中心/死区/方向|ADC增加100后的量化值|CART作用|
|---|---|---|---:|---|
|A0|lx|506/20/+1|-1|Y减少|
|A1|ly|519/20/-1|+1|X减少|
|A3|ry|519/20/+1|-1|Z增加，Mark时固定teachZ|
|A2|rx|515/20/+1|-1|普通CART夹爪角减少，TeachSetup/Mark屏蔽|

joystick.cpp的枚举配置顺序LX/LY/RY/RX，而JoystickInput字段顺序LX/LY/RX/RY；readJoysticks已经明确重排为{axes[0],axes[1],axes[3],axes[2]}，不能再次交换。名称代表软件配置，物理方向需与实物线序核对。quantizeAxis先以中心作差，abs<=20为0，abs>=500为高速档，再按原正ADC→负运动及每轴符号处理；测试保留全部参数。

manual_control.cpp在CART中复制连续cartTarget，ly改X，lx改Y，ry改Z；解IK3后经prepareOutputPose/requestPose，再由控制tick输出整数角。TEACH J3条件只保护Gripper赋值，不涉及前三轴。TEACH START先切CART，置active、清marking，并syncManualTarget：从commandedPose做FK，清除积压时间，armed=false；需要全部四轴回中后才接受移动。Motion忙、held或pending同样禁止积分。TEACH MARKING则固定Z为teachZ，这是进入存点后的既有契约。

高度captureHeight取commandedPose经FK所得Z，即已经输出的整数软件指令，不是连续目标、待提交目标或实测反馈。pending/忙时返回BUSY并保留旧值。STATUS的Z三项是已存的drawZ/teachZ/travelZ，并不是实时笔尖Z；mask7只表示三次捕获有效，不表示高度顺序正确。MARKING仍要求drawZ<teachZ<=travelZ，当前接受姿态需接近teachZ，且摇杆回中。同高度拒绝是正确行为。error可能保留以前的错误，需同时记录对应命令即时OK/ERR，不单凭STATUS旧error判断当前轴是否失败。

新增test/native/teach_axis_tests.cpp直接调用真实ADC读取、手动积分、IK、安全输出和捕获函数；未注入替代运动学。在原标定及日志姿态J=74/64/95/85，从A3输入±100持续400ms：初始FK=(120.681,-16.6546,76.7027)，连续目标为同XY、Z=66.7027或86.7027；整数FK分别约(119.337,-16.5212,66.8030)、(119.212,-16.5088,86.1667)。整数化存在毫米级横向偏差，但不是把Z改成前后轴。普通CART与TEACH同输入的连续和整数关节目标逐项一致。另一个姿态也测双方向。

回归464断言通过：ADC/死区映射、TEACH前后相同Z效果、Z改变捕获值、TEACH J3冻结、显式PC J3允许、EXIT恢复、重新武装回中、三次不同真实接受高度进入MARKING、三次相同高度拒绝、pending及Motion忙时拒绝捕获。日志teach_fixed35_evidence/teach-axis.log。

## 最小现场观测（需用户自行执行）

1. 保存烧录环境与HEX哈希；115200串口，静止时STATUS。当前本地构建记录无法证明已烧录二进制。
2. 普通CART先回中，STATUS记J；只移动右摇杆一个方向约0.4秒，回中后STATUS记J，记录物理方向。另一方向另测，不同时动作多个通道。
3. 同一安全姿态TEACH START后四轴回中，再按同样步骤。TEACH DRAWZ捕获前后都保存J及即时OK/ERR。理想情况下，A3输入导致J1/J2配合变化，A2输入在TEACH下不得改变J3。
4. 若J不变：优先检查是否未回中武装、held、非空闲任务、输入落在死区或目标被IK/限位拒绝；若只有物理前后明显而J变化，使用记录J按当前FK重算XYZ区分量化和机械模型误差。不得用目测结论交换ADC。
5. 若需要确认实物轴，断开机械动力后由用户检查实际摇杆输出脚至A3/A2的接线，或用独立ADC观测程序；本次主控不增加常驻诊断状态。不要为读ADC上传覆盖现场固件而漏记版本。
6. 三次相同Z仍不得进入MARKING；须实际改变软件接受高度并回中后分别捕获draw、travel、teach，最后停在teachZ再MARKING。

本地测试不能证明物理方向、供电或实机TCP模型正确；当前P0未复现为软件映射回归。

## FEED退役与固定35mm/s

生产代码提交79e083b1a2f870e8ef69aaa4322e0c63b8da5b2a。删除FEED命令入口/名称、HELP说明、STATUS动态feed字段、drawingFeed()/setDrawingFeed()和4字节可变状态。输入FEED、FEED 35等现在统一ERR INVALID，不改变运动或配置。H/L和SPEED原语义保留；ABC、EEPROM、录制回放未改。

include/control_settings.h新增单一描线常量kDrawSpeedMmS，默认35.0f；开发测试可通过-DMEARM_DRAW_SPEED_MM_S=25.0f覆盖，静态断言必须为正数。没有运行时setter或第二份动态配置。用户原消息末句停在“为开发测试保留编译期”，据此提供编译期覆盖，不增加新的串口调参入口。

35只影响LINE/SHAPE/POLY/CURVE的STROKE。原代码以动态feed同时约束落笔/描线/抬笔，为满足本次“非描线逻辑不变”，将落笔/抬笔旧默认25固定为kPenTransitionSpeedMmS；这是独立阶段的既有时间规则，不是另一份描线状态。HOME/TRAVEL/RETURN仍按原关节速度，抓放/回放时间不变。

Fast环境MEARM_FIVE_FAST继续仅让POLY/CURVE STROKE不计额外60deg/s规划时间；位置/组合限位、IK分支、整条预检和最终整数输出检查未变。POLY P2/P3/P4仍保持80ms，CURVE不加保持。LINE保留关节时间上限。没有delay、第二Servo写入所有者或额外全局数组。

## 新构建与回归证据

最终构建代码版本79e083b，后续交付提交只修改测试/文档；build-final.log对应相同生产源码。没有使用合成标定。使用的完整功能D10环境为uno_nano_link、uno_nano_five_fast，uno不启用Nano接收。

|环境|旧Flash|新Flash|Flash差值|旧SRAM|新SRAM|SRAM差值|
|---|---:|---:|---:|---:|---:|---:|
|uno|31294|31062|-232|1245|1241|-4|
|uno_nano_link|32112|31882|-230|1299|1295|-4|
|uno_nano_five_fast|32046|31916|-130|1299|1295|-4|

Fast ELF：.text31662+.data254=31916 Flash，.data254+.bss1041=1295 SRAM；Flash余量340B，静态SRAM余量753B。标准D10 Flash余量374B。没有实测栈水位或16MHz控制WCET，不能根据753B静态余量宣称运行时栈安全。PC BTN3深链未扩展；移除setter/状态打印减少相关逻辑，但链接体积不等于执行时长证明。

Fast HEX SHA256：be029cf913eca439387edb6730f1806b84f88b52fbf8268ffc6b5e64e1a4c87d。
标准D10：97d092ea7670d49277dc9eab8fe02c370a2bda099fcd7e2f2390be5ad5123c12。
uno：2594d2595a9e177eb9309606192f1b5b6746a7b041aeff1a5fb140fc18d5efba。

|测试|普通35|Fast35|关键断言|
|---|---:|---:|---|
|完整Native|1375676|1375676|手动/按钮/安全/录制回放/任务互斥；H/L与SPEED不改绘图耗时；FEED全部拒绝；HELP/STATUS无FEED字段|
|TEACH真实ADC|基线464|最终466|相同输入XYZ一致；连续FK与整数FK；返回原TEACHZ时保留三高度进入MARKING；相同高度拒绝|
|fixed_speed_tests|25327|25567|各实际加载段duration严格等于距离和关节时间公式，落笔/抬笔25，STROKE35|
|field_speed_tests|2626|2656|真实五点的整数输出、端点、保持、暂停恢复|
|minimal_tests|2546|2558|LINE/SHAPE/POLY/CURVE共同Z、几何路径、五点经过、安全输出及重复整数输出过滤|
|speed_tests|4976|4876|取消HOME、时间回绕、晚tick、过点、非实验路径关节时间限制|
|draw_t3_tests|5598|—|不同DRAWZ、FK/IK、量化、高度及端点确认|
|teach_grip_tests|101|—|Setup/Mark锁夹爪、PC J3及EXIT|
|field_tests|3523|—|148边界回放、held HOME、N门禁、曲线PATH ERR|
|D10模型/业务|3610/38|相同驱动|HELLO/CRC/epoch/sequence、重复与缓冲异常、二次B2停止|

25/80编译期覆盖的阶段测试分别31518/17200断言通过。标准35/Fast35/Fast25/Fast80共24个非描线阶段的PLAN行逐行一致，见final-comparison.txt。速度变体从原“同进程动态setter”改为不同编译配置，故单次断言数与历史多FEED循环不同；安全/路径断言继续保留。

真实五点（上轮用户坐标、高度82.53/86.71/94.29）模拟20ms控制tick：

|轨迹|旧Fast25描线ms|新标准35描线ms|新Fast35描线ms|新Fast35总任务ms|
|---|---:|---:|---:|---:|
|20mm LINE|800|580|580|2900|
|五点POLY|6020|4140|4380|6620|
|五点CURVE|6040|4360|4360|6600|

POLY Fast含3×80ms保持，CURVE保持0；本组最大相邻整数关节差2度。这里高度是测试输入，未写入生产标定，不能用旧纸面高度替代实机重新确认。理论20mm/35=571.43ms，整数ms段时长与20ms确认节拍下为580ms。总任务含HOME/TRAVEL/PEN DOWN/PEN UP/RETURN，不应当作纯描线速度。

这不是桌面CPU耗时或真实舵机到点时间；固定保持也不是位置反馈。日志中的PLAN测试用1ms模拟步进观察每段加载，只用于公式核对，实际主控tick仍20ms。

## 本轮实机复测最短入口

构建：pio run -e uno_nano_five_fast。确认本轮Fast HEX哈希；当前操作指令无需FEED。未经用户批准不烧录，本轮没有上传行为。

1. 静止时HELP/STATUS：HELP不含FEED，STATUS不含feed=；FEED 35应ERR INVALID且不运动。
2. MODE CART，摇杆回中；按前述轴向观测步骤记录TEACH前后J及动作方向。TEACH期间J3应保持，显式PC J3仅在用户主动调夹爪时使用，避免掉笔。
3. 按真实纸面依次捕获不同DRAWZ/TRAVELZ/TEACHZ，返回TEACHZ回中后MARKING。mask7且三值相同必须继续UNCALIBRATED。不要因测试失败放宽高度关系。
4. 用当前真实点集进行已通过的LINE，再POLY/CURVE，记录PATH VALID、DONE、实测时间和纸面痕迹；出现UNSAFE保留PATH ERR及LIST/STATUS。旧CURVE现场失败仍未由软件复现，本次没有删除检查或声称根因已消除。
5. 复测绘图B1/B2/B3、正常B4 HOME、普通模式B2主动结束录制及B3回放。35mm/s属于待实机验证的实验目标，不能把构建成功当作速度/可靠性验收。

仍保留B2环缓冲修复、148量化修复、N退化门禁、曲线失败点诊断及TEACH夹爪冻结。当前映射没有软件交换证据，故未改引脚/标定/运动学。等待用户提供最小现场记录；不进入CORE-FROZEN、不删除ABC EEPROM编辑器。
