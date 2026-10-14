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
