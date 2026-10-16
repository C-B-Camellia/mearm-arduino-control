# H4仅本地翻页及映射修订（2026-10-16）

本轮基于上一轮已通过构建的双板业务命令版本当前工作树实施，仅修改副板。未烧录、未提交/推送、未清空EEPROM。Uno生产源码、include、platformio.ini及现有HEX修改前后SHA256全部一致，没有重建Uno AVR固件。原Dual_Board_Command_2026-10-16.md中H4自动发命令的描述为历史行为，以此修订为准。

## 行为对照和改动

H4循环仍为NORMAL→BASIC→CAL→TEACH→DRAWING→NORMAL。旧版进入CAL发送TEACH START，进入TEACH发送TEACH MARKING，返回NORMAL发送TEACH EXIT；本轮全部删除。H4分支没有enqueue，仅selectMode同步更新本地映射及页面选择；进入DRAWING保留原本地TEACH返回页记录。

没有新增状态机、修改OLED驱动或业务协议。原显示页缓冲的renderedPage是绘制快照，不参与按键判断；逻辑页面与映射同时更新，实际I2C像素刷新仍需要时间，不声称物理屏幕瞬间切换。H4不清空原发送队列；之前已排队的业务仍可发送，每秒Hello也保留，它们不是由H4生成的新业务。

为满足“未列出的H1/H2/H3不发送指令”，同时修正旧版部分键按下即发送短按的行为：所有短按都在消抖松开后发送；达到800ms则只处理长按，未分配长按无动作。DRAWING的HOLD/CANCEL也须短按松开才发送，按住不会先发短按动作。原25ms消抖、800ms阈值、开机按住抑制、时间回绕、非阻塞循环、旧页按住输入隔离均保留。

## 最终五页映射

| 本地页 | B1短按 | B2短按 | B3短按 | B4短按 | H1/H2/H3 |
|---|---|---|---|---|---|
| NORMAL | PICK NEXT | REC | PLAY | HOME | H1:TEACH START→CAL；H2:本地BASIC |
| BASIC | DRAW LINE→DRAWING | DRAW N→DRAWING | 本地NORMAL | HOME | H2:本地NORMAL |
| CAL | TEACH DRAWZ | TEACH TEACHZ | TEACH TRAVELZ | TEACH EXIT→NORMAL | H1:TEACH EXIT→NORMAL；H2:CLEAR；H3:TEACH MARKING→TEACH |
| TEACH | MARK | DRAW POLY→DRAWING | DRAW CURVE→DRAWING | UNDO | H1:TEACH EXIT→NORMAL；H2:CLEAR→CAL |
| DRAWING | HOLD | GO | CANCEL | 无 | H2:仅本地返回原BASIC/TEACH |

所有页H4只循环本地页面和映射。未列出的H1/H2/H3无动作。OLED底部原H1/H2/H3提示保留、不新增H4提示。CAL/TEACH页面不证明主控处于对应业务状态：用NORMAL H1真正进入TEACH START，用CAL H3真正请求MARKING，PC看到TEACH READY后才MARK。DRAWING H2返回前从PC确认DONE。

## 回归结果

- 副板无屏Native 1056检查、OLED Native 1882检查通过。五页分别长按H4持续2600ms只切换一次，零业务帧、命令sequence不变，松开零短按；同时检查OLED最终渲染标题、页编号和映射一致。
- 从开机页逐次物理H4进入每页再测B1/B2/B3/B4，包括一整圈返回NORMAL，全部对应表中命令；BASIC B3无帧。完整长短按矩阵覆盖真正的H1/H2/H3入口以及未分配长按无帧。
- DRAWING H2从BASIC/TEACH恢复原页且无帧；满队列H4依然纯本地切换。原开机按住、800ms边界、消抖、回绕、序号、epoch及模拟OLED刷新检查通过。
- 现有Uno源码Native测试：原双板端到端2872检查通过，使用本轮副板实际模拟发送字节完成原三高度/示教/绘图流程。新增244项边界检查，总计3116通过。
- Uno NORMAL且副板CAL：三高度请求返回TeachState，状态/高度无变化。未MARKING/READY时MARK返回MarkState；MARKING等待回中时MARK返回Busy，不记录点。
- Uno DRAW LINE预检中及执行中：模拟副板NORMAL发送PICK NEXT/REC/PLAY、CAL发送三高度/TEACH EXIT，均Busy，任务、输出、录制和示教状态不改变。
- HOME是原有允许的受控取消回家，HOLD/GO/CANCEL也是合法控制；不能将这些请求视为必须拒绝。没有发现本轮需要扩展Uno安全逻辑的问题，未改主控。

测试入口：`./nano/test/run.ps1`；边界复现：`./exports/h4-local-only-20261016/run-uno-safety.ps1`（先执行副板测试生成panel-wire-True.bin）。边界测试从现有remote_command_tests.cpp复制并追加断言，仅用于电脑测试、不编入固件。日志与本轮完整diff均在上述证据目录。

## 真实构建与兼容性

实际执行`pio run -d nano -e nano_oled`成功。board=nanoatmega328new，工具链/编译参数未改。

| 固件 | Flash | 静态SRAM | 前后变化 |
|---|---:|---:|---|
| 新副板nano_oled | 12720/30720 B，41.4% | 919/2048 B，44.9% | Flash减少104 B，SRAM不变 |
| 既有Uno生产固件 | 此前31330/32256 B | 此前1293/2048 B | 现有HEX哈希不变；本轮未重建 |

新副板HEX：`exports/h4-local-only-20261016/nano-oled-h4-local-only.hex`，SHA256：`fca4745f31291b99fcd8622741f7d762c98dc30a3fa6fb851e0840e6f99691aa`。

既有Uno HEX SHA256：`f99f721c4a5dc6e2e5ebd6f16b79ca7648971aaccb297ad28389e6305d06810a`。

可直接配合这个此前构建的Uno业务Command=2固件，不需要重新生成或重烧Uno。两端business_commands.h和button_protocol.h逐字节一致；8字节、9600 baud、CRC8、Hello/epoch/sequence及ID不变。更早Press=1旧按钮主控不兼容，不应混用。

## 人工验证与回退

1. 本轮只需人工更新副板，选择真实COM；遵守副板USB和外部7.4V供电互斥、烧录断开板间线的原要求。
2. 确认开机NORMAL；每页H4长按一页、>2秒不连翻，松开不HOME/UNDO/EXIT，PC无由H4新增的OK REMOTE或运动。
3. 确认各页短按松开后发送，重点体验HOLD/CANCEL响应与OLED刷新期间扫描。H4翻CAL/TEACH不会真正进入示教，须用H1/H3流程并看PC READY。
4. 实物OLED重绘/按钮响应、UART与Servo并发、电源及机械运动仍待现场验证；没有返回线，不显示Uno真实接受/完成状态。

备份panel-before.zip含旧副板源码和HEX，可仅回退副板；无需改Uno或擦EEPROM。未执行任何硬件操作。

修改文件：nano/src/main.cpp、nano/test/nano_tests.cpp、nano/README.md，另新增本报告及exports/h4-local-only-20261016证据/测试文件。其他既有未提交改动未覆盖。
