# 自制ATmega328P四键OLED副板

构建：`pio run -d nano -e nano_oled`；无屏：`pio run -d nano -e nano`。board=nanoatmega328new，烧录115200，运行UART9600。A0..A3外部上拉低有效；D1 TX→Uno D10，共地、分别供电、不互接5V。OLED原A4/A5与驱动不变。

| 本地模式 | B1短按 | B2短按 | B3短按 | B4短按 | 其他长按（≥800ms） |
|---|---|---|---|---|---|
| NORMAL | PICK NEXT | REC | PLAY | HOME | H1→TEACH START/CAL；H2→BASIC |
| BASIC | DRAW LINE→DRAWING | DRAW N→DRAWING | 返回NORMAL | HOME | H2→NORMAL |
| CAL | DRAWZ | TEACHZ | TRAVELZ | TEACH EXIT→NORMAL | H1退出；H2 CLEAR；H3 MARKING→TEACH |
| TEACH | MARK | POLY→DRAWING | CURVE→DRAWING | UNDO | H1退出；H2 CLEAR→CAL |
| DRAWING | HOLD | GO | CANCEL | 无动作 | H2仅本地返回原BASIC/TEACH |

所有模式H4同时切换页面和映射：NORMAL→BASIC→CAL→TEACH→DRAWING→NORMAL。H4在所有页面均不发送业务命令，不改变Uno实际状态。TEACH START仅由NORMAL H1触发，MARKING由CAL H3触发，退出使用CAL/TEACH H1或CAL B4短按。一次长按只触发一次，不同时触发短按；所有键短按在消抖松开后发送；未列出的H1/H2/H3长按无动作。开机按住须先松开再按。

这是本地选择，不是Uno状态：发送MARKING后必须看PC的TEACH READY再MARK；绘图结束须看DONE，才用DRAWING H2返回。请求被拒或丢帧时，页面仍可能已经改变，不能把页面当成确认。任务运行中切页不绕过Uno BUSY、安全预检或限位。OLED右上L1..L5表示本地模式，!表示发送队列曾满；没有五点计数或成功状态。

业务帧type=2，Hello=0，8字节CRC8协议；不兼容旧type=1按钮主控，须配套升级。没有返回线。

当前OLED构建12720 B Flash/919 B SRAM；无屏上轮3256/262，本轮只跑无屏Native测试，未重建无屏AVR固件。测试入口 `./nano/test/run.ps1`。本轮未烧录。Uno主控源码位于[Bringup分支](https://github.com/C-B-Camellia/mearm-arduino-control/tree/codex/interaction-patch-2026-10-15)，与本副板协议兼容。

2026-10-16 H4局部修订报告：[H4_Local_Only_2026-10-16.md](../docs/H4_Local_Only_2026-10-16.md)。原双板报告中的H4自动发送行为已被此修订替代。
