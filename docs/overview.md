# MeArm 代码地图

<div class="map-intro">从模块看整体，从共享字段找横向联系，再沿函数调用追踪执行路径。</div>

## 选择一个入口

<div class="map-grid">
<a class="map-card" href="main_8cpp.html"><strong>主循环</strong><span>setup / loop：初始化、串口轮询与控制周期调度。</span></a>
<a class="map-card" href="commands_8cpp.html"><strong>命令处理</strong><span>解析串口输入，提交目标与执行任务控制。</span></a>
<a class="map-card" href="arm_8cpp.html"><strong>机械臂状态</strong><span>关节目标、安全检查、待执行姿态与舵机输出。</span></a>
<a class="map-card" href="motion_8cpp.html"><strong>运动任务</strong><span>轨迹校验、逐点执行、暂停、恢复与取消。</span></a>
<a class="map-card" href="kinematics_8cpp.html"><strong>运动学</strong><span>正解与逆解：关节角度和坐标之间的转换。</span></a>
<a class="map-card" href="config_8h.html"><strong>配置参数</strong><span>串口、控制周期与轨迹步长等常量。</span></a>
</div>

## 先看共享状态

- \ref JointControl ：从 `goalAngle`、`commandedAngle`、`outputEnabled` 等字段的引用开始。
- \ref Pose ：查看完整姿态如何在请求、安全校验与输出之间传递。
- \ref MotionStatus ：查看任务状态与轨迹进度。

<div class="map-note">字段页面的引用列表表示“哪些函数涉及这个字段”，不自动区分全部读写。要判断是谁修改、谁使用，请点到源码中的具体位置。</div>

## 再看执行路径

1. 打开 \ref loop() ，先看调用关系图。
2. 沿 \ref pollSerial() 进入命令处理，沿 \ref updateMotion() 进入运动任务。
3. 在 \ref requestPose() 和 \ref applyPendingPose() 之间查看目标姿态的提交与应用。
4. 对不理解的函数，同时查看调用图与被调用图，确认它在上下游的位置。

<div class="map-note">调用图描述静态调用关系，不代表实际执行先后或一定执行的分支。参数、共享状态和控制条件的影响需结合源码理解。</div>
