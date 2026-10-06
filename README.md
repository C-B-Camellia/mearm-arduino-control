# meArm Arduino Control

Arduino Uno / PlatformIO mechanical arm controller. The current version controls
the shoulder and elbow through serial commands and a nonblocking planar line task.
All four joints retain their configuration; base and gripper outputs are disabled.

## Build and tests

```powershell
pio run
powershell -NoProfile -ExecutionPolicy Bypass -File test/native/run.ps1
```

The desktop tests need a C++11-capable `g++`. To use CLion's bundled compiler:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File test/native/run.ps1 -Compiler "C:\Program Files\JetBrains\CLion 2026.2\bin\mingw\bin\g++.exe"
```

Tests compile the actual controller against simulated serial, servos and time.
They cover safety, input recovery, task validation, timing, pause/resume, cancel,
busy requests and clock rollover. They do not verify physical arm motion.
An optional `-LegacySource <saved-main.cpp>` also compares the original FK/IK and
ordinary command output with the current implementation. No legacy copy is needed
for the normal test suite. Test executables are placed under ignored `.pio/`.

## Serial protocol

Use 115200 baud, one command per line and spaces between arguments. CRLF is also
accepted. Commands are uppercase; tabs may separate tokens. Coordinates remain
the original planar **x/z in mm**, including the historical command name `XY`.
Numbers accept ordinary decimals, with optional sign; exponent notation, `nan`
and `inf` are rejected. Every command checks its exact argument count.

| Command | Behavior |
| --- | --- |
| `IK <x> <z>` or `<x> <z>` | Print IK, never move |
| `XY <x> <z>` | Queue the IK pose for the next 20 ms control interval |
| `J1 <angle>` | Queue shoulder angle, preserve the other goals |
| `J2 <angle>` | Queue elbow angle, preserve the other goals |
| `LINE <x> <z>` | Preview and validate waypoints without motion |
| `RUNLINE <x> <z>` | Validate the entire path, then send waypoints |
| `HOLD` | Pause an active task; while idle, hold the last commanded pose |
| `GO` | Resume a paused task; while idle, reapply the held pose at the next tick |
| `CANCEL` | Discard the task and pending goals, keep the last commanded pose |
| `SPEED <deg/s>` | Save a value between 5 and 90; **configuration only** |
| `STATUS` | Show command/goal angles, configured speed, task phase and progress |
| `HELP` | Show commands and limits |

`SPEED` does not yet change motion speed. Unimplemented assessment commands,
including bare `O`, `S`, `H`, `L`, are not registered or acknowledged as successful.

An active preview or execution task rejects `XY`, `J1`, `J2`, `LINE` and `RUNLINE`
with `ERR busy`, including while paused. Cancel before issuing new motion.
Starting `LINE`/`RUNLINE` also requires the previous ordinary pose request to have
been sent; otherwise it returns `ERR busy`. Diagnostic commands and speed
configuration remain available during tasks.

A line holds at most 63 printable characters, excluding the ending. On overflow,
the controller prints `ERR command too long` once and discards through newline.
The discarded suffix cannot become a separate command. Each loop processes at
most 64 received bytes and one completed line so motion can keep advancing.
Serial logging still uses Arduino's ordinary synchronous printing.

Example sequence (wait for `XY` to execute before `RUNLINE`):

```text
XY 42.7 51
STATUS
RUNLINE 71.6 42.5
HOLD
STATUS
GO
CANCEL
```

## Task behavior

`LINE` and `RUNLINE` create a path from the last commanded shoulder/elbow pose.
Validation advances by at most one waypoint per loop. For `RUNLINE`, no waypoint
is sent until all points pass IK and safety checks. Failure clears the task and
keeps the last commanded pose.

Execution sends at most one waypoint on each eligible control tick. Spacing is
5 mm and the wait is 1000 ms after **every** waypoint, including the last. There
is no angle ramp and no catch-up burst after a late loop. Seven waypoints require
about seven seconds from the first write to completion, plus validation and tick
scheduling overhead. Only startup retains a blocking 500 ms delay.

`HOLD` freezes the current phase, indices and remaining wait. `GO` resumes them
without counting paused time. Both commands are idempotent while a task is active.
`CANCEL` removes the task and synchronizes all goals to the commanded pose; an
idle `GO` cannot resurrect the canceled target. Motor movement during a pause is
not measured: these hobby servos supply no actual position feedback.

## Code organization and hardware

| Module | Responsibility |
| --- | --- |
| `src/main.cpp` | Startup, bounded input polling and 20 ms scheduling |
| `src/arm.cpp` / `include/arm.h` | Four-joint table, full pose checks, queued/immediate output |
| `src/kinematics.cpp` / `include/kinematics.h` | Original FK/IK model, no serial output |
| `src/motion.cpp` / `include/motion.h` | Preview/execution task and pause/resume/cancel |
| `src/commands.cpp` / `include/commands.h` | Parse a request, dispatch it, print diagnostics |
| `include/config.h` | Serial, scheduling, waypoint and speed settings |

| Joint | Joystick | Center | Servo pin | Startup | Limits | Direction | Output |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Base | A0 | 506 | 9 | 90° | 0..180° | +1 | Disabled |
| Shoulder / J1 | A1 | 519 | 8 | 120° | 20..125° | -1 | Enabled |
| Elbow / J2 | A3 | 519 | 7 | 95° | 40..130° | +1 | Enabled |
| Gripper | A2 | 515 | 6 | 50° | 0..180° | +1 | Disabled |

Shoulder + elbow must be at least 148°. Kinematics constants remain lengths
80/80 mm, right zero 184° and elbow offset 116.9°. All four pose angles are checked
for finiteness and limits before submission. Changes to a disabled joint are
rejected, so software cannot claim a motion that was never sent to that servo.

`commandedAngle` is the software angle submitted to the output, `goalAngle` is a
queued target. Servo writes still truncate to integer degrees as before. Neither
angle is measured feedback. `directionSign` is reserved for interpreting joystick
input, not for reversing already calibrated servo output angles.

## Future extension points

- **Manual control and buttons:** add input polling at the marked location in
  `loop()`. New movement sources must respect task ownership, build a complete
  `Pose`, and call `requestPose()`; do not write directly to `Servo`.
- **Base/gripper:** implement their real control behavior before enabling their
  `outputEnabled` flags. Calibrate open/close positions rather than inventing them.
- **Pick/place and drawing:** add actual task kinds and step state to the motion
  module. Advance one bounded step per loop and preserve progress on pause. Reuse
  `applyPoseImmediate()` for approved trajectory outputs.
- **Recording:** sample after motion updates at the marked location in `loop()`.
  Decide sampling rate, capacity and persistence before allocating a bounded
  buffer. Do not add unused recording arrays now.
- **Five-point teaching:** store points at runtime when the feature is added;
  use the shared task ownership and cancellation behavior for drawing.
- **Coordinates:** introduce base rotation, paper coordinates and pencil offsets
  only after their geometry is calibrated; do not silently reinterpret `XY`.

These are integration notes, not executable placeholder functions or supported
commands. The program uses fixed storage and adds no runtime libraries beyond
the existing Arduino framework and Servo library.
