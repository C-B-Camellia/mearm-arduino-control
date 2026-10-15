> Current Bringup (2026-10-15): runtime `FEED` is retired. LINE/SHAPE remain 35 mm/s; five-point POLY/CURVE strokes use 70 mm/s. Use `uno_nano_five_fast`. [Fast35/70 evidence, HEX hash and bringup steps](docs/Fast70_2026-10-15.md).

# MeArm assessment firmware candidate

PlatformIO / classic Arduino Uno / ATmega328P / Servo. This candidate implements
manual JOINT/CART, calibrated 3D FK/IK, serial moves, bounded recording/playback,
HOME, A/B/C programs, shared button events, five-point teaching and drawing.
It preserves the original 2D FK/IK and x/z LINE/RUNLINE commands.

**Production J0/J3, 3D, grip, HOME, A/B/C and external UART configuration are
not calibrated yet. Their corresponding actions refuse motion.** J1/J2 retain
their existing verified configuration. No upload or physical validation was done
by the agent. Software command angles are not servo position feedback.

## Build and automated checks

```powershell
pio run -e uno
& test/native/run.ps1 -Compiler 'C:/msys64/ucrt64/bin/g++.exe'
python test/capacity_check.py
```

Use another C++11 g++ path if needed. `run.ps1` defaults to `sprint_tests.cpp`;
the old `test_main.cpp` is a historical test fixture with obsolete timing/state
assertions. Optional `-LegacySource <saved-original-main.cpp>` compares 2D math
against the original implementation, not intentionally changed command behavior.

The capacity check builds a separate `.pio/synthetic-capacity-check` project
with fake test measurements and an enabled software UART. **Never upload its
firmware.** It checks capacity only; the default production configuration stays
unchanged. Build artifacts and test executables remain under ignored `.pio/`.

## Timing and speed

- Main control clock: 20 ms, phase preserved, one new Pose per tick; no late-loop
  burst. Manual integration caps elapsed time at 40 ms and resets after takeover.
- RUNLINE minimum segment time: 10 ms. This is not the PWM refresh period.
- Servo uses Timer1 to generate the pulse train independently of main; its
  library refresh period remains 20 ms. Timer0/millis is unchanged.
- JOINT joystick bands: 40/80 deg/s. H/L changes only manual multiplier,
  default 1, bounded 0.75..1.5 in 0.25 increments.
- CART bands: 10/20 mm/s, same manual multiplier; independent angular units.
- SPEED changes automatic angular limit, default 60, accepted 5..90 deg/s.
  Changes during tasks are rejected. Drawing's nominal Cartesian feed is
  `10 * SPEED / 60` mm/s with angular constraints. Replay preserves recorded
  time gaps unless a slower angular limit requires stretching.
- Actual speed, low-speed heating, settling, paper accuracy and PWM jitter still
  require physical tuning. Integer Servo commands limit spatial resolution.

## Serial commands

115200 baud, newline terminated; CRLF and spaces/tabs between tokens accepted.
Commands use the case shown. Decimal numbers only, no nan/inf/exponents. One line
is at most 63 characters. Overflow or invalid control bytes discard the entire
line through newline. At most 64 bytes and one command are processed per loop.
Logging uses Arduino synchronous printing; STATUS/HELP can add loop latency.

| Command | Meaning |
|---|---|
| `IK x z` or `x z` | Original 2D diagnostic, no motion |
| `XY x z` | Rate-limited shoulder/elbow goal; historical XY name still means x/z |
| `LINE x z` | Incremental whole-path preview, no servo motion |
| `RUNLINE x z` | Validate, then execute original planar path with 4 mm samples |
| `J0/J1/J2/J3 angle` | One joint target, other joints preserved; disabled axes reject |
| `x90,y120,z95` | Atomic angle triple: x=J0, y=J1, z=J2; exactly three unique fields |
| `XYZ x y z` / `IK3 x y z` | World tool position in mm / diagnostic only |
| `MODE JOINT` / `MODE CART` | Switch only when neutral, idle and not held; CART needs calibration |
| `O` / `S` | Configured gripper open / closed goal |
| `H` / `L` | Increase / decrease only joystick multiplier |
| `SPEED value` | Configure automatic task speed only |
| `HOLD` / `GO` | Pause / resume; preserves phase/progress, no paused-time catch-up |
| `CANCEL` | Stop and hold; drawing instead lifts and requests calibrated HOME |
| `A` / `B` / `C` | Eight measured pick/place stages with configured dwell |
| `REC`, `REC START`, `REC STOP` | Toggle / start / finalize bounded RAM recording |
| `PLAY` | Safe approach to first frame, then replay all recorded frames |
| `HOME` | Stop current task, prevalidate and execute configured HOME/via route |
| `BTN 1` .. `BTN 4` | Same application entry as physical board buttons |
| `TEACH START` | Enter CART teaching setup, retain points until explicit CLEAR |
| `TEACH DRAWZ`, `TEACH TRAVELZ` | Capture current modeled tool height |
| `TEACH MARKING` | Lock Z to travelZ; enter at captured travel pose with centered sticks |
| `MARK`, `LIST`, `CLEAR` | Append / list ordered XY points / clear points and return setup |
| `ORDER 1 3 2 5 4` | Permute all five recorded points, no rebuild |
| `TEACH EXIT` | Exit teaching and synchronize manual target |
| `DRAW LINE x1 y1 x2 y2` | Paper line using configured heights and HOME |
| `DRAW SHAPE N x0 y0 x1 y1` | N in runtime bounding rectangle; also TRIANGLE/Z/V |
| `DRAW SHAPE N` | Bounds from first two ordered taught points |
| `DRAW POLY`, `DRAW CURVE` | Four segments through five runtime points |
| `STATUS`, `HELP` | State and protocol |

New target tasks reject while any task is validating, running or paused. All
complete poses are checked before submission and again after integer truncation.
No modules besides arm own Servo. Normal task completion clears ownership;
explicit HOLD is separate. GO never resurrects a canceled goal. Drawing requires
the HOME/via gripper angles to keep the pencil clamped.

Recording: 80 frames, 6 bytes each, 150 ms minimum sampling interval plus actual
relative timestamps; the 20 ms tick normally samples about every 160 ms. Initial
and stopped frames are saved. Buffer full explicitly stops without overwriting.
A valid record is >10 s and varies one recorded axis by >=5 deg; this software
threshold is not the official assessment's "obvious motion" criterion. Starting
a new recording replaces the previous RAM recording. No EEPROM persistence.

## Physical and simulated four keys

| Context | B1 | B2 | B3 | B4 |
|---|---|---|---|---|
| Normal | A/B/C accepted-task cycle | Record toggle | PLAY | HOME |
| Teach setup | Capture drawZ | Capture travelZ | Begin marking | Exit teach |
| Marking | MARK | POLY | CURVE | Clear and return setup |
| Drawing, including paused/validation | HOLD | GO | Cancel, lift, HOME | No action |

During recording other buttons reject. During non-drawing tasks B1/B2/B3 reject,
B4 requests cancel/HOME. External board sends one press-edge frame `B1\n` ..
`B4\n`; identical frames within 30 ms are filtered. The board must avoid repeat
events during a held button. PC BTN calls the same dispatcher directly.

Only define MEARM_BUTTON_RX/TX/BAUD after confirming real wiring, common ground,
logic levels and protocol. All three definitions are required. Compile-time pin
checks exclude D0/D1 USB UART, D6-D9 Servo outputs and A0-A3 joystick inputs.
The candidate has no guessed board pin assignment. SoftwareSerial and Servo
concurrency remains unverified on hardware. D2/D3/9600 in the capacity test are
test-only values, not an approved wiring plan.

## Calibration and hardware

| Joint | ADC | Center | Servo | Existing startup | Existing range | Default output |
|---|---|---:|---|---:|---|---|
| J0 Base | A0 | 506 | D9 | 90 | Nominal only | Disabled |
| J1 Shoulder | A1 | 519 | D8 | 120 | 20..125 | Enabled |
| J2 Elbow | A3 | 519 | D7 | 95 | 40..130 | Enabled |
| J3 Gripper | A2 | 515 | D6 | 50 | Nominal only | Disabled |

J1+J2 must remain >=148 degrees, including integer output. Current joystick
deadzone is 20 ADC counts, fast thresholds <=8 or >=1015. JOINT mapping:
LX/LY/RY/RX = J0/J1/J2/J3. CART: LX/LY/RY = X/Y/Z; RX = gripper. Marking masks
vertical and gripper input. The source's LY sign is reversed; confirm real axes.

Enter real measurements in `src/hardware_config.cpp`: base/gripper valid flags,
limits/startup; yaw world-zero/sign; shoulder offsets; tool model and offsets;
gripper open/closed; HOME and up to two validated via poses. Do not substitute
synthetic tests or old nominal values. `src/pick_place.cpp` contains flash-resident
measured A/B/C programs, each eight poses and dwell times; leave valid false until
actual pickup/placement routes have been approved. Rebuild and rerun checks after
changing configuration. These static parameters are not hot-updated by serial.

3D uses positive radial solutions with calibrated yaw; supports fixed offsets in
the rotating radial plane or a rigid tool attached to the distal-link frame. It
does not model arbitrary unknown gripper/pencil mechanisms or obstacle geometry.
NaN FK / invalid IK means calibration or reachability failed, not zero position.

STATUS numeric fields: phase 0 idle, 1 validating, 2 running, 3 paused; workflow
0 normal, 1 setup, 2 marking; rec 0 empty, 1 recording, 2 ready. Task kinds are
0 none, 1 LINE, 2 RUNLINE, 3 joint, 4 Cartesian, 5 HOME, 6 pick, 7 replay,
8 draw line, 9 shape, 10 polyline, 11 curve. Idle can retain the last task kind
and completed progress. `enabled` is the four-axis bitmask (default 6); `error`
is the most recent rejection code from control_error.h. Disabled J values are
nominal software state, not proof of output or actual position.

Tonight's measured commissioning sequence, commands and evidence checklist are
in `docs/Assessment_Firmware_Acceptance_2026-10-12.md`. The editable Word derivation
is `docs/MeArm_Kinematics_Derivation_2026-10-12.docx`.
