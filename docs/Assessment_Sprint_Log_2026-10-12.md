# MeArm assessment sprint, 2026-10-12

User snapshot: `53243ec`, branch `assessment-sprint-2026-10-12`.
No hardware upload or real motion has been performed by the agent.

## Batch 0

Fixed full-pose integer output checks, explicit HOLD state, and ADC direction at
fast-band thresholds. Preserved old source tests as a historical fixture;
`sprint_tests.cpp` is the current native suite and does not assume old 5 mm/1 s
trajectory parameters. Exact 2D IK boundaries retain the original strict checks.

Native: 2,893 checks passed. Uno: flash 15,704 B, static SRAM 538 B.
Checkpoints: `133cb38`, `a0bcf20`, `fb9496e`.

## Batch 1

Flow: serial / joystick -> manual or unique motion owner -> queued complete Pose
-> arm integer safety check -> main control tick -> Servo Timer1.

JOINT 40/80 deg/s, manual multiplier 0.75..1.5 in 0.25 steps. CART uses independent
10/20 mm/s. H/L changes only the multiplier; SPEED changes only automatic speed.
MODE changes synchronize from the last commanded integer pose. Motion targets
are prevalidated, interpolated, and emitted once per control tick. Main uses a
phase-preserving 20 ms clock; RUNLINE retains its 10 ms minimum segment duration.
PWM remains Servo's interrupt-driven 20 ms pulse train.

3D supports calibrated yaw, shoulder offsets, tangential tool offset, and either
fixed planar or distal-link tool offsets. Tests use synthetic measurements only.
All production J0/J3/3D/grip/HOME validity flags remain false pending user data.

Build: flash 22,070 B, static SRAM 817 B. Native results recorded at checkpoint.
Native: 4,439 checks passed. Checkpoint: `6df83a1`.
New commands: J0/J3, XYZ/IK3, O/S, H/L, atomic x/y/z angle triple.
Safe software queries: STATUS; IK 42.7 51; MODE JOINT; H; L; SPEED 60.
Uncalibrated XYZ/IK3/O/S and disabled J0/J3 must reject without servo output.

## Batch 2

Recording observes integer command frames only. 96 frames * 6 bytes, 150 ms
minimum sampling interval, actual elapsed timestamps, initial/final frames, and
explicit full-buffer stop. A valid recording lasts >10 s and varies at least one
axis by 5 degrees (software validity threshold, not an official scoring tolerance).
Playback is a motion task: rate-limited approach to first frame, then recorded
timing with safe interpolation. A slower speed limit can lengthen playback.

HOME uses configured via poses; pick/place uses eight measured poses and dwell
times per object, stored in flash. All production programs remain uncalibrated.
PC BTN and external B1..B4 newline frames share application dispatch. A 30 ms
same-button duplicate filter handles short repeated frames. The board must send
press edges; long-press repeat is not synthesized by the arm firmware.

External link disabled until MEARM_BUTTON_RX/TX/BAUD are supplied. Compile-time
pin checks exclude USB UART, current Servo outputs and analog joystick pins.
No claim of Servo/SoftwareSerial concurrency validation.

Native: 5,676 checks passed before final fixture expansion. Uno: flash 25,144 B,
static SRAM 1,442 B. Test inputs: REC START, joystick motion >10s, REC STOP, PLAY,
HOLD, GO, CANCEL; BTN 1..4; A/B/C; HOME. Missing calibration rejects motion.

Final Batch 2 native: 5,677 checks. Checkpoint: `fdc36ca`.

## Batch 3

Five runtime XY points, captured draw/travel heights, order permutations and four
context-specific buttons. Marking locks the continuous CART target to travelZ;
enter marking at the pose used to capture travelZ, with centered sticks.
No point/config editing while a task owns motion. New teaching never silently
overwrites existing points; CLEAR explicitly resets points and returns setup.

Drawing: HOME/via -> lift above HOME -> approach first point -> lower -> stroke
-> lift at last point -> HOME/via. The HOME route must preserve the pencil's grip
angle. Cancellation prevalidates a lift at current XY followed by the HOME route.
Missing/unsafe recovery keeps the last commanded pose and reports failure.

Line, N/Triangle/Z/V, four-segment polyline and centripetal Catmull-Rom use the
same owner. Curves interpolate the five ordered points; mirrored endpoint knots,
near-duplicate rejection, bounded adaptive-density samples and incremental
whole-task checking. Chord-length timing approximates Cartesian feed; integer
angle output limits real paper accuracy and requires physical validation.

Recording capacity reduced to 80 frames (480 B), still >10 seconds at the current
150 ms sampling minimum. AVR command names moved to flash; ordinary decimal
parser avoids strtod; equivalent Hermite tangent factors share divisions.
AVR call-prologue sharing, linker relaxation and constant pooling fit features.

Native: 10,540 checks passed, including all four shapes, runtime teaching, point
order, pause/resume/cancel and modeled polyline pass-through. The 3 mm test
tolerance reflects integer command quantization, not an official paper tolerance.
An isolated synthetic fully-enabled UART/calibration build is also checked;
its firmware MUST NOT be uploaded. Production calibration is unchanged.
