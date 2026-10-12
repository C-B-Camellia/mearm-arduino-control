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
New commands: J0/J3, XYZ/IK3, O/S, H/L, atomic x/y/z angle triple.
Safe software queries: STATUS; IK 42.7 51; MODE JOINT; H; L; SPEED 60.
Uncalibrated XYZ/IK3/O/S and disabled J0/J3 must reject without servo output.
