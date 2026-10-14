#include "Arduino.h"
#include "arm.h"
#include "joystick.h"
#include "commands.h"
#include "config.h"
#include "kinematics.h"
#include "motion.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>

#ifdef LEGACY_REFERENCE
#include "legacy_reference.h"
#endif

uint32_t simulatedTime = 0;
std::vector<ServoWrite> servoWrites;
std::vector<int> attachedPins;
std::vector<uint32_t> delayCalls;
SerialStub Serial;

void setup();
void loop();

namespace {
int assertions = 0;
JointControl initialJoints[JointCount];

void check(bool condition, const char* expression, int line)
{
    ++assertions;
    if (!condition) {
        std::cerr << "FAIL line " << line << ": " << expression << '\n';
        std::exit(1);
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

bool contains(const char* text)
{
    return Serial.output.find(text) != std::string::npos;
}

void advance(uint32_t duration)
{
    simulatedTime += duration;
    loop();
}

void runFor(uint32_t duration)
{
    while (duration >= CONTROL_INTERVAL_MS) {
        advance(CONTROL_INTERVAL_MS);
        duration -= CONTROL_INTERVAL_MS;
    }
    if (duration > 0) {
        advance(duration);
    }
}

void send(const std::string& command)
{
    CHECK(Serial.input.empty());
    Serial.input = command + '\n';
    loop();
    CHECK(Serial.input.empty());
}

void fresh()
{
    // Finish any incomplete serial line before resetting the test clock.
    Serial.input = "\n";
    pollSerial();
    cancelMotion();
    for (uint8_t i = 0; i < JointCount; ++i) {
        joints[i] = initialJoints[i];
    }
    simulatedTime = 0;
    Serial.input.clear();
    Serial.output.clear();
    Serial.bytesRead = 0;
    servoWrites.clear();
    attachedPins.clear();
    delayCalls.clear();
    setup();
    CHECK(simulatedTime == STARTUP_DELAY_MS);
    CHECK(Serial.baud == 115200);
    CHECK(attachedPins.size() == 2 && attachedPins[0] == 8 && attachedPins[1] == 7);
    CHECK(servoWrites.size() == 2 && servoWrites[0].angle == 120 && servoWrites[1].angle == 95);
    advance(20);
    CHECK(!hasPendingPose() && isArmHeld());
    CHECK(delayCalls.size() == 1 && delayCalls[0] == 500);
    servoWrites.clear();
    Serial.output.clear();
}

void prepareA()
{
    send("XY 42.7 51");
    CHECK(hasPendingPose());
    advance(20);
    CHECK(!hasPendingPose());
    FKResult fk = solveFK(joints[Shoulder].commandedAngle, joints[Elbow].commandedAngle);
    CHECK(std::fabs(fk.position.x - 42.7f) < 0.001f);
    CHECK(std::fabs(fk.position.z - 51.0f) < 0.001f);
    servoWrites.clear();
    Serial.output.clear();
}

void validateRun()
{
    send("RUNLINE 71.6 42.5");
    CHECK(motionStatus().state == MotionState::Validating);
    CHECK(motionStatus().validatedWaypoints == 1);
    CHECK(motionStatus().totalWaypoints == 7);
    while (motionStatus().state == MotionState::Validating) {
        uint16_t validated = motionStatus().validatedWaypoints;
        loop();
        CHECK(motionStatus().validatedWaypoints == validated + 1);
        CHECK(servoWrites.empty());
    }
    CHECK(motionStatus().state == MotionState::Running);
    CHECK(motionStatus().completedWaypoints == 0);
    CHECK(contains("PATH VALID"));
}

void testSafetyAndQueuedPoses()
{
    fresh();
    CHECK(joints[Base].servoPin == 9 && joints[Shoulder].servoPin == 8);
    CHECK(joints[Elbow].servoPin == 7 && joints[Gripper].servoPin == 6);
    CHECK(joysticksCentered(readJoysticks())); // Actual ADC mapping is tested in sprint_tests.

    CHECK(!joints[Base].outputEnabled && !joints[Gripper].outputEnabled);
    CHECK(isSafePlanarPose(20, 128));
    CHECK(!isSafePlanarPose(20, 127.9f));
    CHECK(!isSafePlanarPose(19.9f, 130));
    CHECK(!isSafePlanarPose(125.1f, 95));
    CHECK(!isSafePlanarPose(120, 39.9f));
    CHECK(!isSafePlanarPose(120, 130.1f));

    Pose pose = commandedPose();
    pose.angles[Base] = std::numeric_limits<float>::quiet_NaN();
    CHECK(!isSafePose(pose) && !requestPose(pose));
    pose = commandedPose();
    pose.angles[Gripper] = std::numeric_limits<float>::infinity();
    CHECK(!isSafePose(pose));
    pose = commandedPose();
    pose.angles[Base] = 91;
    CHECK(!requestPose(pose));
    CHECK(servoWrites.empty());

    send("J1 119.9");
    CHECK(joints[Shoulder].commandedAngle == 120 && hasPendingPose());
    advance(19);
    CHECK(servoWrites.empty());
    advance(1);
    CHECK(servoWrites.size() == 2 && servoWrites[0].angle == 119);
    CHECK(joints[Shoulder].commandedAngle == 119.9f);
    send("J1 120");
    send("HOLD");
    advance(20);
    CHECK(joints[Shoulder].goalAngle == 119.9f);
    CHECK(servoWrites.size() == 2);
    send("GO");
    advance(20);
    CHECK(servoWrites.size() == 4 && servoWrites[2].angle == 119);

    send("J1 120");
    Serial.output.clear();
    send("RUNLINE 71.6 42.5");
    CHECK(contains("ERR busy") && !isMotionBusy());
    advance(20);
    CHECK(joints[Shoulder].commandedAngle == 120);

    send("J1 110");
    send("CANCEL");
    advance(20);
    CHECK(joints[Shoulder].commandedAngle == 120 && !hasPendingPose());
}

void testValidationAndPreview()
{
    fresh();
    prepareA();
    send("LINE 71.6 42.5");
    CHECK(motionStatus().validatedWaypoints == 1 && servoWrites.empty());
    send("HOLD");
    uint16_t saved = motionStatus().validatedWaypoints;
    runFor(2000);
    CHECK(motionStatus().state == MotionState::Paused);
    CHECK(motionStatus().validatedWaypoints == saved && servoWrites.empty());
    send("GO");
    CHECK(motionStatus().validatedWaypoints == saved + 1);
    while (isMotionBusy()) {
        loop();
    }
    CHECK(contains("LINE VALID") && servoWrites.empty());

    Serial.output.clear();
    send("RUNLINE 0 0");
    while (isMotionBusy()) {
        loop();
    }
    CHECK(contains("RUNLINE ABORTED before motion") && servoWrites.empty());
    CHECK(!hasPendingPose());

    Serial.output.clear();
    send("RUNLINE 999999999999999999999999999999999999 0");
    CHECK(!isMotionBusy() && servoWrites.empty());
    CHECK(contains("ERR line:"));

    send("RUNLINE 71.6 42.5");
    CHECK(isMotionBusy());
    send("CANCEL");
    CHECK(!isMotionBusy() && servoWrites.empty());
    runFor(3000);
    CHECK(servoWrites.empty());
}

void testWaypointTimingAndBusyPolicy()
{
    fresh();
    prepareA();
    validateRun();
    advance(20);
    CHECK(servoWrites.size() == 2);
    uint32_t firstTime = servoWrites[0].time;

    const char* busyCommands[] = {"J1 120", "J2 95", "XY 42.7 51", "LINE 42.7 51", "RUNLINE 42.7 51"};
    for (const char* command : busyCommands) {
        Serial.output.clear();
        send(command);
        CHECK(contains("ERR busy"));
        CHECK(motionStatus().completedWaypoints == 1 && servoWrites.size() == 2);
    }
    Serial.output.clear();
    send("STATUS");
    CHECK(contains("task=RUNLINE phase=RUNNING") && contains("move=1/7"));
    send("IK 42.7 51");
    send("HELP");
    send("SPEED 90");
    CHECK(servoWrites.size() == 2 && motionStatus().completedWaypoints == 1);

    for (uint16_t index = 2; index <= 7; ++index) {
        runFor(999);
        CHECK(servoWrites.size() == static_cast<size_t>((index - 1) * 2));
        advance(1);
        CHECK(servoWrites.size() == static_cast<size_t>(index * 2));
        CHECK(servoWrites[(index - 1) * 2].time == firstTime + (index - 1) * 1000);
    }
    CHECK(isMotionBusy() && !contains("RUNLINE DONE"));
    runFor(999);
    CHECK(isMotionBusy());
    advance(1);
    CHECK(!isMotionBusy() && contains("RUNLINE DONE"));
    CHECK(simulatedTime - firstTime == 7000);
    CHECK(servoWrites.size() == 14 && delayCalls.size() == 1);
    FKResult fk = solveFK(joints[Shoulder].commandedAngle, joints[Elbow].commandedAngle);
    CHECK(std::fabs(fk.position.x - 71.6f) < 0.001f && std::fabs(fk.position.z - 42.5f) < 0.001f);
}

void testPauseResumeAndCancel()
{
    fresh();
    prepareA();
    validateRun();
    send("HOLD");
    runFor(1500);
    CHECK(motionStatus().state == MotionState::Paused && servoWrites.empty());
    send("GO");
    advance(20);
    CHECK(servoWrites.size() == 2);
    runFor(300);
    send("HOLD");
    send("HOLD"); // Repeated pause must not consume the saved remaining wait.
    runFor(5000);
    send("STATUS");
    CHECK(contains("phase=PAUSED") && servoWrites.size() == 2);
    Serial.output.clear();
    send("J1 120");
    CHECK(contains("ERR busy"));
    send("GO");
    send("GO"); // Repeated resume must not restart the countdown.
    runFor(699);
    CHECK(servoWrites.size() == 2);
    advance(1);
    CHECK(servoWrites.size() == 4 && motionStatus().completedWaypoints == 2);

    for (int i = 3; i <= 7; ++i) {
        runFor(1000);
    }
    CHECK(servoWrites.size() == 14);
    runFor(300);
    send("HOLD");
    runFor(2000);
    CHECK(isMotionBusy() && !contains("RUNLINE DONE"));
    send("GO");
    runFor(700);
    CHECK(!isMotionBusy() && contains("RUNLINE DONE"));

    fresh();
    prepareA();
    validateRun();
    advance(20);
    Pose stopped = commandedPose();
    send("HOLD");
    send("CANCEL");
    runFor(3000);
    CHECK(!isMotionBusy() && servoWrites.size() == 2 && !hasPendingPose());
    CHECK(joints[Shoulder].goalAngle == stopped.angles[Shoulder]);
    send("GO");
    advance(20);
    CHECK(joints[Shoulder].commandedAngle == stopped.angles[Shoulder]);
    CHECK(servoWrites.size() == 4 && servoWrites[2].angle == servoWrites[0].angle);
    send("J1 120");
    advance(20);
    CHECK(joints[Shoulder].commandedAngle == 120);
}

void testClockRolloverAndNoCatchUpBurst()
{
    fresh();
    prepareA();
    validateRun();
    simulatedTime = UINT32_MAX - 400;
    loop();
    CHECK(servoWrites.size() == 2);
    runFor(300);
    send("HOLD");
    runFor(1000); // Cross UINT32_MAX while paused.
    send("GO");
    runFor(699);
    CHECK(servoWrites.size() == 2);
    advance(1);
    CHECK(servoWrites.size() == 4);

    advance(10000);
    CHECK(servoWrites.size() == 6); // One waypoint, never a burst of missed points.
    loop();
    CHECK(servoWrites.size() == 6);
    runFor(999);
    CHECK(servoWrites.size() == 6);
    advance(1);
    CHECK(servoWrites.size() == 8);
}

void testSerialParsingAndFairness()
{
    fresh();
    const char* invalid[] = {
        "IK", "IK 42.7", "IK nan 51", "IK -inf 51", "IK 1e2 51", "IK 42.7 51 extra",
        "XY 42.7 51 extra", "J1 120 extra", "J2 .", "J1 +", "SPEED 60 extra",
        "SPEED 999999999999999999999999999999999999999999999",
        "LINE 71.6 42.5 extra", "RUNLINE 71.6 42.5 extra", "HOLD extra", "GO extra",
        "CANCEL extra", "STATUS extra", "HELP extra", "UNKNOWN", "O", "S", "H", "L"
    };
    for (const char* command : invalid) {
        Serial.output.clear();
        send(command);
        CHECK(contains("ERR") && servoWrites.empty() && !hasPendingPose() && !isMotionBusy());
    }
    for (const char* command : {"SPEED 4.9", "SPEED 90.1"}) {
        Serial.output.clear();
        send(command);
        CHECK(contains("ERR speed range"));
    }
    for (const char* command : {"SPEED 5", "SPEED 90", "SPEED 60"}) {
        Serial.output.clear();
        send(command);
        CHECK(contains("OK SPEED"));
    }
    for (const char* command : {"IK +42.7 51.", "42.7 51", "  IK\t42.7\t51  \r"}) {
        Serial.output.clear();
        send(command);
        CHECK(contains("IK J1=") && servoWrites.empty());
    }
    Serial.output.clear();
    send("IK -10 +20.5");
    CHECK(contains("ERR IK:") && !contains("ERR usage:"));

    Serial.output.clear();
    std::string maximumLine = "IK 42.7 51";
    maximumLine.append(63 - maximumLine.size(), ' ');
    send(maximumLine);
    CHECK(contains("IK J1=") && !contains("ERR command too long"));

    Serial.output.clear();
    Serial.input = "IK 42.";
    loop();
    CHECK(Serial.output.empty());
    Serial.input = "7 51\r\n";
    loop();
    CHECK(contains("IK J1="));

    Serial.output.clear();
    Serial.input = std::string(64, 'A') + "J1 100\nSTATUS\n";
    loop();
    CHECK(Serial.bytesRead > 0 && contains("ERR command too long"));
    CHECK(!Serial.input.empty());
    loop(); // Discard the remainder through its newline, including J1 100.
    CHECK(!hasPendingPose() && servoWrites.empty());
    loop();
    CHECK(contains("task=NONE") && !contains("OK 100"));

    Serial.output.clear();
    Serial.input = std::string(128, 'A');
    size_t reads = Serial.bytesRead;
    loop();
    CHECK(Serial.bytesRead - reads == 64 && Serial.input.size() == 64);
    Serial.input += '\n';
    loop();
    loop();
    CHECK(Serial.input.empty());

    Serial.output.clear();
    Serial.input = "STATUS\nSTATUS\n";
    loop();
    CHECK(Serial.input == "STATUS\n");
    loop();
    CHECK(Serial.input.empty());

    prepareA();
    validateRun();
    advance(20);
    // Continuous input does not prevent timed waypoint progression.
    for (int i = 0; i < 50; ++i) {
        Serial.input += "STATUS\n";
        advance(20);
    }
    CHECK(servoWrites.size() == 4 && motionStatus().completedWaypoints == 2);
    CHECK(delayCalls.size() == 1);
}

void testExecutionFailureStopsTask()
{
    fresh();
    prepareA();
    validateRun();
    joints[Elbow].maxAngle = 40; // Simulate changed safety data after validation.
    advance(20);
    CHECK(!isMotionBusy() && servoWrites.empty() && !hasPendingPose());
    CHECK(contains("ERR unexpected IK failure"));

    fresh();
    prepareA();
    validateRun();
    joints[Base].minAngle = 91; // IK is still valid, but the complete pose is unsafe.
    advance(20);
    CHECK(!isMotionBusy() && servoWrites.empty() && !hasPendingPose());
    CHECK(contains("ERR execution safety failure"));
}

#ifdef LEGACY_REFERENCE
void testLegacyMathAndOutputEquivalence()
{
    fresh();
    for (int shoulder = 20; shoulder <= 125; shoulder += 3) {
        for (int elbow = 40; elbow <= 130; elbow += 3) {
            auto before = legacy::solveFK(shoulder, elbow);
            auto after = solveFK(shoulder, elbow);
            CHECK(before.alpha == after.alpha && before.beta == after.beta && before.gamma == after.gamma);
            CHECK(before.position.x == after.position.x && before.position.z == after.position.z);
            auto oldIK = legacy::solveIK(before.position.x, before.position.z);
            auto newIK = solveIK(before.position.x, before.position.z);
            CHECK(oldIK.valid == newIK.valid && oldIK.j1 == newIK.j1 && oldIK.j2 == newIK.j2);
            CHECK(oldIK.alpha == newIK.alpha && oldIK.beta == newIK.beta && oldIK.gamma == newIK.gamma);
            CHECK(legacy::isSafePose(shoulder, elbow) == isSafePlanarPose(shoulder, elbow));
        }
    }

    const char* commands[] = {"J1 120", "J2 95", "XY 42.7 51", "42.7 51", "J1 19.9", "J2 130.1", "SPEED 60"};
    for (const char* command : commands) {
        legacy::j1.targetAngle = legacy::j1.requestedAngle = 120;
        legacy::j2.targetAngle = legacy::j2.requestedAngle = 95;
        char line[64];
        std::strcpy(line, command);
        Serial.output.clear();
        legacy::handleCommand(line);
        std::string expected = Serial.output;
        fresh();
        send(command);
        CHECK(Serial.output == expected);
    }
}
#endif
}

int main()
{
    for (uint8_t i = 0; i < JointCount; ++i) {
        initialJoints[i] = joints[i];
    }
    testSafetyAndQueuedPoses();
    testValidationAndPreview();
    testWaypointTimingAndBusyPolicy();
    testPauseResumeAndCancel();
    testClockRolloverAndNoCatchUpBurst();
    testSerialParsingAndFairness();
    testExecutionFailureStopsTask();
#ifdef LEGACY_REFERENCE
    testLegacyMathAndOutputEquivalence();
#endif
    std::cout << "PASS: " << assertions << " checks\n";
}
