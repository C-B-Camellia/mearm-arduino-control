#include "commands.h"
#include "arm.h"
#include "config.h"
#include "kinematics.h"
#include "motion.h"
#include "manual_control.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {
enum class CommandType : uint8_t {
    PrintIK, MovePlanar, MoveJoint, SetSpeed,
    Hold, Go, Cancel, Status, Help, PreviewLine, RunLine, SetMode
};

struct Command {
    CommandType type;
    float first;
    float second;
    JointId joint;
    ControlMode mode;
};

char serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialIndex = 0;
bool discardLine = false;
float configuredMoveSpeed = DEFAULT_MOVE_SPEED;

// Ordinary decimal syntax; scientific notation, nan and inf are not accepted.
bool parseNumber(const char* text, float& value)
{
    if (text == nullptr || *text == '\0') {
        return false;
    }
    const char* digits = text;
    if (*digits == '+' || *digits == '-') {
        ++digits;
    }
    bool hasDigit = false;
    bool hasDot = false;
    while (*digits != '\0') {
        if (*digits >= '0' && *digits <= '9') {
            hasDigit = true;
        }
        else if (*digits == '.' && !hasDot) {
            hasDot = true;
        }
        else {
            return false;
        }
        ++digits;
    }
    if (!hasDigit) {
        return false;
    }
    value = atof(text);
    return isfinite(value);
}

bool parseCommand(char* line, Command& command)
{
    // Four tokens detect extra arguments for every current command.
    char* tokens[4];
    uint8_t count = 0;
    char* token = strtok(line, " \t");
    while (token != nullptr && count < 4) {
        tokens[count++] = token;
        token = strtok(nullptr, " \t");
    }
    if (count == 0) {
        return false;
    }

    // Retain the old two-number, diagnostic-only shortcut.
    if (count == 2 && parseNumber(tokens[0], command.first) &&
        parseNumber(tokens[1], command.second)) {
        command.type = CommandType::PrintIK;
        return true;
    }

    const __FlashStringHelper* usage = nullptr;
    uint8_t expectedCount = 1;
    if (strcmp(tokens[0], "IK") == 0) {
        command.type = CommandType::PrintIK;
        usage = F("ERR usage: IK <x> <z>");
        expectedCount = 3;
    }
    else if (strcmp(tokens[0], "XY") == 0) {
        command.type = CommandType::MovePlanar;
        usage = F("ERR usage: XY <x> <z>");
        expectedCount = 3;
    }
    else if (strcmp(tokens[0], "J1") == 0 || strcmp(tokens[0], "J2") == 0) {
        command.type = CommandType::MoveJoint;
        command.joint = strcmp(tokens[0], "J1") == 0 ? Shoulder : Elbow;
        usage = command.joint == Shoulder ? F("ERR usage: J1 <angle>") : F("ERR usage: J2 <angle>");
        expectedCount = 2;
    }
    else if (strcmp(tokens[0], "SPEED") == 0) {
        command.type = CommandType::SetSpeed;
        usage = F("ERR usage: SPEED <deg/s>");
        expectedCount = 2;
    }
    else if (strcmp(tokens[0], "LINE") == 0 || strcmp(tokens[0], "RUNLINE") == 0) {
        command.type = strcmp(tokens[0], "LINE") == 0 ? CommandType::PreviewLine : CommandType::RunLine;
        usage = command.type == CommandType::PreviewLine ? F("ERR usage: LINE <x> <z>") : F("ERR usage: RUNLINE <x> <z>");
        expectedCount = 3;
    }
    else if (strcmp(tokens[0], "HOLD") == 0) {
        command.type = CommandType::Hold;
        usage = F("ERR usage: HOLD");
    }
    else if (strcmp(tokens[0], "GO") == 0) {
        command.type = CommandType::Go;
        usage = F("ERR usage: GO");
    }
    else if (strcmp(tokens[0], "CANCEL") == 0) {
        command.type = CommandType::Cancel;
        usage = F("ERR usage: CANCEL");
    }
    else if (strcmp(tokens[0], "STATUS") == 0) {
        command.type = CommandType::Status;
        usage = F("ERR usage: STATUS");
    }
    else if (strcmp(tokens[0], "HELP") == 0) {
        command.type = CommandType::Help;
        usage = F("ERR usage: HELP");
    }
    else if (strcmp(tokens[0], "MODE") == 0) {
        command.type = CommandType::SetMode;
        if (count != 2) {
            Serial.println(F("ERR invalid mode"));
            return false;
        }
        if (strcmp(tokens[1], "JOINT") == 0) {
            command.mode = ControlMode::Joint;
            return true;
        }
        if (strcmp(tokens[1], "CART") == 0){
            command.mode = ControlMode::Cartesian;
            return true;
        }
        Serial.println(F("ERR invalid mode"));
        return false;
    }
    else {
        Serial.print(F("ERR unknown command: "));
        Serial.println(tokens[0]);
        return false;
    }

    bool valid = count == expectedCount;
    if (valid && expectedCount >= 2) {
        valid = parseNumber(tokens[1], command.first);
    }
    if (valid && expectedCount == 3) {
        valid = parseNumber(tokens[2], command.second);
    }
    if (!valid) {
        Serial.println(usage);
    }
    return valid;
}

void printIKResult(const IKResult& result)
{
    Serial.print(F("IK J1="));
    Serial.print(result.j1, 1);
    Serial.print(F(" J2="));
    Serial.print(result.j2, 1);
    Serial.print(F(" | alpha="));
    Serial.print(result.alpha, 1);
    Serial.print(F(" gamma="));
    Serial.print(result.gamma, 1);
    Serial.print(F(" beta="));
    Serial.println(result.beta, 1);
}

void submitPose(const Pose& pose)
{
    if (!requestPose(pose)) {
        Serial.print(F("ERR unsafe pose: J1="));
        Serial.print(pose.angles[Shoulder], 1);
        Serial.print(F(" J2="));
        Serial.print(pose.angles[Elbow], 1);
        Serial.print(F(" sum="));
        Serial.println(pose.angles[Shoulder] + pose.angles[Elbow], 1);
        return;
    }

    Serial.print(F("OK "));
    Serial.print(pose.angles[Shoulder], 1);
    Serial.print(' ');
    Serial.println(pose.angles[Elbow], 1);

    FKResult fk = solveFK(pose.angles[Shoulder], pose.angles[Elbow]);
    Serial.print(F("alpha="));
    Serial.print(fk.alpha, 1);
    Serial.print(F(" gamma="));
    Serial.print(fk.gamma, 1);
    Serial.print(F(" beta="));
    Serial.print(fk.beta, 1);
    Serial.print(F(" | FK x="));
    Serial.print(fk.position.x, 1);
    Serial.print(F(" z="));
    Serial.println(fk.position.z, 1);
}

void printStatus()
{
    MotionStatus motion = motionStatus();
    Serial.print(F("J1 cur="));
    Serial.print(joints[Shoulder].commandedAngle, 1);
    Serial.print(F(" req="));
    Serial.print(joints[Shoulder].goalAngle, 1);
    Serial.print(F(" | J2 cur="));
    Serial.print(joints[Elbow].commandedAngle, 1);
    Serial.print(F(" req="));
    Serial.print(joints[Elbow].goalAngle, 1);
    Serial.print(F(" | sum="));
    Serial.print(joints[Shoulder].commandedAngle + joints[Elbow].commandedAngle, 1);
    Serial.print(F(" | speed="));
    Serial.print(configuredMoveSpeed, 1);
    Serial.print(F(" (config only) | state="));
    bool held = isMotionBusy() ? motion.state == MotionState::Paused : isArmHeld();
    Serial.print(held ? F("HOLD") : F("RUN"));

    Serial.print(F(" | task="));
    switch (motion.kind) {
        case TaskKind::None: Serial.print(F("NONE")); break;
        case TaskKind::LinePreview: Serial.print(F("LINE")); break;
        case TaskKind::LineExecution: Serial.print(F("RUNLINE")); break;
    }
    Serial.print(F(" phase="));
    switch (motion.state) {
        case MotionState::Idle: Serial.print(F("IDLE")); break;
        case MotionState::Validating: Serial.print(F("VALIDATING")); break;
        case MotionState::Running: Serial.print(F("RUNNING")); break;
        case MotionState::Paused: Serial.print(F("PAUSED")); break;
    }
    Serial.print(F(" validated="));
    Serial.print(motion.validatedWaypoints);
    Serial.print('/');
    Serial.print(motion.totalWaypoints);
    Serial.print(F(" move="));
    Serial.print(motion.completedWaypoints);
    Serial.print('/');
    Serial.println(motion.totalWaypoints);
    Serial.print(F(" | mode="));

    if (getControlMode() == ControlMode::Joint) {
        Serial.print(F("JOINT"));
    } else {
        Serial.print(F("CART"));
    }
}

void printHelp()
{
    Serial.println(F("=== MeArm Calibration Controller ==="));
    Serial.println(F("One command per line; coordinates are planar x/z in mm."));
    Serial.println(F("  IK <x> <z> or <x> <z> (print only)"));
    Serial.println(F("  XY <x> <z> (queue IK pose)"));
    Serial.println(F("  J1 <angle> / J2 <angle> (degrees)"));
    Serial.println(F("  LINE <x> <z> (preview; no motion)"));
    Serial.println(F("  RUNLINE <x> <z> (validate, then execute)"));
    Serial.println(F("  HOLD / GO (pause/resume active task)"));
    Serial.println(F("  CANCEL (stop task and keep last command pose)"));
    Serial.println(F("  SPEED <deg/s> (5..90; configuration only, no motion effect)"));
    Serial.println(F("  STATUS / HELP"));
    Serial.println(F("Active tasks reject new motion/line commands: cancel first."));
    Serial.print(F("J1 range: "));
    Serial.print(joints[Shoulder].minAngle, 1);
    Serial.print(F(" .. "));
    Serial.println(joints[Shoulder].maxAngle, 1);
    Serial.print(F("J2 range: "));
    Serial.print(joints[Elbow].minAngle, 1);
    Serial.print(F(" .. "));
    Serial.println(joints[Elbow].maxAngle, 1);
    Serial.print(F("Constraint: J1 + J2 >= "));
    Serial.println(J1_J2_MIN_SUM, 1);
}

void executeCommand(const Command& command)
{
    if ((command.type == CommandType::MovePlanar || command.type == CommandType::MoveJoint) &&
        isMotionBusy()) {
        Serial.println(F("ERR busy"));
        return;
    }

    switch (command.type) {
        case CommandType::PrintIK:
        case CommandType::MovePlanar: {
            IKResult ik = solveIK(command.first, command.second);
            if (!ik.valid) {
                Serial.println(F("ERR IK: unreachable or unsafe target"));
                return;
            }
            printIKResult(ik);
            if (command.type == CommandType::MovePlanar) {
                Pose pose = goalPose();
                pose.angles[Shoulder] = ik.j1;
                pose.angles[Elbow] = ik.j2;
                submitPose(pose);
            }
            break;
        }
        case CommandType::MoveJoint: {
            Pose pose = goalPose();
            pose.angles[command.joint] = command.first;
            submitPose(pose);
            break;
        }
        case CommandType::SetSpeed:
            if (command.first < MIN_MOVE_SPEED || command.first > MAX_MOVE_SPEED) {
                Serial.print(F("ERR speed range "));
                Serial.print(MIN_MOVE_SPEED, 1);
                Serial.print(F(" .. "));
                Serial.println(MAX_MOVE_SPEED, 1);
                return;
            }
            configuredMoveSpeed = command.first;
            Serial.print(F("OK SPEED "));
            Serial.println(configuredMoveSpeed, 1);
            break;
        case CommandType::Hold:
            if (!pauseMotion(millis())) {
                holdArm();
            }
            Serial.println(F("OK HOLD"));
            break;
        case CommandType::Go:
            if (!resumeMotion(millis())) {
                resumeArm();
            }
            Serial.println(F("OK GO"));
            break;
        case CommandType::Cancel:
            cancelMotion();
            Serial.println(F("OK CANCEL"));
            break;
        case CommandType::Status: printStatus(); break;
        case CommandType::Help: printHelp(); break;
        case CommandType::PreviewLine:
        case CommandType::RunLine:
            startLine({command.first, command.second},
                      command.type == CommandType::PreviewLine ? TaskKind::LinePreview : TaskKind::LineExecution);
            break;
        case CommandType::SetMode: {
            if (setControlMode(command.mode)) {
                if (command.mode == ControlMode::Joint) Serial.println(F("OK MODE JOINT"));
                else Serial.println(F("OK MODE CART"));
            }
            else Serial.println(F("ERR busy"));
            break;
        }
    }
}
}

void pollSerial()
{
    // Bound byte processing and dispatch so motion always gets a turn.
    for (uint8_t consumed = 0; consumed < SERIAL_BYTES_PER_LOOP && Serial.available() > 0; ++consumed) {
        char c = static_cast<char>(Serial.read());
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            if (!discardLine) {
                serialBuffer[serialIndex] = '\0';
                Command command{};
                if (parseCommand(serialBuffer, command)) {
                    executeCommand(command);
                }
            }
            serialIndex = 0;
            discardLine = false;
            return;
        }
        if (discardLine) {
            continue;
        }
        if ((c >= 32 && c <= 126) || c == '\t') {
            if (serialIndex < SERIAL_BUFFER_SIZE - 1) {
                serialBuffer[serialIndex++] = c;
            }
            else {
                discardLine = true;
                Serial.println(F("ERR command too long"));
            }
        }
    }
}
