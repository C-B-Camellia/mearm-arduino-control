#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ============================================================
// Hardware mapping
// ============================================================

constexpr uint8_t J1_SERVO_PIN = 8;
constexpr uint8_t J2_SERVO_PIN = 7;

// ============================================================
// Control timing
// ============================================================

constexpr uint32_t CONTROL_INTERVAL_MS = 20;

// ============================================================
// Calibration motion parameters
// ============================================================

constexpr float DEFAULT_MOVE_SPEED = 60.0f;   // deg/s
constexpr float MIN_MOVE_SPEED     = 5.0f;
constexpr float MAX_MOVE_SPEED     = 90.0f;

// ============================================================
// Measured joint limits
// ============================================================

constexpr float J1_MIN_ANGLE = 20.0f;
constexpr float J1_MAX_ANGLE = 125.0f;

constexpr float J2_MIN_ANGLE = 40.0f;
constexpr float J2_MAX_ANGLE = 130.0f;

// 实测机械耦合安全边界
constexpr float J1_J2_MIN_SUM = 148.0f;

// ============================================================
// Safe startup pose
// ============================================================

constexpr float START_J1 = 120.0f;
constexpr float START_J2 = 95.0f;

// Shared kinematics working model (lengths in mm, angles in degrees).
constexpr float L1 = 80.0f;
constexpr float L2 = 80.0f;
constexpr float RIGHT_ZERO = 184.0f;
constexpr float ELBOW_OFFSET = 116.9f;

// ============================================================
// Runtime state
// ============================================================

Servo servoJ1;
Servo servoJ2;

struct CalibrationJoint {
    Servo* servo;

    float currentAngle;
    float requestedAngle;

    float minAngle;
    float maxAngle;
};

CalibrationJoint j1{
    &servoJ1,
    START_J1,
    START_J1,
    J1_MIN_ANGLE,
    J1_MAX_ANGLE
};

CalibrationJoint j2{
    &servoJ2,
    START_J2,
    START_J2,
    J2_MIN_ANGLE,
    J2_MAX_ANGLE
};

float moveSpeed = DEFAULT_MOVE_SPEED;
bool holdMotion = false;

uint32_t lastControlTime = 0;

// ============================================================
// Serial buffer
// ============================================================

constexpr size_t SERIAL_BUFFER_SIZE = 64;

char serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialIndex = 0;

// ============================================================
// Utilities
// ============================================================

float degToRad(float deg)
{
    return deg * PI / 180.0f;
}

// Diagnostic output for the requested servo angles; no motion control.
void printKinematics(float uL, float uR)
{
    float alpha = RIGHT_ZERO - uR;
    float gamma = uL + uR - ELBOW_OFFSET;
    float beta = alpha - (180.0f - gamma);

    float alphaRad = degToRad(alpha);
    float betaRad = degToRad(beta);

    float x = L1 * cos(alphaRad) + L2 * cos(betaRad);
    float z = L1 * sin(alphaRad) + L2 * sin(betaRad);

    Serial.print(F("alpha="));
    Serial.print(alpha, 1);
    Serial.print(F(" gamma="));
    Serial.print(gamma, 1);
    Serial.print(F(" beta="));
    Serial.print(beta, 1);
    Serial.print(F(" | FK x="));
    Serial.print(x, 1);
    Serial.print(F(" z="));
    Serial.println(z, 1);
}

float moveToward(
    float current,
    float target,
    float maxStep
) {
    if (target > current) {
        return min(current + maxStep, target);
    }

    if (target < current) {
        return max(current - maxStep, target);
    }

    return current;
}

bool isWithinJointLimits(
    float angle,
    const CalibrationJoint& joint
) {
    return angle >= joint.minAngle &&
           angle <= joint.maxAngle;
}

bool isSafePose(
    float angleJ1,
    float angleJ2
) {
    if (!isWithinJointLimits(angleJ1, j1)) {
        return false;
    }

    if (!isWithinJointLimits(angleJ2, j2)) {
        return false;
    }

    if (angleJ1 + angleJ2 < J1_J2_MIN_SUM) {
        return false;
    }

    return true;
}

struct IKResult {
    bool valid;
    float alpha;
    float beta;
    float gamma;
    float j1;
    float j2;
};

IKResult solveIK(float x, float z)
{
    IKResult result{};
    if (!isfinite(x) || !isfinite(z)) {
        return result;
    }

    float d = (x * x + z * z - L1 * L1 - L2 * L2)
              / (2.0f * L1 * L2);

    // Target outside geometric workspace.
    if (!isfinite(d) || d < -1.0f || d > 1.0f) {
        return result;
    }

    // Current physical branch.
    float q = -acos(d);
    float alpha = atan2(z, x)
                  - atan2(L2 * sin(q), L1 + L2 * cos(q));
    float beta = alpha + q;
    float alphaDeg = alpha * 180.0f / PI;
    float betaDeg = beta * 180.0f / PI;
    float gammaDeg = 180.0f + q * 180.0f / PI;
    float targetJ2 = RIGHT_ZERO - alphaDeg;
    float targetJ1 = gammaDeg + ELBOW_OFFSET - targetJ2;

    if (!isSafePose(targetJ1, targetJ2)) {
        return result;
    }

    result.valid = true;
    result.alpha = alphaDeg;
    result.beta = betaDeg;
    result.gamma = gammaDeg;
    result.j1 = targetJ1;
    result.j2 = targetJ2;
    return result;
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

// Diagnostic only: never updates requested angles or motion state.
void printIK(float x, float z)
{
    IKResult result = solveIK(x, z);
    if (!result.valid) {
        Serial.println(F("ERR IK: unreachable or unsafe target"));
        return;
    }
    printIKResult(result);
}

// 如果意外处于危险侧，只允许往安全方向退出
bool isSafeTransition(
    float currentJ1,
    float currentJ2,
    float nextJ1,
    float nextJ2
) {
    if (
        !isWithinJointLimits(nextJ1, j1) ||
        !isWithinJointLimits(nextJ2, j2)
    ) {
        return false;
    }

    float currentSum = currentJ1 + currentJ2;
    float nextSum    = nextJ1 + nextJ2;

    bool safe =
        nextSum >= J1_J2_MIN_SUM;

    bool escaping =
        nextSum > currentSum;

    return safe || escaping;
}

// 判断一个字符串是不是普通十进制数字
// 支持：95 / 95.5 / -10 / +20.5
bool looksNumeric(const char* s)
{
    if (s == nullptr || *s == '\0') {
        return false;
    }

    if (*s == '+' || *s == '-') {
        ++s;
    }

    bool hasDigit = false;
    bool hasDot = false;

    while (*s != '\0') {

        if (*s >= '0' && *s <= '9') {
            hasDigit = true;
        }
        else if (*s == '.' && !hasDot) {
            hasDot = true;
        }
        else {
            return false;
        }

        ++s;
    }

    return hasDigit;
}

// ============================================================
// Status / help
// ============================================================

void printStatus()
{
    Serial.print(F("J1 cur="));
    Serial.print(j1.currentAngle, 1);

    Serial.print(F(" req="));
    Serial.print(j1.requestedAngle, 1);

    Serial.print(F(" | J2 cur="));
    Serial.print(j2.currentAngle, 1);

    Serial.print(F(" req="));
    Serial.print(j2.requestedAngle, 1);

    Serial.print(F(" | sum="));
    Serial.print(
        j1.currentAngle + j2.currentAngle,
        1
    );

    Serial.print(F(" | speed="));
    Serial.print(moveSpeed, 1);

    Serial.print(F(" | state="));

    if (holdMotion) {
        Serial.println(F("HOLD"));
    }
    else {
        Serial.println(F("RUN"));
    }
}

void printHelp()
{
    Serial.println();
    Serial.println(F("=== MeArm Calibration Controller ==="));
    Serial.println();

    Serial.println(F("IK input (mm, print only):"));
    Serial.println(F("  <x> <z>"));
    Serial.println(F("  Example: 41.1 47.3"));

    Serial.println();

    Serial.println(F("Other commands:"));
    Serial.println(F("  XY <x> <z> (mm, moves arm via IK)"));
    Serial.println(F("  J1 <angle> (moves servo)"));
    Serial.println(F("  J2 <angle> (moves servo)"));
    Serial.println(F("  SPEED <deg/s>"));
    Serial.println(F("  HOLD"));
    Serial.println(F("  GO"));
    Serial.println(F("  STATUS"));
    Serial.println(F("  HELP"));

    Serial.println();

    Serial.print(F("J1 range: "));
    Serial.print(J1_MIN_ANGLE, 1);
    Serial.print(F(" .. "));
    Serial.println(J1_MAX_ANGLE, 1);

    Serial.print(F("J2 range: "));
    Serial.print(J2_MIN_ANGLE, 1);
    Serial.print(F(" .. "));
    Serial.println(J2_MAX_ANGLE, 1);

    Serial.print(F("Constraint: J1 + J2 >= "));
    Serial.println(J1_J2_MIN_SUM, 1);

    Serial.println();
}

// ============================================================
// Pose requests
// ============================================================

void requestPose(
    float targetJ1,
    float targetJ2
) {
    if (!isSafePose(targetJ1, targetJ2)) {

        Serial.print(F("ERR unsafe pose: J1="));
        Serial.print(targetJ1, 1);

        Serial.print(F(" J2="));
        Serial.print(targetJ2, 1);

        Serial.print(F(" sum="));
        Serial.println(
            targetJ1 + targetJ2,
            1
        );

        return;
    }

    j1.requestedAngle = targetJ1;
    j2.requestedAngle = targetJ2;

    holdMotion = false;

    Serial.print(F("OK "));
    Serial.print(targetJ1, 1);
    Serial.print(' ');
    Serial.println(targetJ2, 1);
    printKinematics(targetJ1, targetJ2);
}

void requestJ1(float target)
{
    requestPose(
        target,
        j2.requestedAngle
    );
}

void requestJ2(float target)
{
    requestPose(
        j1.requestedAngle,
        target
    );
}

// ============================================================
// Command parser
// ============================================================

void handleCommand(char* line)
{
    // 去掉行首空格
    while (*line == ' ') {
        ++line;
    }

    if (*line == '\0') {
        return;
    }

    // ========================================================
    // 1. Direct numeric input:
    //    x z in mm, IK output only
    // ========================================================

    {
        char buffer[SERIAL_BUFFER_SIZE];

        strncpy(
            buffer,
            line,
            SERIAL_BUFFER_SIZE
        );

        buffer[SERIAL_BUFFER_SIZE - 1] = '\0';

        char* arg1 = strtok(buffer, " ");
        char* arg2 = strtok(nullptr, " ");
        char* arg3 = strtok(nullptr, " ");

        if (
            arg1 != nullptr &&
            arg2 != nullptr &&
            arg3 == nullptr &&
            looksNumeric(arg1) &&
            looksNumeric(arg2)
        ) {
            printIK(atof(arg1), atof(arg2));

            return;
        }
    }

    // ========================================================
    // 2. Normal commands
    // ========================================================

    char* command = strtok(line, " ");

    if (command == nullptr) {
        return;
    }

    // XY uses the existing safe, speed-limited joint motion controller.
    if (strcmp(command, "XY") == 0) {
        char* argX = strtok(nullptr, " ");
        char* argZ = strtok(nullptr, " ");
        char* extra = strtok(nullptr, " ");
        if (
            argX == nullptr || argZ == nullptr || extra != nullptr ||
            !looksNumeric(argX) || !looksNumeric(argZ)
        ) {
            Serial.println(F("ERR usage: XY <x> <z>"));
            return;
        }

        IKResult ik = solveIK(atof(argX), atof(argZ));
        if (!ik.valid) {
            Serial.println(F("ERR IK: unreachable or unsafe target"));
            return;
        }

        printIKResult(ik);
        requestPose(ik.j1, ik.j2);
        return;
    }

    // --------------------------------------------------------
    // J1 <angle>
    // --------------------------------------------------------

    if (strcmp(command, "J1") == 0) {

        char* arg = strtok(nullptr, " ");

        if (
            arg == nullptr ||
            !looksNumeric(arg)
        ) {
            Serial.println(
                F("ERR usage: J1 <angle>")
            );

            return;
        }

        requestJ1(
            atof(arg)
        );

        return;
    }

    // --------------------------------------------------------
    // J2 <angle>
    // --------------------------------------------------------

    if (strcmp(command, "J2") == 0) {

        char* arg = strtok(nullptr, " ");

        if (
            arg == nullptr ||
            !looksNumeric(arg)
        ) {
            Serial.println(
                F("ERR usage: J2 <angle>")
            );

            return;
        }

        requestJ2(
            atof(arg)
        );

        return;
    }

    // --------------------------------------------------------
    // SPEED <deg/s>
    // --------------------------------------------------------

    if (strcmp(command, "SPEED") == 0) {

        char* arg = strtok(nullptr, " ");

        if (
            arg == nullptr ||
            !looksNumeric(arg)
        ) {
            Serial.println(
                F("ERR usage: SPEED <deg/s>")
            );

            return;
        }

        float speed = atof(arg);

        if (
            speed < MIN_MOVE_SPEED ||
            speed > MAX_MOVE_SPEED
        ) {
            Serial.print(
                F("ERR speed range ")
            );

            Serial.print(
                MIN_MOVE_SPEED,
                1
            );

            Serial.print(
                F(" .. ")
            );

            Serial.println(
                MAX_MOVE_SPEED,
                1
            );

            return;
        }

        moveSpeed = speed;

        Serial.print(
            F("OK SPEED ")
        );

        Serial.println(
            moveSpeed,
            1
        );

        return;
    }

    // --------------------------------------------------------
    // HOLD
    // --------------------------------------------------------

    if (strcmp(command, "HOLD") == 0) {

        holdMotion = true;

        // 把当前位置作为新目标，
        // 防止之后 GO 又继续追旧目标
        j1.requestedAngle =
            j1.currentAngle;

        j2.requestedAngle =
            j2.currentAngle;

        Serial.println(
            F("OK HOLD")
        );

        return;
    }

    // --------------------------------------------------------
    // GO
    // --------------------------------------------------------

    if (strcmp(command, "GO") == 0) {

        holdMotion = false;

        Serial.println(
            F("OK GO")
        );

        return;
    }

    // --------------------------------------------------------
    // STATUS
    // --------------------------------------------------------

    if (strcmp(command, "STATUS") == 0) {

        printStatus();

        return;
    }

    // --------------------------------------------------------
    // HELP
    // --------------------------------------------------------

    if (strcmp(command, "HELP") == 0) {

        printHelp();

        return;
    }

    // --------------------------------------------------------
    // Unknown command
    // --------------------------------------------------------

    Serial.print(
        F("ERR unknown command: ")
    );

    Serial.println(command);
}

// ============================================================
// Serial reader
// ============================================================

void handleSerial()
{
    while (Serial.available() > 0) {

        char c =
            static_cast<char>(
                Serial.read()
            );

        // 忽略 CR
        if (c == '\r') {
            continue;
        }

        // 收到 LF，执行整行
        if (c == '\n') {

            serialBuffer[serialIndex] =
                '\0';

            handleCommand(
                serialBuffer
            );

            serialIndex = 0;

            continue;
        }

        // 只接受普通可打印字符
        if (c >= 32 && c <= 126) {

            if (
                serialIndex <
                SERIAL_BUFFER_SIZE - 1
            ) {
                serialBuffer[serialIndex++] =
                    c;
            }
            else {
                serialIndex = 0;

                Serial.println(
                    F("ERR command too long")
                );
            }
        }
    }
}

// ============================================================
// Motion controller
// ============================================================

void updateCalibrationPose()
{
    if (holdMotion) {
        return;
    }

    if (!isSafePose(
            j1.requestedAngle,
            j2.requestedAngle
        )) {
        Serial.println(F("ERR unsafe pose"));
        return;
        }

    j1.currentAngle = j1.requestedAngle;
    j2.currentAngle = j2.requestedAngle;

    servoJ1.write(
        static_cast<int>(j1.currentAngle)
    );

    servoJ2.write(
        static_cast<int>(j2.currentAngle)
    );

    holdMotion = true;
}

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    servoJ1.attach(
        J1_SERVO_PIN
    );

    servoJ2.attach(
        J2_SERVO_PIN
    );

    // 普通 hobby servo 没有给 Arduino
    // 实际位置反馈。
    //
    // 所以上电前确保机构本身已经处于
    // 合理安全姿态。

    servoJ1.write(
        static_cast<int>(
            START_J1
        )
    );

    servoJ2.write(
        static_cast<int>(
            START_J2
        )
    );

    delay(500);

    lastControlTime =
        millis();

    Serial.println();
    Serial.println(
        F("MeArm calibration controller ready.")
    );

    Serial.println(
        F("Input: <x> <z> in mm (IK print only)")
    );

    Serial.println(
        F("Example: 41.1 47.3")
    );

    Serial.println(F("Move via IK: XY <x> <z> (mm)"));

    Serial.println(
        F("Type HELP for commands.")
    );
}

// ============================================================
// Main loop
// ============================================================

void loop()
{
    handleSerial();

    uint32_t now =
        millis();

    if (
        now - lastControlTime
        >= CONTROL_INTERVAL_MS
    ) {
        uint32_t elapsed =
            now - lastControlTime;

        lastControlTime =
            now;

        float dt =
            static_cast<float>(elapsed)
            / 1000.0f;

        updateCalibrationPose();
    }
}
