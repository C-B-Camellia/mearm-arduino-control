//
// Created by 12530 on 2026/9/27.
//
#include <Arduino.h>
#include <Servo.h>


struct JointControl {
    int joystickPin;
    int servoPin;
    int center;

    float targetAngle;
    float minAngle;
    float maxAngle;

    int directionSign;
};

struct JoystickInput {
    int raw;
    int delta;
    int speedLevel;
    int direction;
    float velocity;
};

JoystickInput readJoystick(const JointControl& joint) {
    JoystickInput input;

    input.raw = analogRead(joint.joystickPin);
    input.delta = input.raw - joint.center;

    if (abs(input.delta) <= 10) {
        input.speedLevel = 0;
        input.direction = 0;
        input.velocity = 0.0f;
        return input;
    }

    input.direction = input.delta > 0 ? -1 : 1;

    if (abs(input.delta) < 500) {
        input.speedLevel = 1;
        input.velocity = 45.0f;
    } else {
        input.speedLevel = 2;
        input.velocity = 90.0f;
    }

    input.velocity *= input.direction * joint.directionSign;

    return input;
}

constexpr float J1_J2_MIN_SUM = 148.0f;

void updateCoupledJoints(
    JointControl& j1,
    JointControl& j2,
    Servo& s1,
    Servo& s2,
    const JoystickInput& in1,
    const JoystickInput& in2,
    float dt
) {
    float next1 = j1.targetAngle + in1.velocity * dt;
    float next2 = j2.targetAngle + in2.velocity * dt;

    next1 = constrain(next1, j1.minAngle, j1.maxAngle);
    next2 = constrain(next2, j2.minAngle, j2.maxAngle);

    float currentSum = j1.targetAngle + j2.targetAngle;
    float nextSum = next1 + next2;

    bool safe = nextSum >= J1_J2_MIN_SUM;
    bool escaping = nextSum > currentSum;

    if (safe || escaping) {
        j1.targetAngle = next1;
        j2.targetAngle = next2;
    }

    s1.write(static_cast<int>(j1.targetAngle));
    s2.write(static_cast<int>(j2.targetAngle));
}

void updateJoint(
    JointControl& joint,
    Servo& servo,
    const JoystickInput& input,
    float dt
) {
    if (joint.servoPin == 8 || joint.servoPin == 7) {

    }
    joint.targetAngle += input.velocity * dt;

    joint.targetAngle = constrain(
        joint.targetAngle,
        joint.minAngle,
        joint.maxAngle
    );

    servo.write(static_cast<int>(joint.targetAngle));
}

JointControl joints[4] = {
    {
        .joystickPin = A0,
        .servoPin = 9,
        .center = 506,
        .targetAngle = 90.0f,
        .minAngle = 0.0f,
        .maxAngle = 180.0f,
        .directionSign = 1
    },
    {
        .joystickPin = A1,
        .servoPin = 8,
        .center = 519,
        .targetAngle = 120.0f,
        .minAngle = 20.0f,
        .maxAngle = 125.0f,
        .directionSign = -1
    },
    {
        .joystickPin = A3,
        .servoPin = 7,
        .center = 519,
        .targetAngle = 60.0f,
        .minAngle = 40.0f,
        .maxAngle = 130.0f,
        .directionSign = 1
    },
    {
        .joystickPin = A2,
        .servoPin = 6,
        .center = 515,
        .targetAngle = 50.0f,
        .minAngle = 0.0f,
        .maxAngle = 180.0f,
        .directionSign = 1
    }
};
Servo servos[4];
JoystickInput inputs[4];

unsigned long lastControlTime = 0;
unsigned long lastDebugTime = 0;

const unsigned long CONTROL_INTERVAL = 20;
const unsigned long DEBUG_INTERVAL = 400;

void setup() {
    Serial.begin(115200);
    Serial.println("alright");

    for (int i = 0; i < 4; ++i) {
        servos[i].attach(joints[i].servoPin);
        servos[i].write(static_cast<int>(joints[i].targetAngle));
    }

    lastControlTime = millis();
    lastDebugTime = millis();
}

void loop() {
    unsigned long now = millis();

    if (now - lastControlTime >= CONTROL_INTERVAL) {
        float dt = (now - lastControlTime) / 1000.0f;
        lastControlTime = now;

        for (int i = 0; i < 4; ++i) {
            inputs[i] = readJoystick(joints[i]);
        }

        updateJoint(joints[0], servos[0], inputs[0], dt);

        updateCoupledJoints(
            joints[1], joints[2],
            servos[1], servos[2],
            inputs[1], inputs[2],
            dt
        );

        updateJoint(joints[3], servos[3], inputs[3], dt);
    }

    if (now - lastDebugTime >= DEBUG_INTERVAL) {
        lastDebugTime = now;

        Serial.print("t=");
        Serial.print(now);

        for (int i = 0; i < 4; ++i) {
            Serial.print(" | J");
            Serial.print(i);

            Serial.print(" raw=");
            Serial.print(inputs[i].raw);

            Serial.print(" d=");
            Serial.print(inputs[i].delta);

            Serial.print(" lvl=");
            Serial.print(inputs[i].speedLevel);

            Serial.print(" v=");
            Serial.print(inputs[i].velocity, 1);

            Serial.print(" tgt=");
            Serial.print(joints[i].targetAngle, 1);
        }

        Serial.println();
    }
}