//
// Created by 12530 on 2026/9/27.
//
#include <Arduino.h>
#include <Servo.h>


Servo baseServo;

unsigned long lastUpdateTime = 0;

struct JointControl {
    int joystickPin;
    int servoPin;
    int center;

    float targetAngle;
    float minAngle;
    float maxAngle;

    int directionSign;
};

JointControl base = {
    .joystickPin = A0,
    .servoPin = 9,
    .center = 506,
    .targetAngle = 90.0f,
    .minAngle = 0.0f,
    .maxAngle = 180.0f,
    .directionSign = 1
};

float joystickVelocity(const JointControl& joint) {
    int raw = analogRead(joint.joystickPin);
    int delta = raw - joint.center;

    if (abs(delta) <= 5) return 0.0f;

    float speed;

    if (abs(delta) < 500) speed = 90.0f;
    else speed = 180.0f;

    int direction = delta > 0 ? -1 : 1;

    return direction * joint.directionSign * speed;
}

void updateJoint(JointControl& joint, Servo& servo, float dt) {
    float velocity = joystickVelocity(joint);

    joint.targetAngle += velocity * dt;

    joint.targetAngle = constrain(
        joint.targetAngle,
        joint.minAngle,
        joint.maxAngle
    );

    servo.write(static_cast<int>(joint.targetAngle));
}

void setup() {
    Serial.begin(115200);
    Serial.println("ok");

    baseServo.attach(base.servoPin);
    baseServo.write(static_cast<int>(base.targetAngle));
    lastUpdateTime = millis();
}

void loop() {
    unsigned long now = millis();

    float dt = (now - lastUpdateTime) / 1000.0f;
    lastUpdateTime = now;

    updateJoint(base, baseServo, dt);
}