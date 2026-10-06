#include <Arduino.h>

#include "arm.h"
#include "commands.h"
#include "config.h"
#include "motion.h"

namespace {
uint32_t lastControlTime = 0;
}

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    initializeArm();

    // Hobby servos do not report actual position. Start from a safe physical pose.
    delay(STARTUP_DELAY_MS);
    lastControlTime = millis();

    Serial.println();
    Serial.println(F("MeArm calibration controller ready."));
    Serial.println(F("Input: IK <x> <z> or <x> <z> in mm (print only)"));
    Serial.println(F("Move: XY <x> <z>; trajectory: RUNLINE <x> <z>"));
    Serial.println(F("Task control: HOLD / GO / CANCEL; HELP for commands."));
}

void loop()
{
    pollSerial();

    // Future input hooks: joystick/manual requests and external button messages.
    // They must respect isMotionBusy() and submit poses through arm.h.
    uint32_t now = millis();
    bool controlDue = static_cast<uint32_t>(now - lastControlTime) >= CONTROL_INTERVAL_MS;
    if (controlDue) {
        lastControlTime = now;
        if (!isMotionBusy() && !applyPendingPose()) {
            Serial.println(F("ERR unsafe pose"));
        }
    }
    updateMotion(millis(), controlDue);

    // Future recording hook: sample the commanded pose after motion updates.
}
