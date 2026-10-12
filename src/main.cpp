#include <Arduino.h>

#include "arm.h"
#include "commands.h"
#include "config.h"
#include "motion.h"
#include "manual_control.h"
#include "control_error.h"
#include "recording.h"
#include "app_controller.h"
#include "button_link.h"

namespace {
uint32_t lastControlTime = 0;
}

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    initializeArm();
    initializeApplication(); initializeButtonLink();

    // Hobby servos do not report actual position. Start from a safe physical pose.
    delay(STARTUP_DELAY_MS);
    lastControlTime = millis();
    initializeManualControl(lastControlTime);

    Serial.println();
    Serial.println(F("MeArm dev; HELP"));
}

void loop()
{
    pollSerial();
    pollButtonLink();

    // One control clock, one complete output frame; missed ticks never burst.
    uint32_t now = millis();
    bool controlDue = static_cast<uint32_t>(now - lastControlTime) >= CONTROL_INTERVAL_MS;
    if (controlDue) {
        lastControlTime += ((now-lastControlTime)/CONTROL_INTERVAL_MS)*CONTROL_INTERVAL_MS;
    }
    updateMotion(now, controlDue);
    if(controlDue) {
        if(!isMotionBusy()) updateManualControl(now);
        if(!applyPendingPose()) { rejectControl(ControlError::Unsafe); cancelMotion(); printControlError(); }
        updateRecording(now,commandedPose());
    }

    // Recording observes the output command; it is not position feedback.
}
