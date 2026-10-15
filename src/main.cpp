#include "serial_format.h"
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
#include "pick_place.h"

namespace {
uint32_t lastControlTime = 0;
#ifdef MEARM_TIMING_DIAG
// Bounded telemetry: timing only, no extra pose calculations or servo writes.
struct TimingStats {
    uint32_t lastTickUs;
    uint32_t minGapUs, maxGapUs;
    uint32_t maxManualUs, maxApplyUs, maxControlUs;
    uint16_t ticks, over25ms, over35ms;
    uint32_t lastReportMs;
};
TimingStats perf{};

void reportTiming(uint32_t now) {
    uint32_t duration = now - perf.lastReportMs;
    if (duration < 2000) return;
    Serial.print(F("PERF ms=")); Serial.print(duration);
    Serial.print(F(" ticks=")); Serial.print(perf.ticks);
    Serial.print(F(" gapMinUs=")); Serial.print(perf.minGapUs);
    Serial.print(F(" gapMaxUs=")); Serial.print(perf.maxGapUs);
    Serial.print(F(" manualMaxUs=")); Serial.print(perf.maxManualUs);
    Serial.print(F(" applyMaxUs=")); Serial.print(perf.maxApplyUs);
    Serial.print(F(" controlMaxUs=")); Serial.print(perf.maxControlUs);
    Serial.print(F(" over25=")); Serial.print(perf.over25ms);
    Serial.print(F(" over35=")); Serial.println(perf.over35ms);
    perf.lastReportMs = millis(); // Report printing is not included in the next window duration.
    perf.minGapUs=0; perf.maxGapUs=0; perf.maxManualUs=0; perf.maxApplyUs=0; perf.maxControlUs=0;
    perf.ticks=0; perf.over25ms=0; perf.over35ms=0;
}
#endif
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
#ifdef MEARM_TIMING_DIAG
    perf.lastReportMs = millis();
#endif

}

void loop()
{
    pollSerial();
    pollButtonLink();
    updatePickPersistence();

    // One control clock, one complete output frame; missed ticks never burst.
    uint32_t now = millis();
    bool controlDue = static_cast<uint32_t>(now - lastControlTime) >= CONTROL_INTERVAL_MS;
#ifdef MEARM_TIMING_DIAG
    uint32_t controlStartUs=0;
    if (controlDue) {
        controlStartUs=micros();
        if (perf.lastTickUs) {
            uint32_t gap=controlStartUs-perf.lastTickUs;
            if (!perf.minGapUs || gap<perf.minGapUs) perf.minGapUs=gap;
            if (gap>perf.maxGapUs) perf.maxGapUs=gap;
            if (gap>25000 && perf.over25ms<65535) ++perf.over25ms;
            if (gap>35000 && perf.over35ms<65535) ++perf.over35ms;
        }
        perf.lastTickUs=controlStartUs;
        if (perf.ticks<65535) ++perf.ticks;
    }
#endif
    if (controlDue) {
        lastControlTime += ((now-lastControlTime)/CONTROL_INTERVAL_MS)*CONTROL_INTERVAL_MS;
    }
    updateMotion(now, controlDue);
    if(controlDue) {
#ifdef MEARM_TIMING_DIAG
        uint32_t manualStartUs=micros();
#endif
        if(!isMotionBusy()) updateManualControl(now);
#ifdef MEARM_TIMING_DIAG
        uint32_t manualCostUs=micros()-manualStartUs;
        if (manualCostUs>perf.maxManualUs) perf.maxManualUs=manualCostUs;
#endif
#ifdef MEARM_TIMING_DIAG
        uint32_t applyStartUs=micros();
#endif
        if(!applyPendingPose()) { rejectControl(ControlError::Unsafe); cancelMotion(); printControlError(); }
#ifdef MEARM_TIMING_DIAG
        uint32_t applyCostUs=micros()-applyStartUs;
        if (applyCostUs>perf.maxApplyUs) perf.maxApplyUs=applyCostUs;
#endif
        updateRecording(now,commandedPose());
#ifdef MEARM_TIMING_DIAG
        uint32_t costUs=micros()-controlStartUs;
        if (costUs>perf.maxControlUs) perf.maxControlUs=costUs;
#endif
    }
#ifdef MEARM_TIMING_DIAG
    reportTiming(millis());
#endif

    // Recording observes the output command; it is not position feedback.
}
