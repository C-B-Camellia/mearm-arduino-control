#pragma once

#include <Arduino.h>

constexpr unsigned long SERIAL_BAUD_RATE = 115200;
constexpr size_t SERIAL_BUFFER_SIZE = 64;
constexpr uint8_t SERIAL_BYTES_PER_LOOP = 64;
#ifdef MEARM_TIMING_INTERVAL_MS
constexpr uint32_t CONTROL_INTERVAL_MS = MEARM_TIMING_INTERVAL_MS;
#else
constexpr uint32_t CONTROL_INTERVAL_MS = 20;
#endif
constexpr uint32_t STARTUP_DELAY_MS = 500;

constexpr float LINE_STEP_MM = 4.0f;
constexpr uint32_t WAYPOINT_DELAY_MS = 10;

constexpr float MANUAL_JOINT_SLOW = 45.0f;
constexpr float MANUAL_JOINT_FAST = 90.0f;
constexpr float MANUAL_CART_SLOW = 50.0f;
constexpr float MANUAL_CART_FAST = 100.0f;
constexpr uint32_t MANUAL_MAX_DT_MS = 40;
// Automatic task speed is independent of the joystick multiplier.
constexpr float DEFAULT_MOVE_SPEED = 60.0f;
constexpr float MIN_MOVE_SPEED = 5.0f;
constexpr float MAX_MOVE_SPEED = 90.0f;
