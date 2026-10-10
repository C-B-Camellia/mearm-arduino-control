#pragma once

#include <Arduino.h>

constexpr unsigned long SERIAL_BAUD_RATE = 115200;
constexpr size_t SERIAL_BUFFER_SIZE = 64;
constexpr uint8_t SERIAL_BYTES_PER_LOOP = 64;
constexpr uint32_t CONTROL_INTERVAL_MS = 20;
constexpr uint32_t STARTUP_DELAY_MS = 500;

constexpr float LINE_STEP_MM = 4.0f;
constexpr uint32_t WAYPOINT_DELAY_MS = 10;

// Configuration only until speed-controlled motion is implemented.
constexpr float DEFAULT_MOVE_SPEED = 60.0f;
constexpr float MIN_MOVE_SPEED = 5.0f;
constexpr float MAX_MOVE_SPEED = 90.0f;
