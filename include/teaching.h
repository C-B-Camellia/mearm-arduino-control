#pragma once

#include "kinematics.h"
#include <Arduino.h>

constexpr uint8_t TEACH_POINT_COUNT = 5;

struct TeachPath {
    Point3D points[TEACH_POINT_COUNT];
    uint8_t count = 0;
};