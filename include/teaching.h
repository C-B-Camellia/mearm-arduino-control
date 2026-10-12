#pragma once

#include "kinematics.h"
#include <Arduino.h>

constexpr uint8_t TEACH_POINT_COUNT = 5;

struct TeachPath {
    Point3D points[TEACH_POINT_COUNT];
    uint8_t count = 0;
};
struct TeachPoint { float x,y; };
struct TeachSettings { float drawZ,travelZ,teachZ; bool drawValid,travelValid,teachValid; };
bool beginTeach();
bool captureTeachHeight(bool drawing);
bool captureTeachAlignment();
bool beginMarking();
bool markTeachPoint();
bool clearTeachPoints();
bool undoTeachPoint();
bool exitTeach();
void initializeTeaching();
uint8_t teachPointCount();
const TeachSettings& teachSettings();
bool setTeachOrder(const uint8_t (&order)[TEACH_POINT_COUNT]);
bool getTeachPoint(uint8_t orderedIndex,TeachPoint& out);
bool teachingActive();
bool markingActive();
bool drawSettingsValid();
