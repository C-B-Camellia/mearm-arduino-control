#pragma once

#include <Arduino.h>
#include "joystick.h"

enum class ControlMode : uint8_t {
    Joint,
    Cartesian
};

// 读取当前控制模式
ControlMode getControlMode();

// 请求切换控制模式
// true  = 切换成功，或原本就在该模式
// false = 当前不允许切换
bool setControlMode(ControlMode newMode);

void updateManualControl(uint32_t now);
void updateManualControl(uint32_t now,const JoystickInput& input);
void syncManualTarget(uint32_t now);
void initializeManualControl(uint32_t now);
