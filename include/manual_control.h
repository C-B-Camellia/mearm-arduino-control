#pragma once

#include <Arduino.h>

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