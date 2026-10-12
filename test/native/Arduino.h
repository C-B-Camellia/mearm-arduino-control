#pragma once

// Desktop-only Arduino substitutes. These are never compiled into Uno firmware.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using std::isfinite;
using std::min;
using std::max;

#define PI 3.1415926535897932384626433832795
#define A0 14
#define A1 15
#define A2 16
#define A3 17

class __FlashStringHelper;
#define F(text) reinterpret_cast<const __FlashStringHelper*>(text)

struct ServoWrite {
    int pin;
    int angle;
    uint32_t time;
};

extern uint32_t simulatedTime;
extern std::vector<ServoWrite> servoWrites;
extern std::vector<int> attachedPins;
extern std::vector<uint32_t> delayCalls;
extern int analogValues[4];
inline int analogRead(int pin) { return analogValues[pin - A0]; }

inline uint32_t millis() { return simulatedTime; }
inline void delay(uint32_t duration)
{
    delayCalls.push_back(duration);
    simulatedTime += duration;
}

struct SerialStub {
    std::string input;
    std::string output;
    size_t bytesRead = 0;
    unsigned long baud = 0;

    void begin(unsigned long value) { baud = value; }
    int available() const { return static_cast<int>(input.size()); }
    int read()
    {
        unsigned char value = input.front();
        input.erase(0, 1);
        ++bytesRead;
        return value;
    }
    void print(const __FlashStringHelper* value) { output += reinterpret_cast<const char*>(value); }
    void print(const char* value) { output += value; }
    void print(char* value) { output += value; }
    void print(char value) { output += value; }
    template<class T> void print(T value) { output += std::to_string(value); }
    void print(float value, int precision)
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        output += stream.str();
    }
    void println() { output += "\r\n"; }
    template<class T> void println(T value) { print(value); println(); }
    void println(float value, int precision) { print(value, precision); println(); }
};

extern SerialStub Serial;
