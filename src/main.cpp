//
// Created by 12530 on 2026/9/27.
//
#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("ok");
}

void loop() {
    if (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        Serial.print("RX: ");
        Serial.println(c);
    }
}