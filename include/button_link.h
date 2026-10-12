#pragma once
#include <Arduino.h>
void initializeButtonLink();
void pollButtonLink();
void feedButtonByte(char byte,uint32_t now);
bool buttonLinkConfigured();
