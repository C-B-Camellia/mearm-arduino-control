#pragma once
#include <stdint.h>
void initializeLocalDisplay();
void updateLocalDisplay(uint32_t now,uint8_t lastButton,uint8_t sequence,bool overflowed);
