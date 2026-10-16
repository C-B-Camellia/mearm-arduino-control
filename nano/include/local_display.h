#pragma once
#include <stdint.h>
#include "panel_mode.h"
void initializeLocalDisplay();
void selectLocalDisplayPage(PanelMode page);
void updateLocalDisplay(uint32_t now,PanelMode mode,bool overflowed);
