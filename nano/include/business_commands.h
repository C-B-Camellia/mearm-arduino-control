#pragma once
#include <stdint.h>
// IDs 1..20 are the version-2 wire business commands. Do not renumber.
enum class AppAction : uint8_t {
    PickNext=1, RecordToggle=2, Play=3, Home=4,
    TeachStart=5, DrawHeight=6, TeachAlignment=7, TravelHeight=8,
    Marking=9, TeachExit=10, Mark=11, Undo=12, Clear=13,
    DrawLine=14, DrawShape=15, Polyline=16, Curve=17,
    Hold=18, Go=19, Cancel=20,
    Pick=21, RecordStart=22, RecordStop=23
};
