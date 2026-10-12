#include "serial_format.h"
#include <math.h>

void printDecimal(float value,uint8_t digits) {
    if(!isfinite(value)) { Serial.print(isnan(value)?F("nan"):F("inf")); return; }
    uint8_t scale=digits==0?1:digits==1?10:100;
    // The protocol reports angles/mm and settings, not arbitrary large floats.
    // Reject diagnostic overflow before any conversion (also covers rounding).
    float scaled=fabsf(value)*scale+0.5f;
    if(scaled>=4294967040.0f) { Serial.print(F("ovf")); return; }
    uint32_t rounded=static_cast<uint32_t>(scaled);
    if(value<0) Serial.print('-');
    Serial.print(rounded/scale);
    if(digits) {
        Serial.print('.');
        uint8_t fraction=rounded%scale;
        if(digits>1) Serial.print(static_cast<char>('0'+fraction/10));
        Serial.print(static_cast<char>('0'+fraction%10));
    }
}
