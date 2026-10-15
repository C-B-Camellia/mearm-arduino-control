#include "command_text.h"
int commandIndex(const char* text,const char* names) {
    for(int index=0;;++index) {
#ifdef __AVR__
        if(!pgm_read_byte(names)) return -1;
        if(!strcmp_P(text,names)) return index;
        while(pgm_read_byte(names++)) {}
#else
        if(!*names) return -1;
        if(!strcmp(text,names)) return index;
        while(*names++) {}
#endif
    }
}

#include "serial_format.h"
void printCommandName(const char* names,uint8_t index) {
    while(index--) {
#ifdef __AVR__
        while(pgm_read_byte(names++)) {}
#else
        while(*names++) {}
#endif
    }
#ifdef __AVR__
    printLine(reinterpret_cast<const __FlashStringHelper*>(names));
#else
    Serial.println(names);
#endif
}
