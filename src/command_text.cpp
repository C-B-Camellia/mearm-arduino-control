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
