#pragma once
#include <stdint.h>
struct DebouncedButton {
    bool raw=false,stable=false,armed=false;
    uint32_t changed=0;
    void initialize(bool pressed,uint32_t now) { raw=stable=pressed; armed=!pressed; changed=now; }
    bool observe(bool pressed,uint32_t now) {
        if(pressed!=raw) { raw=pressed; changed=now; }
        if(raw==stable || now-changed<25) return false;
        stable=raw;
        if(!stable) { armed=true; return false; }
        if(!armed) return false;
        armed=false; return true;
    }
};
