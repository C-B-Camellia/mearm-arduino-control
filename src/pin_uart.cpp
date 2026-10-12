#include "pin_uart.h"
#if defined(__AVR__) && defined(MEARM_BUTTON_RX)
#include <Arduino.h>
#include <avr/interrupt.h>
static_assert(F_CPU==16000000UL && MEARM_BUTTON_RX==2,"RX requires Uno D2/INT0 at 16 MHz");
static_assert(MEARM_BUTTON_BAUD==9600,"sampled UART supports 9600 baud only");
// Timer0 remains millis, Timer1 remains Servo. Timer2 only samples RX.
// No delay loop inside an ISR and no whole-byte interrupt blackout.
namespace {
volatile uint8_t bytes[16],head=0,tail=0,shift=0,bit=0;
volatile bool overflow=false;
}
ISR(INT0_vect) {
    EIMSK&=~_BV(INT0);
    shift=bit=0; TCNT2=0; OCR2A=77; // First data sample at 1.5 bits, 2 us/count.
    TIFR2=_BV(OCF2A); TIMSK2|=_BV(OCIE2A);
}
ISR(TIMER2_COMPA_vect) {
    if(bit<8) {
        shift=(shift>>1) | ((PIND&_BV(PD2))?0x80:0);
        ++bit; OCR2A=51; // 104 us, +0.16% baud timing error.
    } else {
        if(PIND&_BV(PD2)) {
            uint8_t next=(head+1)&15;
            if(next!=tail) { bytes[head]=shift; head=next; } else overflow=true;
        } else overflow=true;
        TIMSK2&=~_BV(OCIE2A); EIFR=_BV(INTF0); EIMSK|=_BV(INT0);
    }
}
void initializePinUart() {
    pinMode(2,INPUT_PULLUP);
    uint8_t saved=SREG; cli();
    head=tail=0; overflow=false;
    TCCR2A=_BV(WGM21); TCCR2B=_BV(CS21)|_BV(CS20); TIMSK2=0;
    EICRA=(EICRA&~(_BV(ISC00)|_BV(ISC01)))|_BV(ISC01);
    EIFR=_BV(INTF0); EIMSK|=_BV(INT0); SREG=saved;
}
int readPinUart() {
    if(head==tail) return -1;
    uint8_t value=bytes[tail]; tail=(tail+1)&15; return value;
}
bool pinUartOverflow() {
    uint8_t saved=SREG; cli(); bool value=overflow; overflow=false; SREG=saved; return value;
}
#else
void initializePinUart() {}
int readPinUart() { return -1; }
bool pinUartOverflow() { return false; }
#endif
