#include "pin_uart.h"
#if defined(__AVR__) && defined(MEARM_NANO_LINK)
#include <Arduino.h>
#include <avr/interrupt.h>
static_assert(F_CPU==16000000UL && MEARM_BUTTON_RX==10,"RX requires Uno D10/PB2 at 16 MHz");
static_assert(MEARM_BUTTON_BAUD==9600,"sampled UART supports 9600 baud only");
// Timer0: millis; Timer1: Servo; Timer2: 9600-baud RX sampling only.
// D10/PB2 is PCINT2 in the PCINT0 group, not the PCINT2 vector.
namespace {
volatile uint8_t bytes[16],head=0,tail=0,shift=0,bit=0;
volatile bool overflow=false;
}
ISR(PCINT0_vect) {
    // Pin-change interrupts fire on both edges; only LOW starts reception.
    if(PINB&_BV(PB2)) return;
    PCMSK0&=~_BV(PCINT2); // Mask D10 while data bits are sampled.
    shift=bit=0; TCNT2=0; OCR2A=77; // First sample: 156 us (1.5 bits).
    TIFR2=_BV(OCF2A); TIMSK2|=_BV(OCIE2A);
}
ISR(TIMER2_COMPA_vect) {
    if(bit<8) {
        shift=(shift>>1) | ((PINB&_BV(PB2))?0x80:0);
        ++bit; OCR2A=51; // Subsequent samples: 104 us.
    } else {
        if(PINB&_BV(PB2)) {
            uint8_t next=(head+1)&15;
            if(next!=tail) { bytes[head]=shift; head=next; } else overflow=true;
        } else overflow=true; // Invalid stop bit; discard and require HELLO.
        TIMSK2&=~_BV(OCIE2A);
        PCIFR=_BV(PCIF0); PCMSK0|=_BV(PCINT2);
    }
}
void initializePinUart() {
    pinMode(10,INPUT_PULLUP);
    uint8_t saved=SREG; cli();
    head=tail=0; overflow=false;
    TCCR2A=_BV(WGM21); TCCR2B=_BV(CS21)|_BV(CS20); TIMSK2=0;
    PCIFR=_BV(PCIF0); PCMSK0|=_BV(PCINT2); PCICR|=_BV(PCIE0);
    SREG=saved;
}
int readPinUart() {
    if(head==tail) return -1;
    uint8_t value=bytes[tail]; tail=(tail+1)&15; return value;
}
bool pinUartOverflow() {
    uint8_t saved=SREG; cli();
    bool value=overflow; overflow=false;
    if(value) tail=head; // Never parse pre-error queued frames as a new session.
    SREG=saved; return value;
}
#endif
