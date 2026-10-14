#include <Arduino.h>
#include <iostream>
#include <cstdlib>
volatile uint8_t PINB=0,PCMSK0=0,PCICR=0,PCIFR=0,TCNT2=0,OCR2A=0,TIFR2=0,TIMSK2=0,TCCR2A=0,TCCR2B=0,SREG=0;
uint8_t configuredPin=255,configuredMode=0;
#include "../../src/pin_uart.cpp"
namespace {
unsigned checks=0;
void check(bool ok,int line) { ++checks; if(!ok) { std::cerr<<"FAIL UART "<<line<<"\n"; std::exit(1); } }
#define CHECK(x) check((x),__LINE__)
void receive(uint8_t value,bool stop=true) {
    PINB=0; PCINT0_vect();
    CHECK(!(PCMSK0&_BV(PCINT2)) && (TIMSK2&_BV(OCIE2A)) && OCR2A==77);
    for(uint8_t i=0;i<8;++i) {
        PINB=(value&(1<<i))?_BV(PB2):0;
        TIMER2_COMPA_vect();
        CHECK(OCR2A==51 && (TIMSK2&_BV(OCIE2A)));
    }
    PINB=stop?_BV(PB2):0; TIMER2_COMPA_vect();
    CHECK((PCMSK0&_BV(PCINT2)) && !(TIMSK2&_BV(OCIE2A)));
}
}
int main() {
    SREG=0x80; PCMSK0=0x80; PCICR=0x04;
    initializePinUart();
    CHECK(configuredPin==10 && configuredMode==INPUT_PULLUP && SREG==0x80);
    CHECK(PCMSK0==0x84 && PCICR==0x05 && PCIFR==_BV(PCIF0));
    CHECK(TCCR2A==_BV(WGM21) && TCCR2B==(_BV(CS21)|_BV(CS20)));
    CHECK(readPinUart()==-1 && !pinUartOverflow());
    // HIGH/rising edges and other PB changes while RX HIGH do not start a byte.
    PINB=_BV(PB2)|1; PCINT0_vect(); CHECK(TIMSK2==0 && PCMSK0==0x84);
    for(unsigned i=0;i<256;++i) { receive(i); CHECK(readPinUart()==int(i)); CHECK(readPinUart()==-1); }
    receive(0xA5,false); CHECK(pinUartOverflow() && readPinUart()==-1 && !pinUartOverflow());
    // A periodic HELLO immediately followed by PRESS is 16 bytes. The old
    // 15-byte usable ring lost the stop event when a control/record tick ran.
    for(uint8_t i=0;i<16;++i) receive(i);
    CHECK(!pinUartOverflow());
    for(uint8_t i=0;i<16;++i) CHECK(readPinUart()==i);
    for(uint8_t i=0;i<31;++i) receive(i);
    receive(99); CHECK(pinUartOverflow()); CHECK(readPinUart()==-1); // Flush stale queue.
    receive(0x5A); CHECK(readPinUart()==0x5A && !pinUartOverflow());
    receive(1); initializePinUart(); CHECK(readPinUart()==-1);
    SREG=0; CHECK(!pinUartOverflow() && SREG==0); // Preserve caller interrupt state.
    std::cout<<"PASS D10 UART register model: "<<checks<<" checks\n";
}
