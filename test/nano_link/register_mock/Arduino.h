#pragma once
#include <stdint.h>
#define F_CPU 16000000UL
#define INPUT_PULLUP 2
#define _BV(bit) (uint8_t(1U<<(bit)))
enum { PB2=2, PCINT2=2, PCIF0=0, PCIE0=0, OCF2A=1, OCIE2A=1, WGM21=1, CS21=1, CS20=0 };
extern volatile uint8_t PINB,PCMSK0,PCICR,PCIFR,TCNT2,OCR2A,TIFR2,TIMSK2,TCCR2A,TCCR2B,SREG;
extern uint8_t configuredPin,configuredMode;
inline void pinMode(uint8_t pin,uint8_t mode) { configuredPin=pin; configuredMode=mode; }
