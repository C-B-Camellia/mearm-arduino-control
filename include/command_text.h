#pragma once
#include <string.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#define commandIs(text,literal) (strcmp_P((text),PSTR(literal))==0)
#define commandNames(literal) PSTR(literal)
#else
#define commandIs(text,literal) (strcmp((text),(literal))==0)
#define commandNames(literal) (literal)
#endif
int commandIndex(const char* text,const char* names);
