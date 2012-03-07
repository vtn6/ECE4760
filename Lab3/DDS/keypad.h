// Attach Keypad to PORTD

#ifndef KEYPAD
#define KEYPAD

#include <inttypes.h>

#define KEY_0 0x7d
#define KEY_1 0xee
#define KEY_2 0xed
#define KEY_3 0xeb
#define KEY_4 0xde
#define KEY_5 0xdd
#define KEY_6 0xdb
#define KEY_7 0xbe
#define KEY_8 0xbd
#define KEY_9 0xbb

#define KEY_A 0xe7
#define KEY_B 0xd7
#define KEY_C 0xb7
#define KEY_D 0x77
#define KEY_S 0x7e
#define KEY_P 0x7b


void KeypadDebounce(void);
uint8_t KeypadKey(void);
int KeypadInt(void);
uint8_t KeypadFin(void);

#endif
