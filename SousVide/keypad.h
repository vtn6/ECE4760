// Attach Keypad to PORTD

#ifndef KEYPAD
#define KEYPAD

#include <inttypes.h>

#define KEY_0 14
#define KEY_1 1
#define KEY_2 2
#define KEY_3 3
#define KEY_4 5
#define KEY_5 6
#define KEY_6 7
#define KEY_7 9
#define KEY_8 10
#define KEY_9 11
#define KEY_S 13
#define KEY_P 15

#define KEY_A 4
#define KEY_B 8
#define KEY_C 12
#define KEY_D 16



void KeypadDebounce(void);
uint8_t KeypadKey(void);
unsigned int KeypadInt(void);
uint8_t KeypadFin(void);

#endif
