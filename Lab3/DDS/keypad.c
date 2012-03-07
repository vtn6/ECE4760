#include "keypad.h"

#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#define RELEASED 0
#define MAYBEPUSHED 1
#define PUSHED 2
#define MAYBERELEASED 3
#define DEBOUNCE_TIME 30

unsigned char keytbl[16] = {0xee, 0xed, 0xeb, 0xe7, 
							0xde, 0xdd, 0xdb, 0xd7, 
							0xbe, 0xbd, 0xbb, 0xb7, 
							0x7e, 0x7d, 0x7b, 0x77};
int KeypadInput = 0;
char KeypadFinished = 0; // set to 1 if string has been fully entered

volatile uint8_t curKey;		//current key pressed 
volatile uint8_t keyState = RELEASED;		//current state for Debounce State Machine
volatile uint8_t checkKey;		//key that might be pressed
volatile uint8_t debounceTime;	//the time at which the program should debounce


uint8_t KeypadScan(void) {
	uint8_t key;
	uint8_t butnum;
	//get lower nibble
	DDRD = 0x0f;
	PORTD = 0xf0; 
	_delay_us(5);
	key = PIND;
	
	//get upper nibble
	DDRD = 0xf0;
	PORTD = 0x0f; 
	_delay_us(5);
	key = key | PIND;
	
	//find matching keycode in keytbl
	if (key != 0xff) {
		for (butnum=0; butnum<16; butnum++) {   
			if (keytbl[butnum]==key)  break;   
		}

		if (butnum==16) butnum=0;
		else butnum++;	   //adjust by one to make range 1-16
	}
	else butnum=0;
	
	return butnum;
}

void KeypadAppend(uint8_t key) {
	uint8_t c = 0;
	// reset string
	if(KeypadFinished == 1) {
		KeypadFinished = 0;
		KeypadInput = 0;
	}
	switch(key) {
		case KEY_0:
			c = 0;
			break;
		case KEY_1:
			c = 1;
			break;
		case KEY_2:
			c = 2;
			break;
		case KEY_3:
			c = 3;
			break;
		case KEY_4:
			c = 4;
			break;
		case KEY_5:
			c = 5;
			break;
		case KEY_6:
			c = 6;
			break;
		case KEY_7:
			c = 7;
			break;
		case KEY_8:
			c = 8;
			break;
		case KEY_9:
			c = 9;
			break;
		case KEY_P:
		case KEY_S:
		case KEY_A:
		case KEY_B:
		case KEY_C:
		case KEY_D:
			KeypadFinished = 1;
			break;
	}
	// append to integer
	if(c) {
		KeypadInput = (KeypadInput * 10) + c;
	}
}

// Execute this every 1ms
void KeypadDebounce(void) {
	debounceTime--;
	uint8_t key = KeypadScan(); //Scan the keypad
	switch(keyState){
	
		case RELEASED:
			if (key){
				keyState = MAYBEPUSHED;
				debounceTime = DEBOUNCE_TIME;
				checkKey = key;
			}
			break;
		case MAYBEPUSHED:
			if (debounceTime == 0) {
				if (key && key == checkKey) {
					keyState = PUSHED;
					curKey = key;
					KeypadAppend(key);
				} else {
					keyState = RELEASED;
				}
			}
			break;
		case PUSHED:
			if (!key){
				keyState = MAYBERELEASED;
				debounceTime = DEBOUNCE_TIME;
			}
			break;
		case MAYBERELEASED:
			if (debounceTime == 0) {
				if (!key) {
					keyState = RELEASED;
				} else {
					keyState = PUSHED;
				}
			}
			break;
	}
}


// get last pressed key
uint8_t KeypadKey(void) {
	uint8_t tmpKey = curKey;
	if (curKey){
		curKey = 0;
	}
	return tmpKey;
}


// get pressed integer (delimited before and after by any non-integer)
int KeypadInt(void) {
	return KeypadInput;
}

uint8_t KeypadFin(void) {
	return KeypadFinished;
}
