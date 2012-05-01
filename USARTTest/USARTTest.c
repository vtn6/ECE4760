//Test for USART connection

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd_lib.h"
#include "lcd_lib.c"

//Constants
uint8_t baudH = 127;
uint8_t baudL = 127;

//Variables
volatile uint8_t msg;
uint8_t LCDBuffer[17];  // LCD display buffer

//Function signatures
void usartInit(void);
void initLCD(void);

//Interrupt Handlers

ISR (USART0_RX_vect){
	msg = UDR0;
	sprintf(LCDBuffer, "%c", msg);
}
//Helpers

//initialize the USART
void usartInit(void) {
	//set Baud rate
	//UBRRH0 = baudH;
	UBRR0L = 103;

	//enable receiver - no transmit (yet) and turn on the receive complete interrupt
	UCSR0B = (1 << RXEN0) | (1 << RXCIE0);

	//set frame format
	//UCSR0C = (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00);
}

//initialize the LCD
void initLCD(){
  LCDinit();  //initialize the display
  LCDcursorOFF();
  LCDclr();        //clear the display
}
//Main

int main (void){
	usartInit();
	initLCD();
	while (1){
	LCDGotoXY(0,0);
	LCDstring(LCDBuffer, 1);
	}
	return 0;
}
