// Mega644 version

// Multimeter
// PORTA to autocalibration
// PORTB to Analog Input
// PORTC to LCD
// PORTD to Keypad
// PORTE to LEDs

//  A to D test code
// NOTE -- You MUST MOUNT the Aref jumper

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd_lib.h"
#include <avr/eeprom.h>
#include "uart.h"

// multiState
#define CHOOSE 0
#define VOLTAGE 1
#define FREQUENCY 2
#define RESISTANCE 3

// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

//I like these definitions
#define begin {
#define end  }

volatile uint16_t elapsedTime;  //timer to keep track of time
volatile char multiStatelalal;  //state variable to know what to execute
volatile char triggerPoll; // set to true to poll the inputs

int8_t LCDBuffer[17];  // LCD display buffer

char Ain; 		//raw A to D number
float Vref;


  
//timer 0 compare ISR
//Executes every 1ms
ISR (TIMER0_COMPA_vect) {
  elapsedTime++;
  // Every 1/5 of a second, trigger a poll
  if((elapsedTime % 200) == 0) {
    triggerPoll = 1;
  }
}


ISR (TIMER1_CAPT_vect) {
}
  
void main(void) {
  initalize();

  // measure and display loop
  while (1) {
    if(triggerPoll) {
      poll();
    }
  }
}

void initialize(void) {
	
	multiStatelalal = CHOOSE;
	triggerPoll = 0;

	Vref = 5.00;

  //init the A to D converter
  //channel zero/ left adj /EXTERNAL Aref
  //!!!CONNECT Aref jumper!!!!
  ADMUX = (1<<ADLAR);

  //enable ADC and set prescaler to 1/128*16MHz=125,000
  //and clear interupt enable
  //and start a conversion
  ADCSRA = (1<<ADEN) | (1<<ADSC) + 7;

  // Set A to input (high impedence)
  // Set E as output
  // Set D as input
  DDRA = 0xff;
  // Set to an input to send high
  PORTA = 0xff;
  DDRD = 0x00;
  DDRE = 0xff;
  PORTE = 0;
  
  
  //set up timer 0 for 1 mSec ticks
  TIMSK0 = 2;    //turn on timer 0 cmp match ISR
  OCR0A = 249;    //set the compare reg to 250 time ticks
  TCCR0A = 0b00000010; // turn on clear-on-match
  TCCR0B = 0b00000011;  // clock prescalar to 64
  
  //set up timer 1 to interrupt on capture
  TIMSK1 = (1 << ICIE1) //turn on timer1 input capture ISR
  TCCR1A = 0b00000010;
  //TCCR1B = (1 << ICNC1) | (1 << ICES1) | 0b00000101; // Start 
  
  
  InitLCD();

  // init the UART -- uart_init() is in uart.c
  uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout,"Starting ADC demo...\n\r");
  

  sei();
}

//Initialize the LCD
const int8_t LCDHello[] PROGMEM = "MultiMaster\0"; // Welcome Message
const int8_t LCDHello2[] PROGMEM = " v9001\0";
void InitLCD(void) {
  LCDinit();  //initialize the display
  LCDcursorOFF();
  LCDclr();        //clear the display
  LCDGotoXY(0,0);
  CopyStringtoLCD(LCDHello, 0, 0);
  CopyStringtoLCD(LCDHello2, 0, 1);
}

void poll(void) {
  //get the sample
  Ain = ADCH;
  //start another conversion
  ADCSRA |= (1<<ADSC);
  //results to hyperterm
  printf("%d\n\r",Ain);
  //results to LCD
  LCDclr();
  sprintf(LCDBuffer, "%d", Ain);
  LCDGotoXY(0, 0);
  LCDString(LCDBuffer, strlen(LCDBuffer));
  //fancy stuff
  //voltage = (float)Ain;
  //voltage = (voltage/1024.0)*Vref;
  //dtostrf(voltage, 6, 3, v_string);
  //("%s",v_string);
  //finished polling
  triggerPoll = 0;
}

// getKeyState and switch based on it
void setMode(void) {
  switch(multiState) {
    case CHOOSE:
      break;
    case VOLTAGE:
      DDRA = 0xff;
      // Reference = Vcc
      // ADMUX = (1<<ADLAR);
      // Reference = 2.56
      // ADMUX = (1<<ADLAR) | (3 << REFS0);
      // Reference = 1.1
      // ADMUX = (1<<ADLAR) | (2 << REFS0);
      break;
    case FREQUENCY:
      ADMUX = (1 << ADLAR);
      DDRA = 0xff;
      break;
    case RESISTANCE:
      ADMUX = (1 << ADLAR);
      // Use 1k resistor
      // DDRA = ~(1 << 5);
      // Use 10k resistor
      // DDRA = ~(1 << 6);
      // Use 100k resistor
      // DDRA = ~(1 << 7);
      break;
  }
}
