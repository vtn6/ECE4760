// Mega644 version

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd_lib.h"
#include <avr/eeprom.h> 
//#include <stdio.h>

//set up the debugging utility ASSERT
//#define __ASSERT_USE_STDERR
//#include <assert.h>

// serial communication library
//#include "uart.h"

//define the states for the game
#define INITIAL 0
#define READY 1
#define WAITING 2
#define LED_ON 3
#define DISPLAY 4
#define CHEAT 5

//define the states for the debouncer
#define PUSHED 0
#define UNKNOWN 1
#define RELEASED 2

//define constants for EEPROM memroy addresses
#define EEPROM_TRUE_ADDR 0
#define EEPROM_DATA_ADDR 1

//define timer constants
#define DEBOUNCE_TIME 30
#define RXN_MAX_TIME 1000 

//set constants for Strings to be displayed on the LCD.
//these will be stored in flash memory
const int8_t LCDHello[] PROGMEM = "Hello World\0";
const int8_t LCDReady[] PROGMEM = "Ready\0";
const int8_t LCDCheat[] PROGMEM = "CHEATER\0";
const int8_t LCDScore[] PROGMEM = "Score: \0";
const int8_t LCDHighScore[] PROGMEM = "High Score: \0";
 
//set constant for the initial value to store in EEPROM
const uint16_t DEFAULT_SCORE = 1000;

void UpdateGameState(void);	//based on the current state, and the button input, update the state of the game
void initialize(void); //all the usual mcu stuff 
void Debounce(void);	//debounce the button
void InitLCD(void);		//initialize the LCD
          

volatile uint16_t elapsedTime;	//timer to keep tracK of time
volatile char gameState;	//state variable to Know what to execute
volatile char debouncing;	//flag indicating whether or not the button is being debounced
volatile char debounceCountdown;	//used to control when the button input is read when bebouncing the signal
volatile char pressTime;	//the time when a button press occurs 
volatile char pressed;		//flag indicating whether or not the button has been pressed
volatile char pressedAndReleased;	//flag indicating that the button has been pressed and released
volatile char maybePressed;	//flag indicating that the button has been pressed, but not debounced
volatile char pushState;	//state variable to keep track of where the debounce state machine is.
volatile char waitCount;	//the current amount of time that the player has been waiting
volatile uint16_t rxnCount;	//the amount of time that has elapsed since the LED and buzzer were turned on

char led;					//light states
char readyDisplayed;		//flag indicating whether or not ready is displayed on the LCD screen
char randomTimeChosen;		//flag indicating whether or not a time between 1 and 2 seconds was chosen (at random) 
char ledTurnedOn;			//flag indicating the LED has been turned on
char scoreDisplayed;		//flag indicating the score has been displayed on the LCD screen
char cheatDisplayed;		//flag indicating the cheater messaged has been displayed on the LCD
char waitTime;				//a time between one and two seconds. Determined the amount of time before the LED and buzzer are turned on
int8_t LCDBuffer[17];	// LCD display buffer 


// UART file descriptor
// putchar and getchar are in uart.c
//FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
         
//**********************************************************
//timer 0 compare ISR
ISR (TIMER0_COMPA_vect){
	elapsedTime++;

	//Put into a separate function to maKe code more beautiful

	//ChecK to see if we are currently debouncing a signal
	//if the signal isn't being debounced, checK if the button has been pressed
	//if we are waiting for it to be pressed or checK if it is not pressed if we are waiting for
	//the release event
	if (!debouncing){
		Debounce();
	}
	else{
		if (!debounceCountdown--){
			Debounce();
		}
	}
	UpdateGameState();

}

//**********************************************************       
//Entry point and task scheduler loop
int main(void){  
 	initialize();
	if (eeprom_read_byte((uint8_t*)EEPROM_TRUE_ADDR) != 'T'){
		eeprom_write_word((uint16_t*)EEPROM_DATA_ADDR,DEFAULT_SCORE);
		eeprom_write_byte((uint8_t*)EEPROM_TRUE_ADDR,'T');
	}

  //main task scheduler loop 
  while(1){ 
  	switch (gameState){

	case INITIAL:
		PORTB = ~0x01; //led0
		break;
	
	case READY:
		PORTB = ~0x02; //led1
		if (!readyDisplayed){
			LCDclr();
			LCDGotoXY(0,0);
			CopyStringtoLCD(LCDReady, 0, 0);
			readyDisplayed = 1;
		}
		break;

	case WAITING:
		PORTB = ~0x04; //led2
		if (!randomTimeChosen){
			LCDclr();
			//assign a random time to waitTime
			waitTime = (rand() % 1000) + 1000;
			randomTimeChosen = 1;
		}
		break;

	case LED_ON:
		PORTB = ~0x08; //led3
		if (!ledTurnedOn){
			//turn the buzzer on
			ledTurnedOn = 1;
		}
		break;

	case DISPLAY:
		PORTB = ~0x10; //led4
		if (!scoreDisplayed){
			LCDGotoXY(0,0);
			CopyStringtoLCD(LCDScore, 0, 0);
			LCDGotoXY(0,0);
			CopyStringtoLCD(LCDHighScore, 0, 1);

			//Display the player's score
			//eeprom_write_word((uint16_t*)EEPROM_DATA_ADDR,rxnCount);
			sprintf(LCDBuffer, "%i", rxnCount);
			LCDGotoXY(7, 0);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
			
			//Display the high score
			uint16_t highScore = eeprom_read_word((uint16_t*)EEPROM_DATA_ADDR);
			sprintf(LCDBuffer, "%i", highScore);
			LCDGotoXY(11, 1);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
			scoreDisplayed = 1;
		}
		break;

	case CHEAT: 
		PORTB = ~0x20; //led5
		if (!cheatDisplayed){
			LCDGotoXY(0, 0);
			CopyStringtoLCD(LCDCheat, 0, 0);
			cheatDisplayed = 1;
		}
		break;
	}
  }
}  
  
//********************************************************** 
//Set it all up
void initialize(void) {
  //set up the ports
  DDRB=0xff;    // PORT B is an output  
  PORTB=0;
  DDRD=0x00;	// PORT D is an input 
           
  //set up timer 0 for 1 mSec ticks
  TIMSK0 = 2;		//turn on timer 0 cmp match ISR 
  OCR0A = 249;  	//set the compare reg to 250 time ticks
  TCCR0A = 0b00000010; // turn on clear-on-match
  TCCR0B = 0b00000011;	// clock prescalar to 64
    
  //init the LED status (all off)
  led=0xFF;
   
  
  //init the task timers
  elapsedTime = 0;
  gameState = INITIAL;
  readyDisplayed = 0;
  randomTimeChosen = 0;
  ledTurnedOn = 0;
  scoreDisplayed = 0;
  cheatDisplayed = 0;

  //initialize the LCD
  InitLCD();

  //init the UART -- uart_init() is in uart.c
  //uart_init();
  //stdout = stdin = stderr = &uart_str;
  //fprintf(stdout,"Starting...\n\r");
      
  //crank up the ISRs
  sei();
}  

//Initialize the LCD
void InitLCD(void){
	LCDinit();	//initialize the display
	LCDcursorOFF();
	LCDclr();				//clear the display
	LCDGotoXY(0,0);
	CopyStringtoLCD(LCDHello, 0, 0);
}

//Debounce the button using a debounce state machine
void Debounce(void){
	char down = ~PIND & 0x01; //Read the pin
	switch(pushState){
	
	//in the released state: stay in this state if the button is not down
	//go to UnKnown if the button is down, reset the debounce countdown.
	case RELEASED:
		if (down){
			pushState = UNKNOWN;
			debounceCountdown = DEBOUNCE_TIME;
			debouncing = 1;
			maybePressed = 1;
		}
		break;
	//in the UnKnown state: go to released if the button is not down Update
	//pressedAndReleased if the button was previously pressed.
	//go to PUSHED if the button is down
	case UNKNOWN:
		if (down){
			pushState = PUSHED;
			pressed = 1;
		}
		else{
			pushState = RELEASED;
			if (pressed){
				pressedAndReleased = 1;
				pressed = 0;
				
			}
		}
		debouncing = 0;
		maybePressed = 0;
		break;

	//in the PUSHED state go to UnKnown if the button is not down. 
	//stay in PUSHED if the button is down
	case PUSHED:
		if (!down){
			pushState = UNKNOWN;
			debounceCountdown = DEBOUNCE_TIME;
			debouncing = 1;
		}
		break;
	}
}

//Update the state of the machine based on the current state
void UpdateGameState(void){
	switch (gameState){

	//if in the initial state switch to the READY state when the button is pressed
	case INITIAL:
		if (pressedAndReleased){
			gameState = READY;
			pressedAndReleased = 0;
		}
		break;

	//if in the ready state, switch to the WAITING state when the button is pressed
	case READY:
		if (pressedAndReleased){
			gameState = WAITING;
			pressedAndReleased = 0;
			readyDisplayed = 0;
		}
		break;

	//if in the waiting tate, switch to the LED_ON state when the counter reaches the desired time
	//otherwise, if the played pressed the button, switch to the CHEAT state
	case WAITING:
		if (!waitTime--){
			gameState = LED_ON;
			randomTimeChosen = 0;
			rxnCount = 0;
		}
		else if (maybePressed || pressed){
			gameState = CHEAT;
			randomTimeChosen = 0;
			debouncing = 0;
		}
		break;

	//if in the LED_ON state, switch to the DISPLAY state when the player presses a button or 1 second elapses
	case LED_ON:
		if (!maybePressed && !pressed && !pressedAndReleased){
			rxnCount++;
		}	
		else if (pressedAndReleased){
			gameState = DISPLAY;
			pressedAndReleased = 0;
			ledTurnedOn = 0;
		}
		else if (rxnCount == RXN_MAX_TIME && !(pressed || maybePressed)){
			gameState = DISPLAY;
			ledTurnedOn = 0;
		}
		break;

	//if in the DISPLAY state, switch to the ready state when the button is pressed
	case DISPLAY:
		if (pressedAndReleased){
			gameState = READY;
			pressedAndReleased = 0;
			scoreDisplayed = 0;
			LCDclr();
		}
		break;

	case CHEAT: 
		if (cheatDisplayed && pressedAndReleased){
			gameState = WAITING;
			pressedAndReleased = 0;
			cheatDisplayed = 0;
		}
		break;
	}
}
