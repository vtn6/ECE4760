// Mega644 version

// ReactionMaster v9001
// How to connect stuff:
// Jumper from PORTB to LEDs
// Jumper from PORTC to LCD
// Jumper from PORTD to Switches
// Connect speaker to PORTA[0]
// 16MHz clock

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

#define and &&

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

#define RND_MAX 2000
#define RND_MIN 1000

//set constants for Strings to be displayed on the LCD.
//these will be stored in flash memory
const int8_t LCDHello[] PROGMEM = "ReactionMaster\0"; // Welcome Message
const int8_t LCDHello2[] PROGMEM = " v9001\0";
const int8_t LCDReady[] PROGMEM = "Ready\0";
const int8_t LCDCheat[] PROGMEM = "CHEATER\0";
const int8_t LCDScore[] PROGMEM = "Score: \0";
const int8_t LCDTooSlow[] PROGMEM = "Too Slow!\0";
const int8_t LCDHighScore[] PROGMEM = "High Score: \0";

//set constant for the initial value to store in EEPROM
const uint16_t DEFAULT_SCORE = 1000;

void UpdateGameState(void);  //update the state of the game
void initialize(void); //all the usual mcu stuff
void Debounce(void);  //debounce the button
void InitLCD(void);    //initialize the LCD


volatile uint16_t elapsedTime;  //timer to keep tracK of time
volatile char gameState;  //state variable to Know what to execute
volatile char debouncing;  //flag indicating whether or not the button is being debounced
volatile char debounceCountdown;  //used to control when the button input is read when bebouncing the signal
volatile char pressTime;  //the time when a button press occurs
volatile char pressed;    //flag indicating whether or not the button has been pressed
volatile char pressedAndReleased;  //flag indicating that the button has been pressed and released
volatile char maybePressed;  //flag indicating that the button has been pressed, but not debounced
volatile char pushState;  //state variable to keep track of where the debounce state machine is.
volatile uint16_t rxnCount;  //the amount of time that has elapsed since the LED and buzzer were turned on

volatile char buzzer;               // flag indicating whether the buzzer should be on or not
volatile char readyDisplayed;    //flag indicating whether or not ready is displayed on the LCD screen
volatile char randomTimeChosen;    //flag indicating whether or not a time between 1 and 2 seconds was chosen (at random)
volatile char ledTurnedOn;      //flag indicating the LED has been turned on
volatile char scoreDisplayed;    //flag indicating the score has been displayed on the LCD screen
volatile char cheatDisplayed;    //flag indicating the cheater messaged has been displayed on the LCD
volatile char cheatState;      //flag indicating the player is cheating
volatile uint16_t waitTime;        //a time between one and two seconds. Determined the amount of time before the LED and buzzer are turned on
int8_t LCDBuffer[17];  // LCD display buffer


// UART file descriptor
// putchar and getchar are in uart.c
//FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

//**********************************************************
//timer 0 compare ISR
//Executes every 1ms
ISR (TIMER0_COMPA_vect){

  elapsedTime++;

  //Check to see if we are currently debouncing a signal
  //if the signal isn't being debounced, checK if the button has been pressed
  //if we are waiting for it to be pressed or checK if it is not pressed if we are waiting for
  //the release event
  if (!debouncing){
    Debounce();
  }
  else if (!debounceCountdown--){
    Debounce();
  }
  if (buzzer){
    // Toggle buzzer at 500Hz
    PORTA ^= 0x01;
  }

  UpdateGameState();
}

//**********************************************************
//Entry point and task scheduler loop
int main(void){
   initialize();
  // Check if R (for ReactionMaster) is written. Write default score if not
  if (eeprom_read_byte((uint8_t*)EEPROM_TRUE_ADDR) != 'R'){
    eeprom_write_word((uint16_t*)EEPROM_DATA_ADDR,DEFAULT_SCORE);
    eeprom_write_byte((uint8_t*)EEPROM_TRUE_ADDR,'R');
  }

  // main task scheduler loop
  // set LEDs corresponding to state
  // LED7 is used as reaction timer LED
  while(1){
    switch (gameState){
      case INITIAL:
        PORTB = ~0x01; //led0
        break;

      case READY:
        if (!readyDisplayed){
          pressedAndReleased = 0;
          scoreDisplayed = 0;
          PORTB = ~0x02; //led1
          LCDclr();
          LCDGotoXY(0,0);
          CopyStringtoLCD(LCDReady, 0, 0);
          readyDisplayed = 1;
        }
        break;

      case WAITING:
        if (!randomTimeChosen){
          pressedAndReleased = 0;
          readyDisplayed = 0;
          cheatDisplayed = 0;
          cheatState = 0;
          PORTB = ~0x04; //led2
          LCDclr();
          //assign a random time to waitTime
          waitTime = rand() % (RND_MAX - RND_MIN + 1) + 1000;
          randomTimeChosen = 1;
        }
        break;

      case LED_ON:
        if (!ledTurnedOn){
          randomTimeChosen = 0;
          rxnCount = 0;
          PORTB = ~(0x08 | 0x80); //led7 and led3
          // PORTB = ~0x80; // led7
          // turn the buzzer on
          buzzer = 1;
          ledTurnedOn = 1;
        }
        break;

      case DISPLAY:
        if (!scoreDisplayed){
          PORTB = ~0x10; //led4
          // PORTB = ~0x00; // All off

          buzzer = 0;
          pressedAndReleased = 0;
          ledTurnedOn = 0;

          if(rxnCount == 1000) {
            CopyStringtoLCD(LCDTooSlow, 0, 0);
          } else {
            CopyStringtoLCD(LCDScore, 0, 0);
            //Display the player's score
            //eeprom_write_word((uint16_t*)EEPROM_DATA_ADDR,rxnCount);
            sprintf(LCDBuffer, "%i", rxnCount);
            LCDGotoXY(7, 0);
            LCDstring(LCDBuffer, strlen(LCDBuffer));
          }
          CopyStringtoLCD(LCDHighScore, 0, 1);
          //Display the high score
          uint16_t highScore = eeprom_read_word((uint16_t*)EEPROM_DATA_ADDR);
          sprintf(LCDBuffer, "%i", highScore);
          LCDGotoXY(11, 1);
          LCDstring(LCDBuffer, strlen(LCDBuffer));
          scoreDisplayed = 1;

          //Store the player's score if it is larger than the current high score
          if (rxnCount < highScore){
            eeprom_write_word((uint16_t*)EEPROM_DATA_ADDR,rxnCount);
          }

        }
        break;

      case CHEAT:
        if (!cheatDisplayed){
          PORTB = ~0x20; //led5
          // PORTB = ~0x00; // All off
          randomTimeChosen = 0;
          cheatState = 0;
          buzzer = 0;
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
  // PortA: Buzzer, output
  DDRA=0xff;
  PORTA=0;
  // PortB: LEDs, output
  DDRB=0xff;
  PORTB=0;
  // PortD: Switches, input
  DDRD=0x00;

  //set up timer 0 for 1 mSec ticks
  TIMSK0 = 2;    //turn on timer 0 cmp match ISR
  OCR0A = 249;    //set the compare reg to 250 time ticks
  TCCR0A = 0b00000010; // turn on clear-on-match
  TCCR0B = 0b00000011;  // clock prescalar to 64

  //init the LED status (all off)
  buzzer = 0;

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
  LCDinit();  //initialize the display
  LCDcursorOFF();
  LCDclr();        //clear the display
  LCDGotoXY(0,0);
  CopyStringtoLCD(LCDHello, 0, 0);
  CopyStringtoLCD(LCDHello2, 0, 1);
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
      }
      break;

    //if in the ready state, switch to the WAITING state when the button is pressed
    case READY:
      if (readyDisplayed && pressedAndReleased){
        gameState = WAITING;
      }
      break;

    //if in the waiting tate, switch to the LED_ON state when the counter reaches the desired time
    //otherwise, if the played pressed the button, switch to the CHEAT state
    case WAITING:
      if (randomTimeChosen){
        if (!waitTime--){
          gameState = LED_ON;
        }
        else if (maybePressed || pressed){
          gameState = CHEAT;
        }
      }
      break;

    //if in the LED_ON state, switch to the DISPLAY state when the player presses a button or 1 second elapses
    case LED_ON:
      if (!maybePressed && !pressed && !pressedAndReleased){
        rxnCount++;
      }
      if (pressedAndReleased || ((rxnCount == RXN_MAX_TIME) && !(pressed || maybePressed))){
        gameState = DISPLAY;
      }
      break;

    //if in the DISPLAY state, switch to the ready state when the button is pressed
    case DISPLAY:
      if (scoreDisplayed && pressedAndReleased){
        gameState = READY;
      }
      break;

    case CHEAT:
      if (pressedAndReleased && !cheatState){
        cheatState = 1;
        pressedAndReleased = 0;
      }
      else if (cheatDisplayed && pressedAndReleased && cheatState){
        gameState = READY;
      }
      break;
  }
}
