// DDS output thru PWM on timer0 OC0A (pin B.3)
// Mega644 version
// FM synthesis

// Controls:
// A To switch Mode
// # For help
// D to Enter
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "lcd_lib.h"
#include <inttypes.h>
#include <math.h> 		// for sine
#include "lcd_lib.h"
#include "keypad.h"
//#include "uart.h"
// set up serial for debugging
//FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

//I like these definitions
#define and &&
#define or ||
//because python is awesome-ly

//LCD Strings
const uint8_t LCDHelloTop[] PROGMEM = "SYNTH v9001\0"; // Welcome Message
const uint8_t LCDHelloBot[] PROGMEM = "PRESS # FOR HELP\0";
const uint8_t LCDSequenceId[] PROGMEM = "SEQUENCE ID \0"; 
const uint8_t LCDMainFrequency[] PROGMEM = "MAIN FREQ \0";
const uint8_t LCDMainDecay[] PROGMEM = "MAIN DECAY \0";
const uint8_t LCDMainRise[] PROGMEM = "MAIN RISE \0";
const uint8_t LCDFMFreq[] PROGMEM = "FM FREQ \0";
const uint8_t LCDFMDepth[] PROGMEM = "FM DEPTH \0";
const uint8_t LCDFMDecay[] PROGMEM = "FM DECAY \0";
const uint8_t LCDVoice[] PROGMEM = "VOICE \0";

//LCD String lengths
int8_t seqStrLen;
int8_t mainFreqStrLen;
int8_t mainDecayStrLen;
int8_t mainRiseStrLen;
int8_t fmFreqStrLen;
int8_t fmDepthStrLen;
int8_t fmDecayStrLen;
int8_t voiceStrLen;

// The DDS variables 
volatile unsigned int acc_main, acc_fm1 ;
volatile unsigned char high_main, high_fm1, decay_fm1, decay_main, depth_fm1, rise_main ;
volatile unsigned int inc_main, inc_fm1, amp_main, amp_fm1 ;
volatile unsigned int rise_phase_main, amp_rise_main, amp_fall_main ;
/*
//Voice 1
volatile unsigned int voice1_acc_main, voice1_acc_fm1 ;
volatile unsigned char voice1_high_main, voice1_high_fm1, voice1_decay_fm1, voice1_decay_main, voice1_depth_fm1, voice1_rise_main ;
volatile unsigned int voice1_inc_main, voice1_inc_fm1, voice1_amp_main, voice1_amp_fm1 ;
volatile unsigned int voice1_rise_phase_main, voice1_amp_rise_main, voice1_amp_fall_main ;

//Voice 2
volatile unsigned int voice2_acc_main, voice2_acc_fm1 ;
volatile unsigned char voice2_high_main, voice2_high_fm1, voice2_decay_fm1, voice2_decay_main, voice2_depth_fm1, voice2_rise_main ;
volatile unsigned int voice2_inc_main, voice2_inc_fm1, voice2_amp_main, voice2_amp_fm1 ;
volatile unsigned int voice2_rise_phase_main, voice2_amp_rise_main, voice2_amp_fall_main ;

//Voice 3
volatile unsigned int voice3_acc_main, voice3_acc_fm1 ;
volatile unsigned char voice3_high_main, voice3_high_fm1, voice3_decay_fm1, voice3_decay_main, voice3_depth_fm1, voice3_rise_main ;
volatile unsigned int voice3_inc_main, voice3_inc_fm1, voice3_amp_main, voice3_amp_fm1 ;
volatile unsigned int voice3_rise_phase_main, voice3_amp_rise_main, voice3_amp_fall_main ;

//Voice 4
volatile unsigned int voice4_acc_main, voice4_acc_fm1 ;
volatile unsigned char voice4_high_main, voice4_high_fm1, voice4_decay_fm1, voice4_decay_main, voice4_depth_fm1, voice4_rise_main ;
volatile unsigned int voice4_inc_main, voice4_inc_fm1, voice4_amp_main, voice4_amp_fm1 ;
volatile unsigned int voice4_rise_phase_main, voice4_amp_rise_main, voice4_amp_fall_main ;

//Voice 5
volatile unsigned int voice5_acc_main, voice5_acc_fm1 ;
volatile unsigned char voice5_high_main, voice5_high_fm1, voice5_decay_fm1, voice5_decay_main, voice5_depth_fm1, voice5_rise_main ;
volatile unsigned int voice5_inc_main, voice5_inc_fm1, voice5_amp_main, voice5_amp_fm1 ;
volatile unsigned int voice5_rise_phase_main, voice5_amp_rise_main, voice5_amp_fall_main ;

//Voice 6
volatile unsigned int voice6_acc_main, voice6_acc_fm1 ;
volatile unsigned char voice6_high_main, voice6_high_fm1, voice6_decay_fm1, voice6_decay_main, voice6_depth_fm1, voice6_rise_main ;
volatile unsigned int voice6_inc_main, voice6_inc_fm1, voice6_amp_main, voice6_amp_fm1 ;
volatile unsigned int voice6_rise_phase_main, voice6_amp_rise_main, voice6_amp_fall_main ;
*/

volatile uint8_t maxDuration;
volatile uint8_t voice;
#define max_amp 32767
// tables for DDS			
signed char sineTable[256], fm1 ;

//MarKov Matrices
#define NUM_NOTES 8
uint8_t curNote;
const uint16_t markovFrequencies[8] = {262, 294, 330, 392, 440, 523, 587, 659};
 
const uint8_t ascendingMarkov[64] = {0, 1, 0, 0, 0, 0, 0, 0,
									 0, 0, 1, 0, 0, 0, 0, 0,
									 0, 0, 0, 1, 0, 0, 0, 0,
									 0, 0, 0, 0, 1, 0, 0, 0,
									 0, 0, 0, 0, 0, 1, 0, 0,
									 0, 0, 0, 0, 0, 0, 1, 0,
									 1, 0, 0, 0, 0, 0, 0, 0};
									 
const uint8_t uniformMarkov[64] = {125, 125, 125, 125, 125, 125, 125, 125,
								   125, 125, 125, 125, 125, 125, 125, 125,
								   125, 125, 125, 125, 125, 125, 125, 125,
								   125, 125, 125, 125, 125, 125, 125, 125,
								   125, 125, 125, 125, 125, 125, 125, 125,
								   125, 125, 125, 125, 125, 125, 125, 125,
								   125, 125, 125, 125, 125, 125, 125, 125,
								   125, 125, 125, 125, 125, 125, 125, 125};
const uint8_t chaoticMarkov[64] = {55, 9, 9, 56, 26, 30, 27, 43,
								   43, 46, 54, 21, 11, 40, 2, 38,
								   19, 22, 9, 20, 64, 34, 47, 40,
								   44, 39, 22, 16, 12, 40, 49, 33,
								   22, 63, 20, 5, 34, 39, 57, 15,
								   48, 29, 33, 47, 11, 23, 52, 12,
								   23, 36, 52, 15, 21, 54, 26, 28,  
								   28, 32, 16, 27, 61, 50, 38, 3};
	
const uint8_t descendingMarkov[64] = {0, 0, 0, 0, 0, 0, 0, 1,
									  1, 0, 0, 0, 0, 0, 0, 0,
									  0, 1, 0, 0, 0, 0, 0, 0,
									  0, 0, 1, 0, 0, 0, 0, 0,
									  0, 0, 0, 1, 0, 0, 0, 0,
									  0, 0, 0, 0, 1, 0, 0, 0,
									  0, 0, 0, 0, 0, 1, 0, 0,
									  0, 0, 0, 0, 0, 0, 1, 0}; 

//the sequencer id
volatile uint8_t seqId;

//state variables
volatile uint8_t state;
volatile uint8_t voice;
#define  INIT 0
#define  MAIN_SCREEN 1
#define  MAN 2
#define  SET_SEQUENCE 3
#define  SET_INC_MAIN 4
#define  SET_DECAY_MAIN 5
#define  SET_RISE_MAIN 6
#define  SET_INC_FM 7
#define  SET_DEPTH_FM 8
#define  SET_DECAY_FM 9

//voice IDs
#define VOICE_1 0
#define VOICE_2 1
#define VOICE_3 2
#define VOICE_4 3
#define VOICE_5 4
#define VOICE_6 5

//constants for random number generation
const uint8_t RND_MAX = 255;
const uint8_t RND_MIN = 0;
// trigger
volatile char pluck, pushed ;

// Time variables
// the volitile is needed because the time is only set in the ISR
// time counts mSec, sample counts DDS samples (62.5 KHz)
volatile unsigned int time ;
volatile char  count;

// index for sine table build
unsigned int i;

//string to write to the LCD 
uint8_t LCDBuffer[17];  // LCD display buffer

//States for the Synthesizulatrix
const uint8_t SET_ACC_MAIN;

//function signatures
void Initialize(void);
void initLCD(void);
void updateLCD();
void updateManual(void);
void setState(uint8_t);
void nextState(void);


//returns OCR0A
uint8_t sample(void) {
	// compute exponential attack and decay of amplitude
	// the (time & 0x0ff) slows down the decay computation by 256 times		
	if (!(time & 0x0ff)) {
		amp_fall_main = amp_fall_main - (amp_fall_main>>decay_main) ;
		rise_phase_main = rise_phase_main - (rise_phase_main>>rise_main);
		// compute exponential decay of FM depth of modulation
		amp_fm1 = amp_fm1 - (amp_fm1>>decay_fm1) ;
	}

	// form (1-exp(-t/tau)) for the attack phase
	amp_rise_main =  max_amp - rise_phase_main;
	// product of rise and fall exponentials is the amplitude envelope
	amp_main = (amp_rise_main>>8) * (amp_fall_main>>8) ;

	// Init the synth
	if (pluck==1) {
		amp_fall_main = max_amp; 
		rise_phase_main = max_amp ;
		amp_rise_main = 0 ;
		amp_fm1 = max_amp ;
		// phase lock the synth
		acc_fm1 = 0 ;
		acc_main = 0;
		pluck = 0;
	}

	//the FM DDR -- feeds into final DDR
	acc_fm1 = acc_fm1 + inc_fm1 ;
	high_fm1 = (char)(acc_fm1 >> 8) ;
	fm1 = sineTable[high_fm1] ;

	//the final output DDR 
	// phase accum = main_DDR_freq + FM_DDR * (FM amplitude)
	acc_main = acc_main + (inc_main + (fm1*(amp_fm1>>depth_fm1))) ;
	high_main = (char)(acc_main >> 8) ;
	
	// output the wavefrom sample
	// scale amplitude to use only high byte and shift into range
	// 0 to 255
	return 128 + (((amp_main>>8) * (int)sineTable[high_main])>>7) ;
}

ISR (TIMER1_COMPA_vect) // Fs = 8000
{ 
	// turn on timer for profiling
	//TCNT2 = 0; TCCR2B = 1;

	// Set Sample
	OCR0A = sample();
	
	time++;     //ticks at 8 KHz 
	// profiling 
	//TCCR2B = 0;
} 

// Every 1ms
ISR (TIMER2_COMPA_vect){
	KeypadDebounce();
}
 
/////////////////////////////////////////////////////
//Initialization code
void Initialize(void){
	// make B.3 an output
	DDRB = (1<<PINB3) ;

	//Keypad
	DDRD=0x00;

	//init the UART -- uart_init() is in uart.c
	//uart_init();
	//stdout = stdin = stderr = &uart_str;
	//fprintf(stdout,"Starting...\n\r");

	// init the sine table
	for (i=0; i<256; i++)
	{
		sineTable[i] = (char)(127.0 * sin(6.283*((float)i)/256.0)) ;
	}  

	// init the time counter
	time=0;

	// timer 0 runs at full rate
	TCCR0B = 1 ;  
	//turn off timer 0 overflow ISR
	TIMSK0 = 0 ;
	// turn on PWM
	// turn on fast PWM and OC0A output
	// at full clock rate, toggle OC0A (pin B3) 
	// 16 microsec per PWM cycle sample time
	TCCR0A = (1<<COM0A0) | (1<<COM0A1) | (1<<WGM00) | (1<<WGM01) ; 
	OCR0A = 128 ; // set PWM to half full scale

	// timer 1 ticks at 8000 Hz or 125 microsecs period=2000 ticks
	OCR1A = 1999 ; // 2000 ticks
	TIMSK1 = (1<<OCIE1A) ;
	TCCR1B = 0x09; 	//full speed; clear-on-match
	TCCR1A = 0x00;	//turn off pwm and oc lines

	//set up timer 2 for 1 mSec ticks
	TIMSK2 = 2;		//turn on timer 2 cmp match ISR
	OCR2A = 249;	//set the compare reg to 250 time ticks
	TCCR2A = 0b00000010; // turn on clear-on-match
	TCCR2B = 0b00000011;	// clock prescalar to 64


	seqStrLen = strlen(LCDSequenceId);
	mainFreqStrLen = strlen(LCDMainFrequency);
	mainDecayStrLen = strlen(LCDMainDecay);
	mainRiseStrLen = strlen(LCDMainRise);
	fmFreqStrLen = strlen(LCDFMFreq);
	fmDepthStrLen = strlen(LCDFMDepth);
	fmDecayStrLen = strlen(LCDFMDecay);
	voiceStrLen = strlen(LCDVoice);

	initLCD();

	// turn on all ISRs
	sei() ;
   

	///////////////////////////////////////////////////
	// Sound parameters
	///////////////////////////////////////////////////
	// Base frequency
	// 2^16/8000*freq = 8.192*freq
	inc_main = (int)(8.192 * 261) ; 
	// rise and decay SHIFT factor  -- bigger is slower
	// 6 implies tau of 64 cycles
	// 8 implies tau of 256 cycles
	// max value is 8
	decay_main = 4 ;
	rise_main = 0 ;
	//
	// FM modulation rate -- also a frequency
	inc_fm1 = (int)(8.192 * 65) ;
	// FM modulation depth SHIFT factor 
	// bigger factor implies smaller FM!
	// useful range is 4 to 15
	depth_fm1 = 7 ;
	// decay SHIFT factor -- bigger is slower
	// 6 implies tau of 64 cycles
	// 8 implies tau of 256 cycles
	// max value is 8
	decay_fm1 = 6 ;
}
  ////////////////////////////////////////////////////

//Initialize the LCD
void initLCD(void){
	LCDinit();	//initialize the display
	LCDcursorOFF();
	LCDclr();				//clear the display
	LCDGotoXY(0,0);
	CopyStringtoLCD(LCDHelloTop, 0, 0);
	CopyStringtoLCD(LCDHelloBot, 0, 1);
}
/////////////////////////////////////////////////////

///////////////////////////////////////////////////// 
//Update the LCD
void updateLCD(void){
	return;
	LCDclr();
	switch (state) {
	 	case MAIN_SCREEN:
			CopyStringtoLCD(LCDHelloTop, 0, 0);
			CopyStringtoLCD(LCDHelloBot, 0, 1);
			break;
		case MAN:
			updateManual();
			break;
		case SET_SEQUENCE:
			CopyStringtoLCD(LCDSequenceId, 0, 1);
			LCDGotoXY(seqStrLen, 1);
			sprintf(LCDBuffer, "%d", seqId);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
			break;
		case SET_INC_MAIN:
			CopyStringtoLCD(LCDMainFrequency, 0, 1);
			LCDGotoXY(mainFreqStrLen, 1);
			sprintf(LCDBuffer, "%d", inc_main);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
			break;
		case SET_DECAY_MAIN:
			CopyStringtoLCD(LCDMainDecay, 0, 1);
			LCDGotoXY(mainDecayStrLen, 1);
			sprintf(LCDBuffer, "%d", decay_main);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
			break;
		case SET_RISE_MAIN:
			CopyStringtoLCD(LCDMainRise, 0, 1);
			LCDGotoXY(mainRiseStrLen, 1);
			sprintf(LCDBuffer, "%d", rise_main);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
			break;
		case SET_INC_FM:
			CopyStringtoLCD(LCDFMFreq, 0, 1);
			LCDGotoXY(fmFreqStrLen, 1);
			sprintf(LCDBuffer, "%d", inc_fm1);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
			break;
		case SET_DEPTH_FM:
			CopyStringtoLCD(LCDFMDepth, 0, 1);
			LCDGotoXY(fmDepthStrLen, 1);
			sprintf(LCDBuffer, "%d", depth_fm1);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
			break;
		case SET_DECAY_FM:
			CopyStringtoLCD(LCDFMDecay, 0, 1);
			LCDGotoXY(fmDecayStrLen, 1);
			sprintf(LCDBuffer, "%d", decay_fm1);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
			break;
	}

	CopyStringtoLCD(LCDVoice, 0, 0);
	LCDGotoXY(voiceStrLen, 0);
	sprintf(LCDBuffer, "%d", voice);
	LCDstring(LCDBuffer, 1);
}

void updateManual(void){

}

void setState(uint8_t s) {
	state = s;
	updateLCD();
}

//set the next note to play
void setNextNote(){
	switch (seqId){
		case 1:
			inc_main = markovFrequencies[curNote++ % NUM_NOTES];
			break;
	}
	//for some reason, doesn't like rand()
	//uint8_t nextNote = (rand() % (RND_MAX - RND_MIN + 1)) + RND_MIN;
}

// update to next state if key is pressed
uint8_t waitingForInput = 0;
void nextState(void){
	if(waitingForInput) {
		// output input to screen
	}
	uint8_t key = KeypadKey();
	if(key != 0) {
		//_delay_ms(1000);
		sprintf(LCDBuffer, "%d", key);
		//LCDGotoXY(0, 0);
		LCDstring(LCDBuffer, strlen(LCDBuffer));
	}
	switch(key) {
		case KEY_P:
			setState(MAN);
			break;
		case KEY_A:
			setState((state + 1) % 10);
			break;
	}
	switch (state) {
		case SET_SEQUENCE:
			if(key == KEY_D) {
				seqId = KeypadInt();
				waitingForInput = 0;
			} else {
				setState(MAIN_SCREEN);
				waitingForInput = 1;
			}
			break;
		case SET_INC_MAIN:
			if(key == KEY_D) {
				inc_main = KeypadInt();
				waitingForInput = 0;
			} else {
				setState(MAIN_SCREEN);
				waitingForInput = 1;
			}
			break;
		case SET_DECAY_MAIN:
			if(key == KEY_D) {
				decay_main = KeypadInt();
				waitingForInput = 0;
			} else {
				setState(MAIN_SCREEN);
				waitingForInput = 1;
			}
			break;
		case SET_RISE_MAIN:
			if(key == KEY_D) {
				rise_main = KeypadInt();
				waitingForInput = 0;
			} else {
				setState(MAIN_SCREEN);
				waitingForInput = 1;
			}
			break;
		case SET_INC_FM:
			if(key == KEY_D) {
				inc_fm1 = KeypadInt();
				waitingForInput = 0;
			} else {
				setState(MAIN_SCREEN);
				waitingForInput = 1;
			}
			break;
		case SET_DEPTH_FM:
			if(key == KEY_D) {
				depth_fm1 = KeypadInt();
				waitingForInput = 0;
			} else {
				setState(MAIN_SCREEN);
				waitingForInput = 1;
			}
			break;
		case SET_DECAY_FM:
			if(key == KEY_D) {
				decay_fm1 = KeypadInt();
				waitingForInput = 0;
			} else {
				setState(MAIN_SCREEN);
				waitingForInput = 1;
			}
			break;
		
	}
}
/////////////////////////////////////////////////////
int main(void)
{ 
   Initialize();
   while(1) {  
		// Check pushbutton to pluck string
		// and oneshot it
		//  
		/*
		if (!(time & 0xff)) {
			if ((~PINC & 0x01) && !pushed) {
				 pluck = 1;
				 pushed = 1;
 			}
			if (!(~PINC & 0x01)  && pushed) {
				pushed = 0;
			}
		//	printf("%d\n\r", TCNT2);
		}
		*/
		nextState();

   } // while(1)

}  //} main
////////////////////////////////////////////////////////
/*
Examples:      

Chime:
   inc_main = (int)(8.192 * 261.0) ; 
   decay_main = 5 ;
   rise_main = 1 ;
   inc_fm1 = (int)(8.192 * 350.0) ;
   depth_fm1 = 9 ;
   decay_fm1 = 5 ;

Plucked String:
	inc_main = (int)(8.192 * 500.0) ; 
   decay_main = 3 ;
   rise_main = 1 ;
   inc_fm1 = (int)(8.192 * 750.0) ;
   depth_fm1 = 8 ;
   decay_fm1 = 3 ;

Plucked String:
   inc_main = (int)(8.192 * 600) ; 
   decay_main = 5 ;
   rise_main = 0 ;
   inc_fm1 = (int)(8.192 * 150) ;
   depth_fm1 = 8 ;
   decay_fm1 = 6 ;

Bowed string
   inc_main = (int)(8.192 * 300) ;  
   decay_main = 5 ;
   rise_main = 4 ;
   inc_fm1 = (int)(8.192 * 300) ;
   depth_fm1 = 8 ;
   decay_fm1 = 6 ;

Small, stiff rod
 	inc_main = (int)(8.192 * 1440) ;   
   decay_main = 3 ;
   rise_main = 1 ;   
   inc_fm1 = (int)(8.192 * 50) ; // at 100 get stiff string; at 200 get hollow pipe
   depth_fm1 = 10 ; //or 9
   decay_fm1 = 5 ;

Bell/chime
   inc_main = (int)(8.192 * 1440) ; 
   decay_main = 5 ;
   rise_main = 1 ;
   inc_fm1 = (int)(8.192 * 600) ;
   depth_fm1 = 8 ;
   decay_fm1 = 6 ;

Bell
   inc_main = (int)(8.192 * 300) ; 
   decay_main = 5 ;
   rise_main = 0 ;
   inc_fm1 = (int)(8.192 * 1000) ;
   depth_fm1 = 8 ;
   decay_fm1 = 6 ;
*/
