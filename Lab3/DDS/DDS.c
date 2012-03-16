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
const uint8_t LCDVoiceId[] PROGMEM = "VOICE ID \0"; 
const uint8_t LCDMainFrequency[] PROGMEM = "MAIN FREQ \0";
const uint8_t LCDMainDecay[] PROGMEM = "MAIN DECAY \0";
const uint8_t LCDMainRise[] PROGMEM = "MAIN RISE \0";
const uint8_t LCDFMFreq[] PROGMEM = "FM FREQ \0";
const uint8_t LCDFMDepth[] PROGMEM = "FM DEPTH \0";
const uint8_t LCDFMDecay[] PROGMEM = "FM DECAY \0";

//LCD String lengths
int8_t seqStrLen;
int8_t voiceStrLen;
int8_t mainFreqStrLen;
int8_t mainDecayStrLen;
int8_t mainRiseStrLen;
int8_t fmFreqStrLen;
int8_t fmDepthStrLen;
int8_t fmDecayStrLen;

//#define NUM_VOICES 6
#define NUM_VOICES 4

// The DDS variables 
volatile unsigned int acc_main[NUM_VOICES], acc_fm1[NUM_VOICES] ;
volatile unsigned char high_main, high_fm1, decay_fm1[NUM_VOICES], decay_main[NUM_VOICES], depth_fm1[NUM_VOICES], rise_main[NUM_VOICES];
volatile unsigned int inc_main[NUM_VOICES], inc_fm1[NUM_VOICES], amp_main[NUM_VOICES], amp_fm1[NUM_VOICES];
volatile unsigned int rise_phase_main[NUM_VOICES], amp_rise_main[NUM_VOICES], amp_fall_main[NUM_VOICES];
volatile uint8_t sampling = 1; //set to 1 to start sampling


volatile uint8_t maxDuration;
#define max_amp 32767
// tables for DDS			
signed char sineTable[256], fm1 ;

//MarKov Matrices
#define NUM_NOTES 8
uint8_t curNote;
const uint16_t markovFrequencies[8] = {262, 294, 330, 392, 440, 523, 587, 659};
 
const uint8_t ascendingMarkov[64] = {0, 255, 0, 0, 0, 0, 0, 0,
									 0, 0, 255, 0, 0, 0, 0, 0,
									 0, 0, 0, 255, 0, 0, 0, 0,
									 0, 0, 0, 0, 255, 0, 0, 0,
									 0, 0, 0, 0, 0, 255, 0, 0,
									 0, 0, 0, 0, 0, 0, 255, 0,
									 255, 0, 0, 0, 0, 0, 0, 0};
									 
const uint8_t uniformMarkov[64] = {31, 63, 95, 127, 159, 191, 223, 255,
								   31, 63, 95, 127, 159, 191, 223, 255,
								   31, 63, 95, 127, 159, 191, 223, 255,
								   31, 63, 95, 127, 159, 191, 223, 255,
								   31, 63, 95, 127, 159, 191, 223, 255,
								   31, 63, 95, 127, 159, 191, 223, 255,
								   31, 63, 95, 127, 159, 191, 223, 255,
								   31, 63, 95, 127, 159, 191, 223, 255};
/*
 40, 30, 14, 29,  3, 50, 61, 28,
 45, 28,  9, 42, 47, 44, 27, 13,
 28, 20, 38, 40, 13, 24, 48, 43,
 61, 47,  6, 32, 27,  1, 24, 58,
 15, 42, 35, 23, 46, 22, 51, 22,
  8, 58, 40, 11, 27, 32, 30, 50,
  6, 50, 46, 32, 40, 15, 43, 24,
  4, 63, 31, 17, 25, 13, 49, 54
Cumulative is below
*/
const uint8_t chaoticMarkov[64] = {	 40, 70, 84,113,116,166,227,255,
									 45, 73, 82,124,171,215,242,255,
									 28, 48, 86,126,139,163,211,255,
									 61,108,114,146,173,174,198,255,
									 15, 57, 92,115,161,183,234,255,
									  8, 66,106,117,144,176,206,255,
									  6, 56,102,134,174,189,232,255,
									  4, 67, 98,115,140,153,202,255  };

	
const uint8_t descendingMarkov[64] = {0, 0, 0, 0, 0, 0, 0, 255,
									  255, 0, 0, 0, 0, 0, 0, 0,
									  0, 255, 0, 0, 0, 0, 0, 0,
									  0, 0, 255, 0, 0, 0, 0, 0,
									  0, 0, 0, 255, 0, 0, 0, 0,
									  0, 0, 0, 0, 255, 0, 0, 0,
									  0, 0, 0, 0, 0, 255, 0, 0,
									  0, 0, 0, 0, 0, 0, 255, 0}; 

volatile uint8_t vidx = 0;

//the sequencer id
volatile uint8_t seqId = 0;

//state variables
volatile uint8_t state;
#define  MAIN_SCREEN 0
#define  SET_VIDX 1
#define  SET_SEQUENCE 2
#define  SET_INC_MAIN 3
#define  SET_DECAY_MAIN 4
#define  SET_RISE_MAIN 5
#define  SET_INC_FM 6
#define  SET_DEPTH_FM 7
#define  SET_DECAY_FM 8

//voice IDs
#define VOICE_1 0
#define VOICE_2 1
#define VOICE_3 2
#define VOICE_4 3
//#define VOICE_5 4
//#define VOICE_6 5

//Random Numer Generator stuff
#define bit30 0x4000
#define bit27 0x0800
char bit0, bit1;
unsigned long noiseGen = 412294;

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
uint8_t sample(uint8_t);
uint8_t random8Bits(void);

//returns OCR0A
uint8_t sample(uint8_t idx) {

	// compute exponential attack and decay of amplitude
	// the (time & 0x0ff) slows down the decay computation by 256 times		
	if (!(time & 0x0ff)) {
		amp_fall_main[idx] = amp_fall_main[idx] - (amp_fall_main[idx]>>decay_main[idx]);
		rise_phase_main[idx] = rise_phase_main[idx] - (rise_phase_main[idx]>>rise_main[idx]);
		// compute exponential decay of FM depth of modulation
		amp_fm1[idx] = amp_fm1[idx] - (amp_fm1[idx]>>decay_fm1[idx]) ;
	}

	// form (1-exp(-t/tau)) for the attack phase
	amp_rise_main[idx] =  max_amp - rise_phase_main[idx];
	// product of rise and fall exponentials is the amplitude envelope
	amp_main[idx] = (amp_rise_main[idx]>>8) * (amp_fall_main[idx]>>8) ;

	// Init the synth
	if (pluck==1) {
		if(idx == 0) {
			setNextNote();
		}
		amp_fall_main[idx] = max_amp; 
		rise_phase_main[idx] = max_amp ;
		amp_rise_main[idx] = 0 ;
		amp_fm1[idx] = max_amp ;
		// phase lock the synth
		acc_fm1[idx] = 0 ;
		acc_main[idx] = 0;

		//DIRTY UGLY HACK FIX THIS STUPID
		if (idx == (NUM_VOICES - 1)) {
			pluck = 0;
		}

	}

	//the FM DDR -- feeds into final DDR
	acc_fm1[idx] = acc_fm1[idx] + inc_fm1[idx] ;
	high_fm1 = (char)(acc_fm1[idx] >> 8) ;
	fm1 = sineTable[high_fm1] ;

	//the final output DDR 
	// phase accum = main_DDR_freq + FM_DDR * (FM amplitude)
	acc_main[idx] = acc_main[idx] + (inc_main[idx] + (fm1*(amp_fm1[idx]>>depth_fm1[idx]))) ;
	high_main = (char)(acc_main[idx] >> 8) ;
	
	// output the wavefrom sample
	// scale amplitude to use only high byte and shift into range
	// 0 to 255
	return 128 + (((amp_main[idx]>>8) * (int)sineTable[high_main])>>7) ;
}

ISR (TIMER1_COMPA_vect) // Fs = 12000
{ 
	// turn on timer for profiling
	//TCNT2 = 0; TCCR2B = 1;
	
	if(sampling) {
		// Set Sample
		uint8_t idx;
		uint8_t voiceAccum = 0;
		for (idx=0; idx < NUM_VOICES; idx++){
			voiceAccum += sample(idx) >> 2;
		}
		OCR0A = voiceAccum;
		//OCR0A = sample(1);
	}
	
	time++;     //ticks at 12 KHz 
	// profiling 
	//TCCR2B = 0;
} 

// Every 1ms
volatile uint16_t mscount = 0;
ISR (TIMER2_COMPA_vect){
	KeypadDebounce();
	mscount++;
	if(!(mscount % 1100)) {
		pluck = 1;
	}
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

	/////////// timer 1 ticks at 8000 Hz or 125 microsecs period=2000 ticks
	// timer 1 ticks at 12000 Hz = 1333 ticks
	OCR1A = 1332 ; // 2000 ticks
	TIMSK1 = (1<<OCIE1A) ;
	TCCR1B = 0x09; 	//full speed; clear-on-match
	TCCR1A = 0x00;	//turn off pwm and oc lines

	//set up timer 2 for 1 mSec ticks
	TIMSK2 = 2;		//turn on timer 2 cmp match ISR
	OCR2A = 249;	//set the compare reg to 250 time ticks
	TCCR2A = 0b00000010; // turn on clear-on-match
	TCCR2B = 0b00000011;	// clock prescalar to 64


	seqStrLen = strlen(LCDSequenceId);
	voiceStrLen = strlen(LCDVoiceId);
	mainFreqStrLen = strlen(LCDMainFrequency);
	mainDecayStrLen = strlen(LCDMainDecay);
	mainRiseStrLen = strlen(LCDMainRise);
	fmFreqStrLen = strlen(LCDFMFreq);
	fmDepthStrLen = strlen(LCDFMDepth);
	fmDecayStrLen = strlen(LCDFMDecay);

	initLCD();

	// turn on all ISRs
	sei() ;
   

	///////////////////////////////////////////////////
	// Sound parameters
	///////////////////////////////////////////////////
	// Base frequency
	// 2^16/8000*freq = 8.192*freq
	inc_main[0] = (int)(8.192 * 261) ; 
	// rise and decay SHIFT factor  -- bigger is slower
	// 6 implies tau of 64 cycles
	// 8 implies tau of 256 cycles
	// max value is 8
	decay_main[0] = 4 ;
	rise_main[0] = 0 ;
	//
	// FM modulation rate -- also a frequency
	inc_fm1[0] = (int)(8.192 * 65) ;
	// FM modulation depth SHIFT factor 
	// bigger factor implies smaller FM!
	// useful range is 4 to 15
	depth_fm1[0] = 7 ;
	// decay SHIFT factor -- bigger is slower
	// 6 implies tau of 64 cycles
	// 8 implies tau of 256 cycles
	// max value is 8
	decay_fm1[0] = 6 ;

//Bell/chime
   inc_main[0] = (int)(8.192 * 1440) ; 
   decay_main[0] = 5 ;
   rise_main[0] = 1 ;
   inc_fm1[0] = (int)(8.192 * 600) ;
   depth_fm1[0] = 8 ;
   decay_fm1[0] = 6 ;

//Chime:
	inc_main[1] = (int)(8.192 * 261.0) ; 
	decay_main[1] = 5 ;
	rise_main[1] = 1 ;
	inc_fm1[1] = (int)(8.192 * 350.0) ;
	depth_fm1[1] = 9 ;
	decay_fm1[1] = 5 ;

//Plucked String:
	inc_main[2] = (int)(8.192 * 500.0) ; 
	decay_main[2] = 3 ;
	rise_main[2] = 1 ;
	inc_fm1[2] = (int)(8.192 * 750.0) ;
	depth_fm1[2] = 8 ;
	decay_fm1[2] = 3 ;

//Small, stiff rod
	inc_main[3] = (int)(8.192 * 1440) ;   
	decay_main[3] = 3 ;
	rise_main[3] = 1 ;   
	inc_fm1[3] = (int)(8.192 * 50) ; // at 100 get stiff string; at 200 get hollow pipe
	depth_fm1[3] = 10 ; //or 9
	decay_fm1[3] = 5 ;

//Hollow rod
/*	inc_main[4] = (int)(8.192 * 1440) ;   
	decay_main[4] = 3 ;
	rise_main[4] = 1 ;   
	inc_fm1[4] = (int)(8.192 * 200) ; // at 100 get stiff string; at 200 get hollow pipe
	depth_fm1[4] = 10 ; //or 9
	decay_fm1[4] = 5 ;

//Bell
	inc_main[5] = (int)(8.192 * 300) ; 
	decay_main[5] = 5 ;
	rise_main[5] = 0 ;
	inc_fm1[5] = (int)(8.192 * 1000) ;
	depth_fm1[5] = 8 ;
	decay_fm1[5] = 6 ;*/
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
	LCDclr();
	switch (state) {
	 	case MAIN_SCREEN:
			CopyStringtoLCD(LCDHelloTop, 0, 0);
			CopyStringtoLCD(LCDHelloBot, 0, 1);
			break;
		case SET_VIDX:
			CopyStringtoLCD(LCDVoiceId, 0, 1);
			LCDGotoXY(voiceStrLen, 1);
			sprintf(LCDBuffer, "%d", vidx);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
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
}

uint8_t random8Bits(void){
	bit0 = (noiseGen & bit27) > 0;
	bit1 = (noiseGen & bit30) > 0;
	noiseGen <<= 1;
	noiseGen += bit0 ^ bit1;
	return noiseGen & 0x7f;

}

void updateManual(void){

}

void setState(uint8_t s) {
	sampling = 0; // disable sampling when switching states
	state = s;
	updateLCD();
	sampling = (state == MAIN_SCREEN);
}

//set the next note to play
void setNextNote(void){
	uint8_t nextNote = 0;
	uint8_t rnd = random8Bits();
	uint8_t nextNoteFound = 0;
	switch (seqId){
		case 0:
			while (!nextNoteFound){
				if (rnd <= ascendingMarkov[curNote * 8 + nextNote]){
					nextNoteFound = 1;
				} else {
					nextNote++;
				}
			}
			break;

		case 1:
			while (!nextNoteFound){
				if (rnd <= descendingMarkov[curNote * 8 + nextNote]){
					nextNoteFound = 1;
				} else {
					nextNote++;
				}
			}
			break;

		case 2:
			while (!nextNoteFound){
				if (rnd <= uniformMarkov[curNote * 8 + nextNote]){
					nextNoteFound = 1;
				} else {
					nextNote++;
				}
			}
			break;
		
		case 3:
			while (!nextNoteFound){
				if (rnd <= chaoticMarkov[curNote * 8 + nextNote]){
					nextNoteFound = 1;
				} else {
					nextNote++;
				}
			}
			break;
	}
	for(uint8_t i = 0; i < NUM_VOICES; i++) {
		inc_main[i] = (int)(8.192 * markovFrequencies[nextNote]);
	}
	curNote = nextNote;
}

// update to next state if key is pressed
uint8_t waitingForInput = 0;
unsigned int lastInt = 0;
void nextState(void){
	if(waitingForInput) {
		// output input to screen
		unsigned int in = KeypadInt();
		if(lastInt != in) {
			LCDGotoXY(0, 0);
			sprintf(LCDBuffer, "%d", in);
			LCDstring(LCDBuffer, strlen(LCDBuffer));
		}
	}
	uint8_t key = KeypadKey();

	switch(key) {
		case KEY_A:
			waitingForInput = 0;
			setState((state + 1) % 9);
			break;
	}
	if(key) {
		switch (state) {
			case SET_VIDX:
				if(key == KEY_D) {
					vidx = KeypadInt() % NUM_VOICES;
					waitingForInput = 0;
					setState(MAIN_SCREEN);
				} else {
					waitingForInput = 1;
				}
				break;
			case SET_SEQUENCE:
				if(key == KEY_D) {
					seqId = KeypadInt();
					waitingForInput = 0;
					setState(MAIN_SCREEN);
				} else {
					waitingForInput = 1;
				}
				break;
			case SET_INC_MAIN:
				if(key == KEY_D) {
					inc_main[vidx] = KeypadInt();
					waitingForInput = 0;
					setState(MAIN_SCREEN);
				} else {
					waitingForInput = 1;
				}
				break;
			case SET_DECAY_MAIN:
				if(key == KEY_D) {
					decay_main[vidx] = KeypadInt();
					waitingForInput = 0;
					setState(MAIN_SCREEN);
				} else {
					waitingForInput = 1;
				}
				break;
			case SET_RISE_MAIN:
				if(key == KEY_D) {
					rise_main[vidx] = KeypadInt();
					waitingForInput = 0;
					setState(MAIN_SCREEN);
				} else {
					waitingForInput = 1;
				}
				break;
			case SET_INC_FM:
				if(key == KEY_D) {
					inc_fm1[vidx] = KeypadInt();
					waitingForInput = 0;
					setState(MAIN_SCREEN);
				} else {
					waitingForInput = 1;
				}
				break;
			case SET_DEPTH_FM:
				if(key == KEY_D) {
					depth_fm1[vidx] = KeypadInt();
					waitingForInput = 0;
					setState(MAIN_SCREEN);
				} else {
					waitingForInput = 1;
				}
				break;
			case SET_DECAY_FM:
				if(key == KEY_D) {
					decay_fm1[vidx] = KeypadInt();
					waitingForInput = 0;
					setState(MAIN_SCREEN);
				} else {
					waitingForInput = 1;
				}
				break;
		
		}
	}
}
/////////////////////////////////////////////////////
int main(void)
{ 
   Initialize();
   while(1) {  

		random8Bits();
		nextState();

   } // while(1)

}  //} main
////////////////////////////////////////////////////////
