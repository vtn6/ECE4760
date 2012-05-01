#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "lcd_lib.h"
#include "lcd_lib.c"
#include "trtSettings.h"
#include "trtkernel644.c"
#include <util/delay.h>
#include <avr/sleep.h>
#include <ctype.h>

//BECAUSE PYTHON IS GOOD
#define and &&
#define or ||

//DEFINE CONSTANTS
#define MAX_KEYS 16
#define CHARS_IN_VAL 16
#define RELEASED 0
#define UNKNOWN 1
#define PUSHED 2

//DEFINE KEYPAD STATES
#define INPUT_T_REF 0
#define INPUT_THICKNESS 1
#define INPUT_MAT_PROP 2
//DECLARE CONSTANTS
const float VrefRanges[3] = {4.955, 2.56, 1.1};
const uint8_t keytbl[MAX_KEYS]={0xee, 0xed, 0xeb, 
								0xde, 0xdd, 0xdb, 
								0xbe, 0xbd, 0xbb, 
								0xe7, 0xd7, 0xb7,
								0x77, 0x7e, 0x7b, 0x7d};

//VOLATILE VARIABLES
volatile char Ain;//, AinLow;	//The voltage to measure
volatile float Vref;			//The reference voltage
volatile uint8_t rangeIdx = 0;	//which range is currently selected
volatile uint8_t curKey;		//current key pressed 
volatile uint8_t checkKey;		//key that might be pressed
volatile uint8_t keyState;		//current state for Debounce State Machine
volatile uint8_t prevKeyState;	//previous state for Debounce State Machine
volatile uint8_t debounceTime;	//the time at which the program should debounce
volatile uint8_t debouncing;	//flag for whether or not to scan the ketypad
volatile uint8_t mode;			//current mode of the DMM
volatile uint8_t returnMode;	//mode to return to after exiting the manual
volatile uint8_t manPage;		//current page in the manual
volatile uint8_t justSwitched;	//did the DMM just switch states
volatile uint8_t rangeIdx;		//which range is currently selected (not AR)

//KEYPAD VARIABLES
uint8_t waitingForInput = 0;
uint8_t lastInt = 0;
uint8_t key = 0;


//ADC INPUT VARIABLES
float voltage;			//voltage read from the ADC

//LCD variables
uint8_t LCDBuffer[16];  // LCD display buffer
uint8_t valBuffer[16];	// Input parameter buffer

// semaphores to protect shared variables
#define SEM_T_WATER_REF 1
#define SEM_T_WATER 2
#define SEM_T 3
#define SEM_T_REF 4
#define SEM_THICKNESS 5
#define SEM_MAT_PROP 6

//shared variables
float k = 1;			//Thermal conductivity of the food
float thickness = 1;	//Thickness of the food (cm)

//Reference temmperatures of the water (Fahrenheit) and food (Fahrenheit)
float waterTempRef = 100.0, foodTempRef = 100.0;

//Actual temperatures of the water (Fahrenheit) and food (Fahrenheit)
float waterTemp = 100.0, foodTemp = 100.0;


//FUNCTION SIGNATURES
void initialize(void);
void initLCD(void);
void setVref(uint8_t);
void autorange(void);
void debouce(void);
uint8_t scanKeypad(void);
void pidControl(void* args);
void keypadComm(void* args);
uint8_t getCurKey(void);


//Initialize the MCU
void initialize() {
	//enable ADC and set prescaler to 1/128*16MHz=125,000
	//and clear interupt enable
	//and start a conversion
	ADCSRA = (1<<ADEN) + 7;
	
	// Set A to input (high impedence)
	DDRA = 0b11111100;
	PORTA = 0x10; 	//Turn on the pump

	// PortB: Keypad
	DDRB=0x00;

	// PortD: Bluetooth module


	// Set analog comp to connect to timer capture input 
	// and turn on the band gap reference on the positive input  
	ACSR =  (1<<ACIC) ; //0b01000100  ;

	ADMUX = (1 << ADLAR) | (1 << REFS0); //set Vref to 5v
	setVref(0); //Set to 5v Vref
	initLCD();
}

//Initialize the LCD
void initLCD(){
	LCDinit();	//initialize the display
	LCDcursorOFF();
	LCDclr();				//clear the display
	LCDGotoXY(0,0);
	//CopyStringtoLCD(LCDHelloTop, 0, 0);
	//CopyStringtoLCD(LCDHelloBot, 0, 1);
}

//set the reference voltage
void setVref(uint8_t idx) {
	switch(idx) {
		case 0:
			ADMUX = (1 << ADLAR) | (1 << REFS0); //5v
			break;
		case 1:
			ADMUX = (1 << ADLAR) | (3 << REFS0); //2.56v
			break;
		case 2:
			ADMUX = (1 << ADLAR) | (2 << REFS0); //1.1v
			break;
	}
	Vref = VrefRanges[idx];
}

//autorange the reference voltage for best accuracy
// Set rangeIdx if autoRange
void autorange(void){
	switch (rangeIdx){
		//In the 5 Volt range, move to a smaller scale if the voltage is less than 0.525 of Vref
		case 0:
			if (voltage <= 2.0){
				rangeIdx++;
			}
			break;
		
		//In the 2.56 Volt range, move to a smaller scale if the voltage is less than 0.4 of Vref
		//or move to a larger scale if the voltage is close to Vref
		case 1:
			if (voltage < 0.9){
				rangeIdx++;
			}
			else if (voltage > 2.0){
				rangeIdx--;
			}
			break;
		
		//In the 1.1 Volt range, move to a larger scale if the voltage is close to Vref
		case 2:
			if (voltage > 0.9){
				rangeIdx--;
			}
			break;
	}
	setVref(rangeIdx);	
}

//Debounce the button using a debounce state machine
void debounce(void){
	uint8_t key = scanKeypad(); //Scan the keypad
	switch(keyState){
	
	//in the RELEASED state: stay in this state if a key is not pressed
	//go to UNKNOWN if any key is pressed and reset the debounce countdown.
	case RELEASED:
		if (key){
			keyState = UNKNOWN;
			prevKeyState = RELEASED;
			debouncing = 1;
			checkKey = key;
		}
		break;

	//in the UNKNOWN state: go to released if the button is not down
	//go to PUSHED if the button is down
	case UNKNOWN:
		if (key){
			if (key == checkKey){
				keyState = PUSHED;
				prevKeyState = UNKNOWN;
				debouncing = 0;
			}
			else {
				debouncing = 1;
				checkKey = key;
			}
		}
		else{
			keyState = RELEASED;
			if (prevKeyState == PUSHED) {
				curKey = checkKey; //The key to be checked has been pressed and debounced
			}
			prevKeyState = UNKNOWN;
			debouncing = 0;
		}
	
		break;

	//in the PUSHED state go to UnKnown if the button is not down.
	//stay in PUSHED if the button is down
	case PUSHED:
		if (!key){
			keyState = UNKNOWN;
			prevKeyState = PUSHED;
			debouncing = 1;
		}
		else if (key != checkKey){
			keyState = UNKNOWN;
			prevKeyState = RELEASED; //...???
			debouncing = 1;
			curKey = checkKey;
			checkKey = key;
		}
		break;
	}
}

//scan the keypad
uint8_t scanKeypad(void){
	uint8_t key;
	uint8_t butnum;
	//get lower nibble
	DDRB = 0x0f;
	PORTB = 0xf0; 
	_delay_us(5);
	key = PINB;
	
	//get upper nibble
	DDRB = 0xf0;
	PORTB = 0x0f; 
	_delay_us(5);
	key = key | PINB;
	
	//find matching keycode in keytbl
	if (key != 0xff) {
		for (butnum=0; butnum<MAX_KEYS; butnum++){
			if (keytbl[butnum]==key) {
				break;
			}   
		}

		if (butnum==MAX_KEYS) {
			butnum=0;
		}
		else {
			butnum++;	   //adjust by one to make range 1-16
		}
	}
	else butnum=0;
	
	return butnum;
}

//Get the currect key
uint8_t getCurKey(void){
	uint8_t tmpKey = curKey;
	if (curKey){
		curKey = 0;
	}
	return tmpKey;
}

//Actual tasks to complete
//PID Control Stuff...worry about this silt later
// --- define task 1  ----------------------------------------
void pidControl(void* args) 
  {	
  	uint32_t rel, dead ; //relase and deadline times
	float error = 0;		//Calculated error
	float prevError = 0;	//previously calculated error
	float prevWaterTemp = 0; //previously measured motor frequency
	int8_t prevSign = 0;	//previously calculated sign of the error
	
	//Local copies of shared system parameters
	uint16_t localWaterTemp;	
	uint16_t localWaterTempRef;
	
	//PID parameters
	float k_p = 1.0, k_i = 0.0, k_d = 0.0;

	//Declarations for calculated values.
	int8_t sign = 0;
	int16_t derivative;
	int16_t output;
	int16_t integral = 0;
	
	uint8_t first = 1;

	//transduction constant for the LM34
	const float transductionConstant = 0.01; // V/degF	

	while(1)
	{
		//update the previous measuremtns
		if (!first){
			prevWaterTemp = localWaterTemp;
			prevSign = sign;
			prevError = error;
		}

		//poll the ADC and convert the voltage to a temperature
		Ain = ADCH;
		voltage = (float)Ain;
		voltage = (voltage/256.0) * Vref;
		autorange();
		localWaterTemp = voltage * transductionConstant;

		//make local copies of the system parameters
		trtWait(SEM_T_WATER);
		waterTemp = localWaterTemp; 
		trtSignal(SEM_T_WATER);

		trtWait(SEM_T_WATER_REF);
		localWaterTempRef = waterTempRef;
		trtSignal(SEM_T_WATER_REF);

		//Proportional Error
		error = localWaterTempRef - localWaterTemp;

		//Integral Error

		//Get the current sign of the error
		if (!first) {
			if (error - prevError > 0){
				sign = 1;
			}
			else if (error - prevError < 0) {
				sign = -1;
			}
			else {
				sign = 0;
			}
		}
		
		//Update the integral of the error
		if (!first){
			if (sign == prevSign){
				integral += error;
			}
			else{
				integral = 0.8 * error;
			}
		}

		//Derivative Error
		if (!first) {
			derivative = error - prevError;
		}

		//determine what the output should be
		if (!first){
			output = k_p * error + k_i * integral + k_d * derivative;
		}
		else{
			output = k_p * error;
			first = 0;
		}

		//clamp the output between 0 and 255 so we can directly set OCR0A
		if (output < 0){
			OCR0A = 0;
		}
		else if (output > 255) {
			OCR0A = 255; //saturated the controller, turn the integrator off
			integral = 0;
		}
		else {
			OCR0A = output;
		}
		
		if (error > 0){
			//Turn the heating thing on
			PORTA |= 0x08; //Pin 3...???
		}
		else{
			//Turn the heating thing off
			PORTA &= ~0x08;
		}

		ADCSRA |= (1<<ADSC);
		//Set the task to execute again in 0.02 seconds.
		rel = trtCurrentTime() + SECONDS2TICKS(0.19);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.21);
	    trtSleepUntil(rel, dead);
	}
 }

 //get input from the keypad
 void keypadComm(void* args) {
	//uint8_t cmd0, cmd1, cmd2, cmd3;
	//float val0, val1, val2, val3;
	//uint8_t numParams;
	uint8_t key = getCurKey();
	uint8_t valBuffer[16];	// Input parameter buffer
	uint32_t valInt = 0;			// Input parameter - before the decimal place
	uint32_t valDec = 0;		// Input parameter after the decimal place
	uint32_t numAfterDecimal = 1; 
	uint8_t valLoc = 0;
	float val;				//The actual input value

	uint8_t beforeDecimal = 1;		//flag indicating whether or not the current
									//key comes before or after the decimal point

	while(1)
	{
		//If we aren't debouncing, scan the keypad and begin debouncing the signal
		if (!debouncing){
			debounce();
		}

		//Otherwise, finish debouncing the signal
		else {
			debounce();
		}

		if (waitingForInput) {
			switch (key){
				case 0x01:
					valBuffer[valLoc] = '1';
					if (beforeDecimal){
						valInt = valInt * 10 + 1;
					}
					else {
						valDec = valDec * 10 + 1;
						numAfterDecimal *= 10;
					}
					valLoc++;
					break;
				case 0x02:
					valBuffer[valLoc] = '2';
					if (beforeDecimal) {
						valInt = valInt * 10 + 2;
					}
					else {
						valDec = valDec * 10 + 2;
						numAfterDecimal *= 10;
					}
					valLoc++;
					break;
				case 0x03:
					valBuffer[valLoc] = '3';
					if (beforeDecimal) {
						valInt = valInt * 10 + 3;
					}
					else{
						valDec = valDec * 10 + 3;
						numAfterDecimal *= 10;
					}
					valLoc++;
					break;
				case 0x04:
					valBuffer[valLoc] = '4';
					if (beforeDecimal) {
						valInt = valInt * 10 + 4;
					}
					else{
						valDec = valDec * 10 + 4;
						numAfterDecimal *= 10;
					}
					valLoc++;
					break;
				case 0x05:
					valBuffer[valLoc] = '5';
					if (beforeDecimal) {
						valInt = valInt * 10 + 5;
					}
					else{
						valDec = valDec * 10 + 5;
						numAfterDecimal *= 10;
					}
					valLoc++;
					break;
				case 0x06:
					valBuffer[valLoc] = '6';
					if (beforeDecimal) {
						valInt = valInt * 10 + 6;
					}
					else{
						valDec = valDec * 10 + 6;
						numAfterDecimal *= 10;
					}
					valLoc++;
					break;
				case 0x07:
					valBuffer[valLoc] = '7';
					if (beforeDecimal) {
						valInt = valInt * 10 + 7;
					}
					else{
						valDec = valDec * 10 + 7;
						numAfterDecimal *= 10;
					}
					valLoc++;
					break;
				case 0x08:
					valBuffer[valLoc] = '8';
					if (beforeDecimal) {
						valInt = valInt * 10 + 8;
					}
					else{
						valDec = valDec * 10 + 8;
						numAfterDecimal *= 10;
					}
					valLoc++;
					break;
				case 0x09:
					valBuffer[valLoc] = '9';
					if (beforeDecimal) {
						valInt = valInt * 10 + 9;
					}
					else{
						valDec = valDec * 10 + 9;
						numAfterDecimal *= 10;
					}
					valLoc++;
					break;
				case 0x10:
					valBuffer[valLoc] = '0';
					if (beforeDecimal) {
						valInt = valInt * 10;
					}
					else{
						valDec = valDec * 10;
						numAfterDecimal *= 10;
					}
					valLoc++;
					break;
				case 0x0E:		//E ==> decimal
					valBuffer[valLoc] = '.';
					valLoc++;
					beforeDecimal = 0;
				case 0x0F:		//F ==> backspace
					valLoc--;
					if (beforeDecimal){
						valInt = valInt / 10;
					}
					else {
						if (valBuffer[valLoc] == '.'){
							valInt = valInt / 10;
							beforeDecimal = 1;
							numAfterDecimal = 1;
						}
						else {
							valDec = valDec / 10;
							numAfterDecimal /= 10;
						}
					}
				case 0x0D:		//D ==> enter
					waitingForInput = 0;
					val = valInt + (float)valDec / (float)numAfterDecimal;
					
					switch (mode){
						case INPUT_T_REF:
							trtWait(SEM_T_WATER_REF);
							waterTempRef = val;
							trtSignal(SEM_T_WATER_REF);
							break;

						case INPUT_THICKNESS:
							trtWait(SEM_THICKNESS);
							thickness = val;
							trtSignal(SEM_THICKNESS);
							break;

						case INPUT_MAT_PROP:
							trtWait(SEM_MAT_PROP);
							k = val;
							trtSignal(SEM_MAT_PROP);
							break;
					} 
					break;
			}
		}							
	}
}

//Display the Temperature on the LCD
void displayTemp(void* args) 
{
	//String constants
	const uint8_t LCDSpeed[9] = "SPEED: \0";
	const uint8_t LCDRPM[5] = "RPM\0";

	//LCD locations
	const uint8_t T_FOOD_REF_LOC = 0;
	const uint8_t T_FOOD = 5;
	const uint8_t PARAM_LOC = 9;


	//String lenghts
	const uint8_t OMEGA_REF_LEN = 4;
	const uint8_t OMEGA_LEN = 4;
	const uint8_t K_P_LEN = 3;
	const uint8_t K_I_LEN = 3;
	const uint8_t K_D_LEN = 3;
	const uint8_t SPEED_LEN = 7;
	const uint8_t RPM_LEN = 3;

	uint8_t tempRefLen;
	uint8_t tempLen;
	float localTemp;
	float localTempRef;

	//String buffers
	uint8_t LCDTempRef[4];
	const uint8_t LCDTemp[5] = "TEMP\0";
	uint8_t LCDk_p[3];
	uint8_t LCDk_i[3];
	uint8_t LCDk_d[3];

	//flags indicating whether a variable needs to be updated
	uint8_t updateTempRefRef;
	uint8_t updatek_p;
	uint8_t updatek_i;
	uint8_t updatek_d;

	//make local copies of the system parametes
	trtWait(SEM_T_REF);
		localTempRef = waterTempRef;
	trtSignal(SEM_T_REF);

	trtWait(SEM_T);
		localTemp = waterTemp;
	trtSignal(SEM_T);

	trtWait(SEM_THICKNESS);
	float localThickness = thickness;
	trtSignal(SEM_THICKNESS);

	trtWait(SEM_MAT_PROP);
	float localK = k;
	trtSignal(SEM_MAT_PROP);

	LCDGotoXY(0, 0);
	LCDstring(LCDTemp, 4);

	uint8_t LCDTempMeas[5];
	sprintf(LCDTempMeas, "%f", localTemp);
	LCDGotoXY(6, 0);
	LCDstring(LCDTempMeas, 5);

	while(1){

		rel = trtCurrentTime() + SECONDS2TICKS(0.2);
		dead = trtCurrentTime() + SECONDS2TICKS(0.225);
		trtSleepUntil(rel, dead);
		
		trtWait(SEM_T_WATER);
		if (localOmegaRef != omegaRef){
			localOmegaRef = omegaRef;
			updateOmegaRef = 1;
		}
		else{
			updateOmegaRef = 0;
		}
		trtSignal(SEM_T_WATER);
	}

	/*
	LCDGotoXY(0, 0);
  	LCDstring(LCDSpeed, SPEED_LEN);
  	
	sprintf(LCDOmega, "%i", omega);
	LCDGotoXY(OMEGA_LOC, 0);
	omegaLen = strlen(LCDOmega);
	if (omegaLen >= OMEGA_LEN) {
		omegaLen = OMEGA_LEN;
	}
	LCDstring(LCDOmega, omegaLen);

	LCDGotoXY(RPM_LOC, 0);
	LCDstring(LCDRPM, RPM_LEN);

	sprintf(LCDOmegaRef, "%i", localOmegaRef);
	omegaRefLen = strlen(LCDOmegaRef);
	if (omegaRefLen >= OMEGA_REF_LEN) {
		omegaRefLen = OMEGA_REF_LEN;
	}
    LCDGotoXY(OMEGA_REF_LOC, 1);
    LCDstring(LCDOmegaRef, omegaRefLen);
	
	sprintf(LCDk_p, "%f", localk_p);
    LCDGotoXY(K_P_LOC, 1);
    LCDstring(LCDk_p, K_P_LEN);

	sprintf(LCDk_i, "%f", localk_i);
    LCDGotoXY(K_I_LOC, 1);
    LCDstring(LCDk_i, K_I_LEN);

	sprintf(LCDk_d, "%f", localk_d);
    LCDGotoXY(K_D_LOC, 1);
    LCDstring(LCDk_d, K_D_LEN);

	uint32_t rel, dead ;
	//Update the LCD about 5 times a second
	while(1)
	{
		//Check if the reference speed has changed
		trtWait(SEM_OMEGA_REF);
		if (localOmegaRef != omegaRef){
			localOmegaRef = omegaRef;
			updateOmegaRef = 1;
		}
		else{
			updateOmegaRef = 0;
		}
		trtSignal(SEM_OMEGA_REF);

		//Check if the proportional gain has changed
		trtWait(SEM_MAT_PROP);
		if (localk_p != k_p) {
			localk_p = k_p;
			updatek_p = 1;
		}
		else{
			updatek_p = 0;
		}
		trtSignal(SEM_K_P);

		//Check if the integral gain has changed
		trtWait(SEM_K_I);
		if (localk_i != k_i) {
			localk_i = k_i;
			updatek_i = 1;
		}
		else{
			updatek_i = 0;
		}
		trtSignal(SEM_K_I);

		//Check if the derivative gain has changed
		trtWait(SEM_K_D);
		if (localk_d != k_d) {
			localk_d = k_d;
			updatek_d = 1;	
		}
		else{
			updatek_d = 0;
		}
		trtSignal(SEM_K_D);


		//Update the LCD
		if (updateOmegaRef){
			sprintf(LCDOmegaRef, "%i ", localOmegaRef);
			omegaRefLen = strlen(LCDOmegaRef);
			if (omegaRefLen >= OMEGA_REF_LEN) {
				omegaRefLen = OMEGA_REF_LEN;
			}
    		LCDGotoXY(OMEGA_REF_LOC, 1);
	    	LCDstring(LCDOmegaRef, omegaRefLen);
		}

		if (updatek_p){
			sprintf(LCDk_p, "%f", localk_p);
            LCDGotoXY(K_P_LOC, 1);
            LCDstring(LCDk_p, K_P_LEN);
		}

		if (updatek_i){
			sprintf(LCDk_i, "%f", localk_i);
            LCDGotoXY(K_I_LOC, 1);
            LCDstring(LCDk_i, K_I_LEN);
		}

		if (updatek_d){
			sprintf(LCDk_d, "%f", localk_d);
            LCDGotoXY(K_D_LOC, 1);
            LCDstring(LCDk_d, K_D_LEN);
		}

		trtWait(SEM_OMEGA);
		localOmega = omega;
		trtSignal(SEM_OMEGA);
		sprintf(LCDOmega, "%i  ", localOmega);
		LCDGotoXY(OMEGA_LOC, 0);
		omegaLen = strlen(LCDOmega);
		LCDstring(LCDOmega, omegaLen);
		
		//Set the task to run again in 0.2 seconds.
		rel = trtCurrentTime() + SECONDS2TICKS(0.2);
		dead = trtCurrentTime() + SECONDS2TICKS(0.225);
		trtSleepUntil(rel, dead);
	}*/
}

// --- Main Program ----------------------------------
int main(void) {
  int args[2]; 
  DDRD = 0b11111011;
  PORTD = 0;

  //Initialize the MCU  
  initialize();

  // start TRT
  trtInitKernel(128); // 80 bytes for the idle task stack

  // variable protection
  trtCreateSemaphore(SEM_T_REF, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_T, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_T_WATER_REF, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_T_WATER, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_MAT_PROP, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_THICKNESS, 1);
 // --- creat tasks  ----------------
  trtCreateTask(pidControl, 256, SECONDS2TICKS(0.05), SECONDS2TICKS(0.05), &(args[0]));
  trtCreateTask(keypadComm, 256, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  trtCreateTask(displayTemp, 256, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  
  sei();
  // --- Idle task --------------------------------------
  // just sleeps the cpu to save power 
  // every time it executes
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  while (1) 
  {
  	sleep_cpu();
  }

}
