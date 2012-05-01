// test TRT with three tasks:
// Task 1  read the speed from the PID algorithm.
//		   runs about 50 times per second
// Task 2  reads a command from the uart console
//         sets, clears, or toggles an LED,
//         command format is
//		   the char s, p, i, or d. Each char must be
//		   followed by a number. Each char/number pair
//		   is spearated by a comma or at least one 
//		   white space character. Each parameter - arg
//		   pair should be separated by at least one white
//		   space character. 
//		   between 0 and 7, e.g. 's 0'
// Task 3  Displays info to the LCD. Runs about 5
//		   times per second
// spoiler Ties up the kernel so that the button task runs 
// intermittantly UNLESS deadline-release>delay time

#include "trtSettings.h"
#include "trtkernel644.c"
#include <stdio.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <ctype.h>
#include "lcd_lib.h"	
#include <avr/pgmspace.h>
#include <string.h>
//#include "trtQuery.c"


// serial communication library
// Don't mess with the semaphores
#define SEM_RX_ISR_SIGNAL 1
#define SEM_STRING_DONE 2 // user hit <enter>
#include "trtUart.h"
#include "trtUart.c"
// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// semaphore to protect shared variables
#define SEM_TEMP_REF 3
#define SEM_TEMP 4
#define SEM_SONAR 5 
#define SEM_OVERFLOW 6
#define SEM_START_HEATING 7
#define SEM_THICKNESS 8

// the usual
#define and &&
#define or ||

// --- control LEDs from buttons and uart -------------------
// input arguments to each thread
// not actually used in this example
int args[2];

uint8_t tempRef = 25; //Centigrade Reference temperature.(temperature of the food)
uint8_t waterTempRef = 30; //Centigrade Reference temperature of the water


uint8_t temp = 10;	//Centigrade calculated temperate of the food
uint8_t waterTemp = 9;	//Centigrade, measured water temperature
 
uint8_t thickness = 0;	//cm, thickness of the food
//flag indicating the PID controller to start heating the water
uint8_t startHeating = 0;

//time for the ultrasonic reange finder pulse to return 
volatile int duration = 0;
volatile int durationOVF = 0;

//actual distance to the surface of the bath
volatile float distance;

//flag indicating when the current measurement has finished
volatile uint8_t sonarFinished;

//flag indicating that the bath is overflowing
uint8_t overflow = 0;

//function signatures
void InitLCD(void);

// --- external interrupt ISR ------------------------
ISR (INT1_vect) {
		if (PORTA & 0x01){
			TCNT2  = 0; 	//Reset timer 2 so we can start measuring the width
							//of the pulse;
		}
		else {
			duration = TCNT2 + durationOVF;
	        durationOVF = 0;
			distance = 171.6 * duration * 0.000064;
			sonarFinished = 1;
			trtSignal(SEM_SONAR);
		}
}

// --- set up extra 8 bits on timer 2 ----------------
ISR (TIMER2_OVF_vect) {
        durationOVF += 255;
}

// --- define a task for making sonar measurements
void sonar(void* args){
	uint32_t rel, dead ; //relase and deadline times
	uint8_t threshold = 5;	//cm, If a new value differs by this much, assume the
							//food was put in
							
	uint8_t refHeight = 0; 		//cm, the (final) height of the water without food
	uint8_t newHeight = 0;		//cm, new height of the bath after adding food
	uint8_t heightAvg = 0;		//cm, average height of the water. without food

	uint8_t newHeightSet = 0; 	//Flag indicating that the thickness of the food
								//has been calculated
	uint8_t refHeightSet = 0;	//Flag indicating that we have measured
								//the reference distance from the sonar
								//to the water's surface

	//Boundaries so we don't measure the top or bottom of the bath
	uint8_t minDist = 3;	//cm
	uint8_t maxDist = 30; //cm

	uint8_t numSamples = 0;	//he number of samples currently taken in the set
	uint8_t desiredNumSamples = 64; //the number of samples to take in a set.
	
	uint8_t prevDistance = 0; //cm, the previous measurement

	//Ratio of the area of the bath and cooking bags
	uint8_t areaRatio = 2;
	
	uint8_t firstMeasurementTaken = 0;	//flag indicating that the very first measurement has not been taken
	uint8_t firstSample = 0;	//cm, first measurement out of 64 in a set of samples

	//flags inicating which distance we are measuring
	uint8_t measuringRefHeight = 1;
	uint8_t measuringNewHeight = 0;

	while(1){
		
		PORTA |= 0x01;	//Set the trigger pin high to start the pulse
		_delay_us(10);	//Give the ultrasonic 10 us to send the pulse
		PORTA &= ~0x01;	//Set the trigger pin low to stop transmitting
		sonarFinished = 0;
		//Use TRTAccept to read the sonarFinishedSemaphor
		while (!sonarFinished){
			trtWait(SEM_SONAR);
		}
		numSamples++;

		//if this is the first sample, just record the distance
		//as the height, previous measurement, and first sample.
		if (!firstMeasurementTaken){
			prevDistance = distance;
			heightAvg = distance;
			firstMeasurementTaken = 1;
			firstSample = distance;
		}

		//Otherwise determine what we wanted to measure
		//and whether or not the state ofthe cooker has changed
		else {
			//check if the measurement is valid.
			if  (distance < maxDist and distance > minDist) {
				
				//check if the depth increased by more than 5cm (corresponds to
				// adding or removing food).
				if (distance - prevDistance > threshold or prevDistance - distance > 5){
					numSamples = 1;
					firstSample = distance;
					newHeight = distance;
					measuringNewHeight = 1;
					measuringRefHeight = 0;
				}
				
				else {
					if (!measuringNewHeight){
						if (!numSamples){
							firstSample = distance;
							heightAvg = 0;
						}
						heightAvg += distance;
					}
					else {
						newHeight += distance;
					}
					numSamples++;
				}	
				//check if we are done sampling
				if (numSamples == desiredNumSamples){
					//check if we are setting the reference height
					//or the new height
					if (!measuringNewHeight){
						heightAvg <<= 6;
						//check if the measurement has increased substantially
						//since taking the first sample. If so, the water level
						//is rising and we should start sampling again
						if (heightAvg - firstSample > threshold or firstSample - heightAvg > threshold){
							measuringRefHeight = 1;
							numSamples = 0;
						}

						//the water level has settled. Set the reference height flag
						//and clear the measuring reference height flag
						else{
							if (measuringRefHeight){
								refHeight = heightAvg;
								refHeightSet = 1;
								measuringRefHeight = 0;
							}
							numSamples = 0;
						}
					}
					
					//We finished taking measurements for the new height
					//of the bath
					else{
						//set the new height of the bath, calculate the thickness
						//of the food, and signal the heating task to start heating
						//the water
						newHeight <<= 6;
						newHeightSet = 1;
						measuringNewHeight = 0;
						trtWait(SEM_THICKNESS);
						thickness = areaRatio * (newHeight - refHeight);
						trtSignal(SEM_THICKNESS);
						trtSignal(SEM_START_HEATING);
						numSamples =0 ;
					}
				}
			}
			else if (distance <= minDist){
				overflow = 1;
			}
			else{
				firstMeasurementTaken = 0;
				measuringNewHeight = 0;
				measuringRefHeight = 1;
				numSamples = 0;
			}

			prevDistance = distance;
		}

		rel = trtCurrentTime() + SECONDS2TICKS(0.015625);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.02);
	    trtSleepUntil(rel, dead);
	}

}
/*
//Code for pulsing the sonar to determine the new volume of the water bath
//This will allow us to calculate the thickness of the food being cooked
//The number of samples averaged is equal to 2 ^ numSamples. This will let
//us use shifts inst
uint32_t getDistance(uint8_t desiredNumSamples) {
	uint8_t sampleId;
	uint32_t distance = 0;
	for (sampleId = 0; sampleId < desiredNumSamples){
		distance += measureDistance();
	}

	distance >>= numSamples;
	return distance;
}
*/

//PID Control Stuff keep the water temperature constant
// --- define task 1  ----------------------------------------
void pidControl(void* args) 
  {	
  	uint32_t rel, dead ; //relase and deadline times
	int16_t error = 0;		//Calculated error
	int16_t prevError = 0;	//previously calculated error
	uint16_t prevTemp; //previously measured motor frequency
	int8_t prevSign =0 ;	//previously calculated sign of the error
	
	//Local copies of shared system parameters
	uint8_t localTemp = 0;	
	uint8_t localTempRef;
	uint8_t localWaterTemp;	
	uint8_t localWaterTempRef;
	float k_p = 7.1;
	float k_i = 0.11;
	float k_d = 0.68;

	//transduction constant for LM35
	float transductionConstant = 0.1;
	
	//Declarations for calculated values.
	int8_t sign = 0;
	int16_t derivative;
	int16_t output;
	int16_t integral = 0;
	
	uint8_t first = 1;

	DDRB = 0xff;
	PORTB = 0;
	while(1)
	{

		while (!startHeating){
			trtWait(SEM_START_HEATING);
		}


		Ain = ADCH;
		voltage = (float)Ain;
		voltage = (voltage/256.0) * Vref;
		localWaterTemp = voltage * transductionConstant 
		//update the previous measuremtns
		if (!first){
			prevTemp = localTemp;
			prevSign = sign;
			prevError = error;
		}



		localWaterTemp = 0;
		//make local copies of the system parameters
		trtWait(SEM_TEMP);
		waterTemp = localWaterTemp; 
		trtSignal(SEM_TEMP);

		trtWait(SEM_TEMP_REF);
		localWaterTempRef = waterTempRef;
		trtSignal(SEM_TEMP_REF);

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
				integral = 0;
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
			OCR0A = 255;
		}
		else {
			OCR0A = output;
		}

		ADCSRA |= (1<<ADSC); //start another converstion
		//Set the task to execute again in 0.02 seconds.
		rel = trtCurrentTime() + SECONDS2TICKS(0.02);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.025);
	    trtSleepUntil(rel, dead);
	}
  }


/*
//read the commands from the user
// --- define task 2  ----------------------------------------
void serialComm(void* args) 
  {
	//uint8_t cmd0, cmd1, cmd2, cmd3;
	//float val0, val1, val2, val3;
	//uint8_t numParams;
	float  val;
	uint8_t cmd;

	while(1)
	{
		// commands:
		// 's val0 p val1 i val2 d val3 ' - set the reference speed to val0
		// 									set the proportional gain to val1
		//									set the integral gain to val2
		//									set the differential gain to val3
		//If an invalid command is received alert the user of what went wrong
		
		fprintf(stdout, ">") ;
		//numParams = fscanf(stdin, "%c %f %c %f %c %f %c %f", &cmd0, &val0, &cmd1, &val1, &cmd2, &val2, &cmd3, &val3) ;
		
		fscanf(stdin, "%s%f", &cmd, &val);
		//trtWait(SEM_STRING_DONE);

		//update the parameters
		//fprintf(stdout, "Cmd is %c\n", cmd);
		//fprintf(stdout, "Val is %f\n", val);
		if (val >= 0) {
			switch (cmd){
				case 's':
					trtWait(SEM_OMEGA_REF);
					omegaRef = (int) val;
					trtSignal(SEM_OMEGA_REF);
					break;
				case 'p':
					trtWait(SEM_K_P);
					k_p = val;
					trtSignal(SEM_K_P);
					break;
				case 'i':
					trtWait(SEM_K_I);
					k_i = val;
					trtSignal(SEM_K_I);
					break;
				case 'd':
					trtWait(SEM_K_D);
					k_d = val;
					trtSignal(SEM_K_D);
					break;
				default:
					fprintf(stdout, "Command %c not recognized\n", cmd);
					break;
			}
		}
		else{
			fprintf(stdout, "Parameters must be non negative, %f is negative\n", val);
		}
	}
  } */

/*
// --- spoiler ---------------------------------------
void displayParams(void* args) 
{
	//String constants
	const uint8_t LCDSpeed[9] = "SPEED: \0";
	const uint8_t LCDRPM[5] = "RPM\0";

	//LCD locations
	const uint8_t OMEGA_REF_LOC = 0;
	const uint8_t K_P_LOC = 5;
	const uint8_t K_I_LOC = 9;
	const uint8_t K_D_LOC = 13;
	const uint8_t OMEGA_LOC = 7;
	const uint8_t RPM_LOC = 13;

	//String lenghts
	const uint8_t OMEGA_REF_LEN = 4;
	const uint8_t OMEGA_LEN = 4;
	const uint8_t K_P_LEN = 3;
	const uint8_t K_I_LEN = 3;
	const uint8_t K_D_LEN = 3;
	const uint8_t SPEED_LEN = 7;
	const uint8_t RPM_LEN = 3;

	uint8_t omegaRefLen;
	uint8_t omegaLen;
	uint16_t localOmega;

	//String buffers
	uint8_t LCDOmegaRef[4];
	uint8_t LCDOmega[4];
	uint8_t LCDk_p[3];
	uint8_t LCDk_i[3];
	uint8_t LCDk_d[3];

	//flags indicating whether a variable needs to be updated
	uint8_t updateOmegaRef;
	uint8_t updatek_p;
	uint8_t updatek_i;
	uint8_t updatek_d;

	//make local copies of the system parametes
	trtWait(SEM_OMEGA_REF);
	int localOmegaRef = omegaRef;
	trtSignal(SEM_OMEGA_REF);

	trtWait(SEM_K_P);
	float localk_p = k_p;
	trtSignal(SEM_K_P);

	trtWait(SEM_K_I);
	float localk_i = k_i;
	trtSignal(SEM_K_I);

	trtWait(SEM_K_D);
	float localk_d = k_d;
	trtSignal(SEM_K_D);


	//initialize the LCD
	InitLCD();
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
		trtWait(SEM_K_P);
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
	}
}
*/
// --- Initialize the LCD ----------------------------
void InitLCD(void){
  LCDinit();  //initialize the display
  LCDcursorOFF();
  LCDclr();        //clear the display
  LCDGotoXY(0,0);
}

// --- Main Program ----------------------------------
int main(void) {

  DDRD = 0b11111011;
  PORTD = 0;
  //init the UART -- trt_uart_init() is in trtUart.c
  trt_uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout,"\n\r TRT 9feb2009\n\r\n\r");
  
  //enable ADC and set prescaler to 1/128*16MHz=125,000
  //and clear interupt enable
  //and start a conversion
    ADCSRA = (1<<ADEN) + 7;

  // Set analog comp to connect to timer capture input 	
  // and turn on the band gap reference on the positive input  
    ACSR =  (1<<ACIC) ; //0b01000100  ;
  
  //initialize Timer2 and the external interrupt
  //set up INT0
	EIMSK = 1<<INT1 ; // turn on int0
	EICRA = 1 << ISC10 ;       // trigger on any edge edge
	// turn on timer 2 to be read in int0 ISR
	TCCR2B = 7 ; // divide by 1024
	// turn on timer 2 overflow ISR for double precision time
	TIMSK2 = 1;


  //setup Timer 0
  // Set the timer for fast PWM mode, clear OC0A on Compare Match, set OC0A
  // at BOTTOM (non-inverting mode)
  TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); //Set the timer

  //Set the prescalar to 256 so the PWM runs at less than 1000 Hz
  TCCR0B = (1 << CS02) | (1 << CS00);

  OCR0A = 127;

  // start TRT
  trtInitKernel(128); // 80 bytes for the idle task stack

  // --- create semaphores ----------
  // You must creat the first two semaphores if you use the uart
  trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>
  
  // variable protection
  trtCreateSemaphore(SEM_TEMP_REF, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_TEMP, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_OVERFLOW, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_THICKNESS, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_START_HEATING, 0) ; // protect shared variables
  trtCreateSemaphore(SEM_SONAR, 1); // Condition Variable for sonar measurement

 // --- creat tasks  ----------------
  trtCreateTask(pidControl, 256, SECONDS2TICKS(0.05), SECONDS2TICKS(0.05), &(args[0]));
 // trtCreateTask(serialComm, 256, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
 // trtCreateTask(displayParams, 256, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  trtCreateTask(sonar, 256, SECONDS2TICKS(0.05), SECONDS2TICKS(0.05), &(args[0]));
  
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

} // main
