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
#define SEM_OMEGA_REF 3
#define SEM_K_P 4
#define SEM_K_I 5
#define SEM_K_D 6
#define SEM_OMEGA 7
#define SEM_MAX_OUTPUT 8
// the usual
#define and &&
#define or ||

// --- control LEDs from buttons and uart -------------------
// input arguments to each thread
// not actually used in this example
int args[2];

// shared led status
uint8_t led;

//shared variables
//PID parameters
float k_p = 1.0, k_i = 1.0, k_d = 1.0;

//Reference speed
uint16_t omegaRef = 1;

//Max output
uint16_t maxOutput = 9001;

//actual speed
uint16_t omega = 1; 

//motor speed
volatile int motor_period;
volatile int motor_period_ovlf;

//function signatures
void setParam(uint8_t, float); //Helper method for setting PID parameters
void InitLCD(void);

// --- external interrupt ISR ------------------------
ISR (INT0_vect) {
        motor_period = TCNT2 + motor_period_ovlf  ;
        TCNT2 = 0 ;
        motor_period_ovlf = 0 ;
}
// --- set up extra 8 bits on timer 2 ----------------
ISR (TIMER2_OVF_vect) {
        motor_period_ovlf = motor_period_ovlf + 256 ;
}

//PID Control Stuff...worry about this silt later
// --- define task 1  ----------------------------------------
void pidControl(void* args) 
  {	
  	uint32_t rel, dead ;
	uint16_t pError;
	uint16_t prevPError;
	uint16_t prevOmega;
	uint8_t prevSign;
	uint16_t localOmega;
	uint16_t localOmegaRef;
	uint8_t sign;
	uint16_t derivative;
	uint16_t output;
	uint16_t localMaxOutput;
	uint16_t integral;
	float localk_p;
	float localk_i;
	float localk_d;

	while(1)
	{
		prevOmega = localOmega;
		prevSign = sign;
		localOmega = ((20000000 / 1024) / 7) / motor_period;

		trtWait(SEM_OMEGA);
		omega = localOmega;
		trtSignal(SEM_OMEGA);

		trtWait(SEM_OMEGA_REF);
		localOmegaRef = omegaRef;
		trtSignal(SEM_OMEGA_REF);

		trtWait(SEM_K_P);
		localk_p = k_p;
		trtSignal(SEM_K_P);

		trtWait(SEM_K_I);
		localk_i = k_i;
		trtSignal(SEM_K_I);

		trtWait(SEM_K_D);
		localk_d = k_d;
		trtSignal(SEM_K_D);

		trtWait(SEM_MAX_OUTPUT);
		localMaxOutput = maxOutput;
		trtSignal(SEM_MAX_OUTPUT);

		//Proportional Error
		pError = localOmegaRef - localOmega;

		//Integral Error

		//Get the current sign of the error
		if (pError - prevPError > 0){
			sign = 1;
		}
		else if (pError - prevPError < 0) {
			sign = -1;
		}
		else {
			sign = 0;
		}

		//Update the integral of the error
		if (sign == prevSign){
			integral += pError;
		}
		else{
			integral = 0;
		}

		//Derivative Error
		derivative = pError - prevPError;

		output = localk_p * pError + localk_i * integral + localk_d * derivative;

		if (output < 0){
			OCR0A = 0;
		}
		else{
			OCR0A = 255 * output / localMaxOutput;
		}

		rel = trtCurrentTime() + SECONDS2TICKS(0.005);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.01);
	    trtSleepUntil(rel, dead);
	}
  }


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
  }

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

	//String buffers
	uint8_t LCDOmegaRef[4];
	uint8_t LCDOmega[4];
	uint8_t LCDk_p[3];
	uint8_t LCDk_i[3];
	uint8_t LCDk_d[3];

	uint8_t updateOmegaRef;
	uint8_t updatek_p;
	uint8_t updatek_i;
	uint8_t updatek_d;

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
			sprintf(LCDOmegaRef, "%i", localOmegaRef);
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
		sprintf(LCDOmega, "%i", omega);
		LCDGotoXY(OMEGA_LOC, 0);
		omegaLen = strlen(LCDOmega);
		if (omegaLen >= OMEGA_LEN) {
			omegaLen = OMEGA_LEN;
		}
		LCDstring(LCDOmega, omegaLen);
		trtSignal(SEM_OMEGA);

		rel = trtCurrentTime() + SECONDS2TICKS(0.1);
		dead = trtCurrentTime() + SECONDS2TICKS(0.225);
		trtSleepUntil(rel, dead);
	}
}

// --- Initialize the LCD ----------------------------
void InitLCD(void){
  LCDinit();  //initialize the display
  LCDcursorOFF();
  LCDclr();        //clear the display
  LCDGotoXY(0,0);
}

// --- Main Program ----------------------------------
int main(void) {


  //init the UART -- trt_uart_init() is in trtUart.c
  trt_uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout,"\n\r TRT 9feb2009\n\r\n\r");
  
  //initialize Timer2 and the external interrupt
  //set up INT0
	EIMSK = 1<<INT0 ; // turn on int0
	EICRA = 3 ;       // rising edge
	// turn on timer 2 to be read in int0 ISR
	TCCR2B = 7 ; // divide by 1024
	// turn on timer 2 overflow ISR for double precision time
	TIMSK2 = 1 ;

  //setup Timer 0
  // Set the timer for fast PWM mode, clear OC0A on Compare Match, set OC0A
  // at BOTTOM (non-inverting mode)
  TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); //Set the timer

  //Set the prescalar to 64 so the PWM runs at less than 1000 Hz
  TCCR0B = (1 << CS01) | (1 << CS00);

  OCR0A = 127;

  // start TRT
  trtInitKernel(128); // 80 bytes for the idle task stack

  // --- create semaphores ----------
  // You must creat the first two semaphores if you use the uart
  trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>
  
  // variable protection
  trtCreateSemaphore(SEM_OMEGA_REF, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_OMEGA, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_K_P, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_K_I, 1) ; // protect shared variables
  trtCreateSemaphore(SEM_K_D, 1) ; // protect shared variables
 // --- creat tasks  ----------------
  trtCreateTask(pidControl, 256, SECONDS2TICKS(0.05), SECONDS2TICKS(0.05), &(args[0]));
  trtCreateTask(serialComm, 256, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  trtCreateTask(displayParams, 256, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  
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
