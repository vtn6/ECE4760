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

// semaphore to protect shared variable
#define SEM_SHARED 3

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
float k_p;
float k_i;
float k_d;

//Reference speed
float omegaRef;

//actual speed
float omega; 


//function signatures
void setParam(uint8_t, float); //Helper method for setting PID parameters
//PID Control Stuff...worry about this silt later
// --- define task 1  ----------------------------------------
void buttonComm(void* args) 
  {	
  	uint32_t rel, dead ;
	uint8_t sw, sw_num ;
	uint8_t sw_state ;

	sw_state = 0 ; // no buttons pushed
	DDRC = 0xff;    // led connections
  	PORTC = 0xff;
	DDRA = 0x01 ;
	

	while(1)
	{
		PORTA =  PORTA ^ 0x01 ;
		// read the buttons
		// if a button is pushed,
		// latch on the corresponding LED
		sw = ~PINB ;
		// update shared leds
		trtWait(SEM_SHARED) ;
		led = led | sw ;
		PORTC = ~led ;
		trtSignal(SEM_SHARED);
		
		// chessy debouncer
		if (sw_state == 0 && sw!=0) // new button push?
		{
			// convert from binary to switch number (0 to 7)
			sw_num = 0 ;
			while(sw>1)
			{
				sw = sw>>1; 
				sw_num++ ;
			}
			fprintf(stdout,"Button pushed=%d\n\r>", sw_num) ;
			sw_state = 1 ;
		}
		if (sw_state == 1 && sw==0) // button release?
		 	sw_state = 0 ;
		// end debouncer

		// Sleep
		// debouncer works well with 50 mSec sleep
		

	    rel = trtCurrentTime() + SECONDS2TICKS(0.01);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.01);
	    trtSleepUntil(rel, dead);
	}
  }


//read the commands from the user
// --- define task 2  ----------------------------------------
void serialComm(void* args) 
  {
	uint8_t cmd0, cmd1, cmd2, cmd3;
	float val0, val1, val2, val3;

	uint8_t numParams;

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
		int  valpoo;
		fscanf(stdin, "%c %d", &cmd0, &valpoo);
		//trtWait(SEM_STRING_DONE);

		if (cmd0 == 'a')
			fprintf(stdout, "OCAML: %d\n", valpoo);

		// update shared leds
		//trtWait(SEM_SHARED);
		

		/*
		if (!(numParams % 2))
			numParams >>= 1;
		
		
		//If the user only set 1 parameter double check that the command is
		//valid and update the appropriate parameter
		if (numParams == 1){
			setParam(cmd0, val0);
		}
		
		//If the user set 2 parameters ensure that both commands are recognized
		//and that the user did not set the same parameter twice
		else if (numParams == 2){
			if (cmd0 != cmd1){
				setParam(cmd0, val0);
				setParam(cmd1, val1);
			}
			else {
				fprintf(stdout, "Cannot change the same parameter twice");
			}
		}

		//If the user is a dumb sacK of shit and thinKs it's cool to set three
		//parameters then we better maKe sure that he sets three difference,
		//legal parameters
		else if (numParams == 3){
			if (cmd0 != cmd1 and cmd0 != cmd2 and cmd1 != cmd2){
				setParam(cmd0, val0);
				setParam(cmd1, val1);
				setParam(cmd2, val2);
			}
			else {
				fprintf(stdout, "Cannot change the same parameter twice\n");
			}
		}
		//If the user is a cool Kid and sets all four parameters, then ensure
		//that each parameter is set once and only once
		else if (numParams == 4){
			if (cmd0 != cmd1 and cmd0 != cmd2 and cmd0 != cmd3 and 
				cmd1 != cmd2 and cmd1 != cmd3 and cmd2 != cmd3){
				setParam(cmd0, val0);
				setParam(cmd1, val1);
				setParam(cmd2, val2);
				setParam(cmd3, val3);
			}
			else {
				fprintf(stdout, "Cannot change the same parameter twice\n");
			}
		}

		//The user did something weird, let him Know
		else {
			fprintf(stdout, "Invalid number of arguments.\n");
		}*/
		//trtSignal(SEM_SHARED);
		
	}
  }

// --- spoiler ---------------------------------------
void spoiler(void* args) 
{
	uint32_t rel, dead ;
	//Ties up the kernel so that the button task runs 
	// intermittantly UNLESS deadline-release>=delay time
	while(1)
	{
		_delay_ms(100);
		rel = trtCurrentTime() + SECONDS2TICKS(0.1);
		dead = trtCurrentTime() + SECONDS2TICKS(0.2);
		trtSleepUntil(rel, dead);
	}
}
// --- Main Program ----------------------------------

void setParam(uint8_t cmd, float val){
	cmd = tolower(cmd);
	if (cmd == 's')
		omegaRef = val;
	else if (cmd == 'p')
		k_p = val;		
	else if (cmd == 'i')		
		k_i = val;
	else if (cmd == 'd')
		k_d = val;
	else
		fprintf(stdout, "Command %c unrecognized\n", cmd);
}

int main(void) {


  //init the UART -- trt_uart_init() is in trtUart.c
  trt_uart_init();
  stdout = stdin = stderr = &uart_str;
  fprintf(stdout,"\n\r TRT 9feb2009\n\r\n\r");
  
  // start TRT
  trtInitKernel(80); // 80 bytes for the idle task stack

  // --- create semaphores ----------
  // You must creat the first two semaphores if you use the uart
  trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>
  
  // variable protection
  trtCreateSemaphore(SEM_SHARED, 1) ; // protect shared variables

 // --- creat tasks  ----------------
  trtCreateTask(buttonComm, 200, SECONDS2TICKS(0.05), SECONDS2TICKS(0.05), &(args[0]));
  trtCreateTask(serialComm, 200, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  trtCreateTask(spoiler, 200, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  
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
