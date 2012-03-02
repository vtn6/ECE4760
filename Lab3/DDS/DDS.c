// DDS output thru PWM on timer0 OC0A (pin B.3)
// Mega644 version
// FM synthesis

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h> 		// for sine
#include <stdio.h>
#include "uart.h"
// set up serial for debugging
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

//I like these definitions
#define begin {
#define end   } 

// The DDS variables 
volatile unsigned int acc_main, acc_fm1 ;
volatile unsigned char high_main, high_fm1, decay_fm1, decay_main, depth_fm1, rise_main ;
volatile unsigned int inc_main, inc_fm1, amp_main, amp_fm1 ;
volatile unsigned int rise_phase_main, amp_rise_main, amp_fall_main ;
#define max_amp 32767
// tables for DDS			
signed char sineTable[256], fm1 ;

// trigger
volatile char pluck, pushed ;

// Time variables
// the volitile is needed because the time is only set in the ISR
// time counts mSec, sample counts DDS samples (62.5 KHz)
volatile unsigned int time ;
volatile char  count;

// index for sine table build
unsigned int i;

ISR (TIMER1_COMPA_vect) // Fs = 8000
begin 
	// turn on timer for profiling
	TCNT2 = 0; TCCR2B = 1;

	// compute exponential attack and decay of amplitude
	// the (time & 0x0ff) slows down the decay computation by 256 times		
	if ((time & 0x0ff) == 0) begin
		amp_fall_main = amp_fall_main - (amp_fall_main>>decay_main) ;
		rise_phase_main = rise_phase_main - (rise_phase_main>>rise_main);
		// compute exponential decay of FM depth of modulation
		amp_fm1 = amp_fm1 - (amp_fm1>>decay_fm1) ;
	end

	// form (1-exp(-t/tau)) for the attack phase
	amp_rise_main =  max_amp - rise_phase_main;
	// product of rise and fall exponentials is the amplitude envelope
	amp_main = (amp_rise_main>>8) * (amp_fall_main>>8) ;

	// Init the synth
	if (pluck==1) begin
		amp_fall_main = max_amp; 
		rise_phase_main = max_amp ;
		amp_rise_main = 0 ;
		amp_fm1 = max_amp ;
		// phase lock the synth
		acc_fm1 = 0 ;
		acc_main = 0;
		pluck = 0;
	end

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
	OCR0A = 128 + (((amp_main>>8) * (int)sineTable[high_main])>>7) ;
	
	time++;     //ticks at 8 KHz 
	// profiling 
	TCCR2B = 0;
end 
 
/////////////////////////////////////////////////////
int main(void)
begin 
   
   // make B.3 an output
   DDRB = (1<<PINB3) ;
   
   //init the UART -- uart_init() is in uart.c
  	uart_init();
  	stdout = stdin = stderr = &uart_str;
  	fprintf(stdout,"Starting...\n\r");

   // init the sine table
   for (i=0; i<256; i++)
   begin
   		sineTable[i] = (char)(127.0 * sin(6.283*((float)i)/256.0)) ;
   end  

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
  ////////////////////////////////////////////////////

   while(1) begin  
		// Check pushbutton to pluck string
		// and oneshot it
		//  
		if ((time & 0xff) == 0) begin
			if ((~PINC & 0x01) && !pushed) begin
				 pluck = 1;
				 pushed = 1;
 			end
			if (!(~PINC & 0x01)  && pushed) begin
				pushed = 0;
			end
		//	printf("%d\n\r", TCNT2);
		end		

   end // while(1)

end  //end main
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
