#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "uart.h"
#include "lcd_lib.h"

//BECAUSE PYTHON IS GOOD*******************************************************
#define and &&
#define or ||
#define begin {
#define end }
//END BECAUSE PYTHON IS GOOD***************************************************

//DEFINE CONSTANTS*************************************************************
#define MAX_KEYS 16
#define RELEASED 0
#define UNKNOWN 1
#define PUSHED 2
#define DEBOUNCE_TIME 30
#define MODE_START 6
#define RANGE_START 7
//END CONSTANTS****************************************************************

//DEFINE STATES FOR THE DMM***************************************************
#define INIT 0
#define MAN 1
#define VOLTMETER 2
#define OHMMETER 3
#define FREQMETER 4
//END STATES FOR THE DMM*******************************************************

//DEFINE STATES FOR THE MANUAL*************************************************
#define WELCOME 0
#define NAVIGATION_1 1
#define NAVIGATION_2 2
#define VOLT_MAN 3
#define OHM_MAN 4
#define FREQ_MAN 5
#define AUTORANGE_MAN_1 6
#define AUTORANGE_MAN_2 7
//END STATES FOR THE MANUAL****************************************************

//DEFINE LCD STRINGS***********************************************************
const uint8_t LCDBlank[] PROGMEM = "                \0";	// Blank message
const uint8_t LCDHelloTop[] PROGMEM = "DMM MASTER v9001\0";	// Welcome Message
const uint8_t LCDHelloBot[] PROGMEM = "PRESS # FOR HELP\0";
const uint8_t LCDRange[] PROGMEM = "RANGE: \0";
const uint8_t LCDAutorangeOn[] PROGMEM = "AUTORANGE: ON\0";
const uint8_t LCD5Volts[] PROGMEM = "5 VOLTS\0";
const uint8_t LCD256Volts[] PROGMEM = "2.56 VOLTS\0";
const uint8_t LCD11Volts[] PROGMEM = "1.1 VOLTS\0";
const uint8_t LCD1kOhm[] PROGMEM = "1 kOHM\0";
const uint8_t LCD10kOhm[] PROGMEM = "10 kOHM\0";
const uint8_t LCD100kOhm[] PROGMEM = "100 kOHM\0";
const uint8_t LCD1kHz[] PROGMEM = "1 kHZ\0";
const uint8_t LCD10kHz[] PROGMEM = "10 kHZ\0";
const uint8_t LCDMode[] PROGMEM = "MODE: \0";
const uint8_t LCDVolt[] PROGMEM = "VOLTMETER \0";
const uint8_t LCDOhm[] PROGMEM = "OHMMETER\0";
const uint8_t LCDFreq[] PROGMEM = "FREQUENCY\0";
const uint8_t LCDManWelcomeTop[] PROGMEM = "HELP MENU\0";
const uint8_t LCDManWelcomeBot[] PROGMEM = "PRESS ANY KEY\0";
const uint8_t LCDNavigation1Top[] PROGMEM = "PRESS # FOR NEXT\0";
const uint8_t LCDNavigation1Bot[] PROGMEM = "PRESS * FOR PREV\0";
const uint8_t LCDNavigation2Top[] PROGMEM = "PRESS 0 TO EXIT\0";
const uint8_t LCDNavigation2Bot[] PROGMEM = "THE HELP MENU\0";
const uint8_t LCDVoltManTop[] PROGMEM = "A FOR VOLTMETER\0";
const uint8_t LCDVoltManBot[] PROGMEM = "VOLTMETER\0";
const uint8_t LCDOhmManTop[] PROGMEM = "B FOR OHMMETER\0";
const uint8_t LCDOhmManBot[] PROGMEM = "OHMMETER\0";
const uint8_t LCDFreqManTop[] PROGMEM = "C FOR FREQUENCY\0";
const uint8_t LCDFreqManBot[] PROGMEM = "FREQUENCIES\0";
const uint8_t LCDAutorangeMan1Top[] PROGMEM = "D TO TOGGLE\0";
const uint8_t LCDAutorangeMan1Bot[] PROGMEM = "AUTORANGE\0";
const uint8_t LCDAutorangeMan2Top[] PROGMEM = "1 TO SWITCH\0";
const uint8_t LCDAutorangeMan2Bot[] PROGMEM = "AUTORANGE VALUE\0";
const float VrefRanges[3] = {4.955, 2.56, 1.1};
const uint8_t resistorRanges[3] = {100, 10, 1};
const uint8_t frequencyRanges[2] = {10, 1};
const uint8_t TIMERAprescalars[2] = {0x01, 0x02}; //TODO: Implement
//END LCD STRINGS**************************************************************

//DECLARE VOLATILE VARIABLES***************************************************
volatile uint8_t curKey;		//current key pressed 
volatile uint8_t keyState;		//current state for Debounce State Machine
volatile uint8_t checkKey;		//key that might be pressed
volatile uint8_t prevKeyState;	//previous state for Debounce State Machine
volatile uint16_t elapsedTime;	//total time the program has been running
volatile uint8_t debounceTime;	//the time at which the program should debounce
volatile uint8_t debouncing;	//flag for whether or not to scan the ketypad
volatile uint8_t mode;			//current mode of the DMM
volatile uint8_t returnMode;	//mode to return to after exiting the manual
volatile uint8_t manPage;		//current page in the manual
volatile uint8_t autoRange;		//is the DMM in autorange mode
volatile uint8_t justSwitched;	//did the DMM just switch states
volatile uint8_t rangeIdx;		//which range is currently selected (not AR)
volatile uint8_t rangeIdxMod;	//limits rangeIdx to 2 or 3
volatile uint8_t triggerPoll;	// set to true to poll the inputs
volatile char Ain;//, AinLow;	//The voltage to measure
volatile float Vref;			//The reference voltage
volatile uint8_t ohmRef;			//The reference resistor
volatile uint16_t T1Capture;		//Used for determining frequency
volatile uint16_t lastT1Capture;	//Used for determining frequency
volatile uint16_t period;		//Period of the waveform measured
//END VOLATILE VARIABLES*******************************************************

//UART STUFF*******************************************************************
// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
//END UART STUFF***************************************************************

//DECLARE VARIABLES************************************************************
unsigned char keytbl[MAX_KEYS]={0xee, 0xed, 0xeb, 
								0xde, 0xdd, 0xdb, 
								0xbe, 0xbd, 0xbb, 
								0xe7, 0xd7, 0xb7,
								0x77, 0x7e, 0x7b, 0x7d};

//uint8_t Ain; 		//raw A to D number
uint8_t LCDBuffer[16];  // LCD display buffer
float voltage;
float ohm;
uint16_t frequency;
char v_string[16];
//END VARIABLES****************************************************************

//DECLARE FUNCTION SIGNATURES**************************************************
void Initialize(void);
uint8_t ScanKeypad(void);
void Debounce(void);
void InitLCD(void);
void UpdateDMMState(void);
void UpdateManState(uint8_t);
void poll(void);
void pollFreq(void);
void setVref(uint8_t);
void setOref(uint8_t);
void Autorange(void);
//END FUNCTION SIGNATURES******************************************************

//TIMER INTERRUPTS*************************************************************

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
		UpdateDMMState();
	}
	else if (debouncing && !--debounceTime){
		Debounce();
		UpdateDMMState();
	}
	if(mode != INIT && mode != MAN) {
		if(elapsedTime % 200 == 0) {
			triggerPoll = 1;
		}
		if(elapsedTime % 200 == 190 && mode != FREQMETER) {
			//start another conversion
			ADCSRA |= (1<<ADSC);
		}
	}
}

ISR (TIMER1_CAPT_vect){
	//read the timer1 capture register
	T1Capture = ICR1;
	
	//period is the difference between this capture and the previous one
	period = T1Capture - lastT1Capture;
	lastT1Capture = T1Capture;

}

//END TIMER INTERRUPTS*********************************************************

//HELPER FUNCTIONS*************************************************************
uint8_t ScanKeypad(void){
	uint8_t key;
	uint8_t butnum;
	//get lower nibble
	DDRD = 0x0f;
	PORTD = 0xf0; 
	_delay_us(5);
	key = PIND;
	
	//get upper nibble
	DDRD = 0xf0;
	PORTD = 0x0f; 
	_delay_us(5);
	key = key | PIND;
	
	//find matching keycode in keytbl
	if (key != 0xff)
	begin
		for (butnum=0; butnum<MAX_KEYS; butnum++)
		begin   
		if (keytbl[butnum]==key)  break;   
		end

		if (butnum==MAX_KEYS) butnum=0;
		else butnum++;	   //adjust by one to make range 1-16
	end
	else butnum=0;
	
	return butnum;
}

//Debounce the button using a debounce state machine
void Debounce(void){
	uint8_t key = ScanKeypad(); //Scan the keypad
	switch(keyState){
	
	//in the RELEASED state: stay in this state if a key is not pressed
	//go to UNKNOWN if any key is pressed and reset the debounce countdown.
	case RELEASED:
		if (key){
			keyState = UNKNOWN;
			prevKeyState = RELEASED;
			debouncing = 1;
			debounceTime = DEBOUNCE_TIME;
			checkKey = key;
		}
		break;

	//in the UNKNOWN state: go to released if the button is not down Update
	//pressedAndReleased if the button was previously pressed.
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
				debounceTime = DEBOUNCE_TIME;
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
			debounceTime = DEBOUNCE_TIME;
		}
		else if (key != checkKey){
			keyState = UNKNOWN;
			prevKeyState = RELEASED; //...???
			debouncing = 1;
			debounceTime = DEBOUNCE_TIME;
			curKey = checkKey;
			checkKey = key;
		}
		break;
	}
}

void Initialize(void) {
	//set up the ports
	triggerPoll = 0;

	//init the A to D converter
	setVref(0); //Set to 5v Vref
	setOref(3); //Disable ohmref

	//enable ADC and set prescaler to 1/128*16MHz=125,000
	//and clear interupt enable
	//and start a conversion
	ADCSRA = (1<<ADEN) + 7;

	// Set A to input (high impedence)
	DDRA = 0x00;

	// PortB
	DDRB=0x00;
	//PORTB=0xff;

	// PortD: Keypad
	DDRD=0x00;

	//set up timer 0 for 1 mSec ticks
	TIMSK0 = 2;		//turn on timer 0 cmp match ISR
	OCR0A = 249;	//set the compare reg to 250 time ticks
	TCCR0A = 0b00000010; // turn on clear-on-match
	TCCR0B = 0b00000011;	// clock prescalar to 64

	//set up timer 1 to interrupt on capture
	//TIMSK1 = (1 << ICIE1); //turn on timer1 input capture ISR
	TCCR1B = (1 << ICES1) + 2; // Start 

	// Set analog comp to connect to timer capture input 
	// and turn on the band gap reference on the positive input  
	ACSR =  (1<<ACIC) ; //0b01000100  ;


	//initialize the current key to null
	curKey = 0;
	elapsedTime = 0;
	debounceTime = 0;

	autoRange = 1;
	mode = INIT;
	manPage = WELCOME;
	keyState = RELEASED;

	T1Capture = 0;
	lastT1Capture = 0;

	rangeIdx = 0;
	rangeIdxMod = 3;

	justSwitched = 0;

	InitLCD();

	sei();
}

uint8_t getCurKey(void){
	uint8_t tmpKey = curKey;
	if (curKey){
		curKey = 0;
	}
	return tmpKey;
}

//Initialize the LCD
void InitLCD(void){
	LCDinit();	//initialize the display
	LCDcursorOFF();
	LCDclr();				//clear the display
	LCDGotoXY(0,0);
	CopyStringtoLCD(LCDHelloTop, 0, 0);
	CopyStringtoLCD(LCDHelloBot, 0, 1);
}

//Update the DMM based on button presses
void UpdateDMMState(void){
	uint8_t key = getCurKey();
	if (key and !justSwitched){
		switch (mode){
			case INIT:
				if (key == 0x0F){
					returnMode = VOLTMETER;
					mode = MAN;
					justSwitched = 1;
				}
				else {
					if (key == 0x0A){
						mode = VOLTMETER;
						justSwitched = 1;
					}
					else if (key == 0x0B){
						mode = OHMMETER;
						rangeIdxMod = 3;
						rangeIdx = rangeIdx % rangeIdxMod;
						justSwitched = 1;
					}
					else if (key == 0x0C){
						mode = FREQMETER;
						rangeIdxMod = 2;
						rangeIdx = rangeIdx % rangeIdxMod;
						justSwitched = 1;
					}
				}
				break; 

			case MAN:
				UpdateManState(key);
				break;
			
			case VOLTMETER:
				if (key == 0x0F){
					mode = MAN;
					returnMode = VOLTMETER;
					justSwitched = 1;
				}
				else if (key == 0x0D){
					autoRange ^= 1;
					justSwitched = 1;
				}
				else if (!autoRange && key == 0x01){
					rangeIdx++;
					rangeIdx = rangeIdx % rangeIdxMod;
					justSwitched = 1;
				}
				else if (key == 0x0B){
					mode = OHMMETER;
					rangeIdxMod = 3;
					rangeIdx = rangeIdx % rangeIdxMod;
					justSwitched = 1;
				}
				else if (key == 0x0C){
					mode = FREQMETER;
					rangeIdxMod = 2;
					rangeIdx = rangeIdx % rangeIdxMod;
					justSwitched = 1;
				}
				break;

			case OHMMETER:
				if (key == 0x0F){
					returnMode = OHMMETER;
					mode = MAN;
					justSwitched = 1;
				}
				else if (key == 0x0D){
					autoRange ^= 1;
					justSwitched = 1;
				}
				else if (!autoRange && key == 0x01){
					rangeIdx++;
					rangeIdx = rangeIdx % rangeIdxMod;
					justSwitched = 1;
				}
				else if (key == 0x0A){
					mode = VOLTMETER;
					rangeIdxMod = 3;
					rangeIdx = rangeIdx % rangeIdxMod;
					justSwitched = 1;
				}
				else if (key == 0x0C){
					mode = FREQMETER;
					rangeIdxMod = 2;
					rangeIdx = rangeIdx % rangeIdxMod;
					justSwitched = 1;
				}
				break;

			case FREQMETER:
				if (key == 0x0F){
					returnMode = FREQMETER;
					mode = MAN;
					justSwitched = 1;
				}
				else if (key == 0x0D){
					autoRange ^= 1;
					justSwitched = 1;
				}
				else if (!autoRange && key == 0x01){
					rangeIdx++;
					rangeIdx = rangeIdx % rangeIdxMod;
					justSwitched = 1;
				}
				else if (key == 0x0A){
					mode = VOLTMETER;
					rangeIdxMod = 3;
					rangeIdx = rangeIdx % rangeIdxMod;
					justSwitched = 1;
				}
				else if (key == 0x0B){
					mode = OHMMETER;
					rangeIdxMod = 3;
					rangeIdx = rangeIdx % rangeIdxMod;
					justSwitched = 1;
				}
				break;
		}
	}
}

//Update the manual based on button presses
void UpdateManState(uint8_t key){
	switch (manPage){
		case WELCOME:
			if (key){
				manPage = NAVIGATION_1;
				justSwitched = 1;
			}
			break;
		case NAVIGATION_1:
			if (key == 0x10){
				mode = returnMode;
				manPage = WELCOME;
				justSwitched = 1;
			}
			else if (key == 0x0E){
				manPage = WELCOME;
				justSwitched = 1;
			}
			else if (key == 0x0F){
				manPage = NAVIGATION_2;
				justSwitched = 1;
			}
			break;
		case NAVIGATION_2:
			if (key == 0x10){
				mode = returnMode;
				manPage = WELCOME;
				justSwitched = 1;
			}
			else if (key == 0x0E){
				manPage = NAVIGATION_1;
				justSwitched = 1;
			}
			else if (key == 0x0F){
				manPage = VOLT_MAN;
				justSwitched = 1;
			}
			break;
	 	case VOLT_MAN:
			if (key == 0x10){
				mode = returnMode;
				manPage = WELCOME;
				justSwitched = 1;
			}
			else if (key == 0x0E){
				manPage = NAVIGATION_2;
				justSwitched = 1;
			}
			else if (key == 0x0F){
				manPage = OHM_MAN;
				justSwitched = 1;
			}
			break;
		case OHM_MAN:
			if (key == 0x10){
				mode = returnMode;
				manPage = WELCOME;
				justSwitched = 1;
			}
			else if (key == 0x0E){
				manPage = VOLT_MAN;
				justSwitched = 1;
			}
			else if (key == 0x0F){
				manPage = FREQ_MAN;
				justSwitched = 1;
			}
			break;
		case FREQ_MAN:
			if (key == 0x10){
				mode = returnMode;
				manPage = WELCOME;
				justSwitched = 1;
			}
			else if (key == 0x0E){
				manPage = OHM_MAN;
				justSwitched = 1;
			}
			else if (key == 0x0F){
				manPage = AUTORANGE_MAN_1;
				justSwitched = 1;
			}
			break;
		case AUTORANGE_MAN_1:
			if (key == 0x10){
				mode = returnMode;
				manPage = WELCOME;
				justSwitched = 1;
			}
			else if (key == 0x0E){
				manPage = FREQ_MAN;
				justSwitched = 1;
			}
			else if (key == 0x0F){
				manPage = AUTORANGE_MAN_2;
				justSwitched = 1;
			}
			break;
		case AUTORANGE_MAN_2:
			if (key == 0x10){
				mode = returnMode;
				manPage = WELCOME;
				justSwitched = 1;
			}
			else if (key == 0x0E){
				manPage = AUTORANGE_MAN_1;
				justSwitched = 1;
			}
			else if (key == 0x0F){
				mode = returnMode;
				manPage = WELCOME;
				justSwitched = 1;
			}
			break;
	}
}

void poll(void) {
	//get the sample
	Ain = ADCH;
	voltage = (float)Ain;
	voltage = (voltage/256.0) * Vref;
	ohm = (ohmRef * voltage)/(Vref - voltage);
	switch(mode) {
		case VOLTMETER:
			dtostrf(voltage, 6, 3, v_string);
			sprintf(LCDBuffer, "%s V",v_string);
			break;
		case OHMMETER:
			dtostrf(ohm, 6, 3, v_string);
			sprintf(LCDBuffer, "%s kOhm",v_string);
			break;
	}
	

	//CopyStringtoLCD(LCDBlank, 0, 0);
	LCDGotoXY(0, 0);
	LCDstring(LCDBuffer, strlen(LCDBuffer));
}

void pollFreq(void) {
	frequency = 2000000 / (period);
	dtostrf(frequency, 5, 0, v_string);
	sprintf(LCDBuffer, "%s Hz",v_string);

	//CopyStringtoLCD(LCDBlank, 0, 0);
	LCDGotoXY(0, 0);
	LCDstring(LCDBuffer, strlen(LCDBuffer));
}

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

void setOref(uint8_t idx) {
	// Don't use Oref
	if(idx < 0 || idx > 2) {
		DDRA = 0x00;
		return;
	}
	switch(idx) {
		case 0:
			DDRA = (1 << 7); //100k resistor output
			break;
		case 1:
			DDRA = (1 << 6); //10k resistor output
			break;
		case 2:
			DDRA = (1 << 5); //1k resistor output
			break;
	}
	ohmRef = resistorRanges[idx];
	PORTA = DDRA;
}

// Set rangeIdx if autoRange
void Autorange(void){
	if(autoRange) {
		switch (mode){
			case VOLTMETER:
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
				setOref(3); // disable ohmref
				setVref(rangeIdx);
				break;

			case OHMMETER:
				switch (rangeIdx){
					//In the 100kOhm range, move to a smaller scale if the resistance is less than 0.4 of Vcc 
					//(R_test is less than 10% of R) 
					case 0:
						if (ohm <= 10.5){
							rangeIdx++;
						}
						break;
				
					//In the 10kOhm range, move to a smaller scale if the reading is less than 0.4 of Vcc
					//(R_test is less than 10% of R). Move to a larger sclae if the reading is almost Vcc
					//(R_test is almost 95% of R).
					case 1:
						if (ohm <= 3.3){
							rangeIdx++;
						}
						else if (ohm > 10.5){
							rangeIdx--;
						}
						break;
	
					//In the 10kOhm range, move to a larger scale if the reading is almost Vcc 
					//(R_test is almost 95% of R).
					case 2:
						if (ohm > 3.3){
							rangeIdx--;
						}
					break;
				}
				setOref(rangeIdx);
				break;
		}
	}
}

//END HELPER FUNCTIONS*********************************************************

int main(void){
	Initialize();
	
	while(1){
		if(triggerPoll) {
			if(mode == FREQMETER) {
				pollFreq();
			} else {
				poll();
				Autorange();
			}
			triggerPoll = 0;
		}
		if (justSwitched){
			justSwitched = 0;
			switch (mode){
				case MAN:
					TIMSK1 = 0; //turn off timer1 input capture ISR
					switch (manPage){
						case WELCOME:
							LCDclr();
							CopyStringtoLCD(LCDManWelcomeTop, 0, 0);
							CopyStringtoLCD(LCDManWelcomeBot, 0, 1);
							break;
						case NAVIGATION_1:
							LCDclr();
							CopyStringtoLCD(LCDNavigation1Top, 0, 0);
					 		CopyStringtoLCD(LCDNavigation1Bot, 0, 1);
							break;
						case NAVIGATION_2:
							LCDclr();
							CopyStringtoLCD(LCDNavigation2Top, 0, 0);
							CopyStringtoLCD(LCDNavigation2Bot, 0, 1);
							break;
					 	case VOLT_MAN:
							LCDclr();
							CopyStringtoLCD(LCDVoltManTop, 0, 0);
							CopyStringtoLCD(LCDVoltManBot, 0, 1);
							break;
						case OHM_MAN:
							LCDclr();
							CopyStringtoLCD(LCDOhmManTop, 0, 0);
							CopyStringtoLCD(LCDOhmManBot, 0, 1);
							break;
						case FREQ_MAN:
							LCDclr();
							CopyStringtoLCD(LCDFreqManTop, 0, 0);
							CopyStringtoLCD(LCDFreqManBot, 0, 1);
							break;
						case AUTORANGE_MAN_1:
							LCDclr();
							CopyStringtoLCD(LCDAutorangeMan1Top, 0, 0);
							CopyStringtoLCD(LCDAutorangeMan1Bot, 0, 1);
							break;
						case AUTORANGE_MAN_2:
							LCDclr();
							CopyStringtoLCD(LCDAutorangeMan2Top, 0, 0);
							CopyStringtoLCD(LCDAutorangeMan2Bot, 0, 1);
							break;
					}
						break;

				case VOLTMETER:
					LCDclr();
				 	CopyStringtoLCD(LCDMode, 0, 0);
					//CopyStringtoLCD(LCDVolt, MODE_START, 0);
					TIMSK1 = 0; //turn off timer1 input capture ISR
					setOref(3); // disable ohmmeter
					if (autoRange){
						CopyStringtoLCD(LCDAutorangeOn, 0, 1);
					}
					else{
						CopyStringtoLCD(LCDRange, 0, 1);
						switch (rangeIdx){
							case 0:
								CopyStringtoLCD(LCD5Volts, RANGE_START, 1);
								break;
							case 1:
								CopyStringtoLCD(LCD256Volts, RANGE_START, 1);
								break;
							case 2:
								CopyStringtoLCD(LCD11Volts, RANGE_START, 1);
								break;
						}
						setVref(rangeIdx);
					}	
					break;

				case OHMMETER:
					LCDclr();
					CopyStringtoLCD(LCDMode, 0, 0);
					//CopyStringtoLCD(LCDOhm, MODE_START, 0);
					TIMSK1 = 0; //turn off timer1 input capture ISR
					setVref(0); // 5v reference
					if (autoRange){
						CopyStringtoLCD(LCDAutorangeOn, 0, 1);
					}
					else{
						CopyStringtoLCD(LCDRange, 0, 1);
						switch (rangeIdx){
							case 0:
								CopyStringtoLCD(LCD100kOhm, RANGE_START, 1);
								break;
							case 1:
								CopyStringtoLCD(LCD10kOhm, RANGE_START, 1);
								break;
							case 2:
								CopyStringtoLCD(LCD1kOhm, RANGE_START, 1);
								break;
						}
						setOref(rangeIdx);
					}
					break;

				case FREQMETER:
					LCDclr();
					CopyStringtoLCD(LCDMode, 0, 0);
					//CopyStringtoLCD(LCDFreq, MODE_START, 0);

					setVref(0);
					setOref(3);

					//Set the prescalar on TIMER1 to the appropriate range
					TIMSK1 = (1 << ICIE1); //turn on timer1 input capture ISR

					if (autoRange){
						CopyStringtoLCD(LCDAutorangeOn, 0, 1);
					}
					else{
						CopyStringtoLCD(LCDRange, 0, 1);
						switch (rangeIdx){
							case 0:
								CopyStringtoLCD(LCD10kHz, RANGE_START, 1);
								break;
							case 1:
								CopyStringtoLCD(LCD1kHz, RANGE_START, 1);
								break;
						}
					}
					break;
			}
		}
	}

}
