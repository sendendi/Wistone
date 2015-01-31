/*******************************************************************************

led_buzzer.c - handle buzzer and LED
====================================

	Revision History:
	=================
 ver 1.00, date: 1.10.11
		- Initial revision
 ver 1.01, date: 29.10.11
		- joint work
 ver 1.02 date: 8.8.12
		- overall joint code review
 ver 1.03 date: 21.9.12
		- BOOT table		
		
********************************************************************************
	General:
	========
the Wistone is equiped with LED, switches and buzzer for debug.
the following functions:
- initiate Timer4 for LED, Buzzer and switch maintanence
- create drivers for: LED, Switch and Buzzer
*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include "timer.h"
#include "analog.h"
#include "system.h"
#include "error.h"				//Application
#include "wistone_main.h"		//Application
#include "HardwareProfile.h"	//Common
#include "p24FJ256GB110.h"		//Common
#include "ads1282.h"			//Devices
#include "led_buzzer.h"			//Devices

/***** GLOBAL VARIABLES: ******************************************************/
int g_buzz_period = 0; 			// in order to play buzzer sound at 1Khz for some period (in mSec units) set this global variable to the period

/***** DEFINES: ***************************************************************/
#define ADS1282_FREQ	400		// external ADC desired sample frequency [Hz]
// assume we have Fcy = 16Mhz
// use Timer pre-scaler of 1:256
// period x 256 = 16000000 / 400
// => period = 16M / (256 x 400) = 1M / (16x400) = 64K / 400 = 16K / 100 = 160
#define TIMER_4_MASK 	0xA030
//#define TIMER_4_PERIOD	320		// we 2.5ms for ADC
#define TIMER_4_PERIOD	((16000000 / 256) / ADS1282_FREQ)

/*******************************************************************************
// init_timer4()
// this function initiates Timer4, used later for:
// - power LED blink
// - buzzer frequency generation
// - switch state scan
// - generate SYNC pulse for ADC ADS1282
*******************************************************************************/
void init_timer4(void)
{
	//Config Timer 4: Module On | prescaler | internal clock source (= Fosc / 2) | period match value to be stored in PR register
	//OpenTimer4(T4_ON | T4_PS_1_1 | T4_SOURCE_INT, 16600);	 
	OpenTimer4(TIMER_4_MASK, TIMER_4_PERIOD); 
	//Timer Interrupt Enable:	
	//ConfigIntTimer4(T4_INT_ON);
	// just in case:
	IFS1bits.T4IF = 0;  //YL 15.8
	IEC1bits.T4IE = 1;
}

/*******************************************************************************
// _T4Interrupt()
// interrupt service routine (ISR) for handling Timer4 event.
// it is configured to occur every 2.5mSec (400Hz)
// - generate SYNC for ADC, 
// - if buzzer period is on, then toggle buzzer output (yield 400/2 Hz sound)
// - according to mSec counter, toggle power LED indication
// - check if power switch is pressed
*******************************************************************************/
void __attribute__((__interrupt__, auto_psv, __shadow__)) _T4Interrupt(void)
{
	static unsigned int 	timer4_tick_counter = 0;
	static int 				power_switch_msec_counter = 0;
	static unsigned int     adc_stage = SAMPLE_STAGE;
	static unsigned long	led_cycle_time = 1600;
	
	//===============
	// ADS1282 SYNC
	//===============
	// when ADS1282 is active, generate 500nSec width pulse at SYNC output:
	if (g_is_ads1282_active == TRUE) {
		ADS1282_SYNC_LAT = 1;
		// minimal SYNC pulse width is 2Tclk = 500nSec
		// Fcy = 16Mhz, every nop takes 2 cycles => 0.125uSec => need 4 nops
		Nop();
		Nop();
		Nop();
		Nop();
		Nop();
		ADS1282_SYNC_LAT = 0;	
	}

	//===============
	// BUZZER
	//===============
	// in case of g_buzzer_period > 0, toggle BUZZER_PORT
	if (g_buzz_period) {
		g_buzz_period--;
		BUZZER_PORT = !BUZZER_PORT;
	}

	//===============
	// LED BLINK
	//===============
	// implement number of consequtive blinks according to POWER_STAT1,2 state:
	// charge state					STAT1n	STAT2n  #of blinks
	//   precharge					0		0		1
	//	 fast charge				0		1		2
	//   charge done				1		0		3
	//	 charge suspend				1		1		4
	// implement PWM (Tccle) according to vbat level: keep on-time very short to save battery power
	led_cycle_time--;
	if (led_cycle_time == (ADS1282_FREQ / 16) * 8) {
		if ((PWR_CHRG_STAT1 & PWR_CHRG_STAT2) & (~PWR_CHRG_USBPG)) {
			LED_TRIS_2 = 0;
			LED_PORT_2 = 1;					// turn LED on 
		}
	}
	if (led_cycle_time == (ADS1282_FREQ / 16) * 7) {
		LED_TRIS_2 = 0;
		LED_PORT_2 = 0;				// turn LED off 
	}
	if (led_cycle_time == (ADS1282_FREQ / 16) * 6) {
		if ((PWR_CHRG_STAT1) & (~PWR_CHRG_USBPG)) {
			LED_TRIS_2 = 0;
			LED_PORT_2 = 1;					// turn LED on 
		}
	}
	if (led_cycle_time == (ADS1282_FREQ / 16) * 5) {
		LED_TRIS_2 = 0;
		LED_PORT_2 = 0;				// turn LED off 
	}
	if (led_cycle_time == (ADS1282_FREQ / 16) * 4) {
		if (((PWR_CHRG_STAT1) | ((~PWR_CHRG_STAT1) & PWR_CHRG_STAT2)) & (~PWR_CHRG_USBPG)) {
			LED_TRIS_2 = 0;
			LED_PORT_2 = 1;					// turn LED on 
		}
	}
	if (led_cycle_time == (ADS1282_FREQ / 16) * 3) {
		LED_TRIS_2 = 0;
		LED_PORT_2 = 0;				// turn LED off 
	}
	if (led_cycle_time == (ADS1282_FREQ / 16) * 2) {
		LED_TRIS_2 = 0;
		LED_PORT_2 = 1;					// turn LED on 
	}
	if (led_cycle_time == (ADS1282_FREQ / 16) * 1) {
		LED_TRIS_2 = 0;
		LED_PORT_2 = 0;				// turn LED off 
		// period between LED turning on times: 2~20 seconds depends on battery level
		if (g_vbat_level > VBAT_STAT_MAX)
			led_cycle_time = 20 * ADS1282_FREQ;	// 20 Sec
		if (g_vbat_level > VBAT_STAT_MIN)
			led_cycle_time = (unsigned long)((((long)g_vbat_level - (long)VBAT_STAT_MIN) * (long)20) / (long)(VBAT_STAT_MAX - VBAT_STAT_MIN)) * ADS1282_FREQ + (2 * ADS1282_FREQ);
		else
			led_cycle_time =  2 * ADS1282_FREQ + 1;	// 2 Sec
	}
	
	//===============
	// POWER SWITCH
	//===============
	//YS 17.8
	// make sure we check power switch every 1K mSec //YL 16.9 and RTC timer interrupt bit:
	timer4_tick_counter++; 				// count timer4 occurences modulo 64K (unsigned int) 
	if ((timer4_tick_counter & 0x01FF) == 0x0000) {
		// read power switch status
		// keep in mind this is switch#1
		if (!get_switch(SWITCH_1) && !power_switch_msec_counter) // if switch is pressed it returns "0" //YS 17.8 - make sure we don't reset the counter
			// set down time counter to 2000 mSec
			power_switch_msec_counter = 100; //YS 17.8
	}
	// in case power_switch_msec_counter was activated:
	if (power_switch_msec_counter) {
		power_switch_msec_counter--;
		if (get_switch(SWITCH_1)) // if switch is released it returns "1"
			power_switch_msec_counter = 0;
		// check if counter managed to reach 1
		if (power_switch_msec_counter == 1) { //YL 15.8
				// raise flag indicating turn off request:
				g_sleep_request = 1; //YL 15.8
			}
	}

	//===============
	// VBAT SAMPLING
	//===============
	// handle analog sampling of Battery Voltage Level:
	// sparse entry to sampling procedure, to reduce load from Timer function
	// perform a single sample stage every ~400mSec
	if ((timer4_tick_counter & 0x00FF) == 0x0000) {
		detect_analog_input(adc_stage);
		// advance the state machine one step:
		switch (adc_stage) {
			case SAMPLE_STAGE:
				adc_stage = CONVERT_STAGE;
				break;
			case CONVERT_STAGE: 
				adc_stage = UPDATE_STAGE;
				break;
			case UPDATE_STAGE: 
				adc_stage = SAMPLE_STAGE;
				break;
		}
	}

	IFS1bits.T4IF = 0; // reset Timer4 interrupt flag and Return from ISR
}

/*******************************************************************************
// init_buzzer()
// this function initializes the Buzzer GPIO, and turn off buzzer
*******************************************************************************/
void init_buzzer(void)
{
    BUZZER_TRIS = 0;  			// output
	BUZZER_PORT = 0;
	g_buzz_period = 0;			// turn buzzer off
}

/*******************************************************************************
// init_leds()
// this function initializes the LEDs GPIOs as inputs, to save power
*******************************************************************************/
void init_leds(void)
{
    LED_TRIS_1 = 1;  			// input
    LED_TRIS_2 = 1;  			// input
}

/*******************************************************************************
// play_buzzer()
// play the 1Khz on the buzzer.
// set the desired period (in units of Sec).
// verify that period is not negative or over 10 Sec
*******************************************************************************/
int play_buzzer(int period)
{
	if ((period < 0) || (period > 10))
		return err(ERR_INVALID_BUZZ_PERIOD);
	// turn on buzzer counter (for Timer4 ISR), convert from Sec to mSec;
	g_buzz_period = period * 1000;
	
	return 0;
}

/*******************************************************************************
// set_led()
// set new value for the requested LED
// - state: "1" turn on, "0" turn off
// - led_num: 1 - "power indication LED", 2 - the other LED
*******************************************************************************/
int set_led(int state, int led_num)
{
	if ((state < 0) || (state > 1))
		return err(ERR_INVALID_LED_STATE);
	switch (led_num) {
		case LED_1:
			LED_TRIS_1 = 0;		//output
			LED_PORT_1 = state;		
		break;
		case LED_2:
			LED_TRIS_2 = 0;		//output
			LED_PORT_2 = state;	
	}
	
	return 0;
}

/*******************************************************************************
// get_switch():
// get status of the SWITCH
// - switch_num: 1 - "power off" switch, 2 - the other switch
// since LED is muxed with switch, we save the LED state ahead, and reconstruct it later
*******************************************************************************/
int get_switch(int switch_num)
{
	int switch_state;
	int saved_led_tris;
	int saved_led_port;
	
	switch (switch_num) {
	case SWITCH_1:
		saved_led_tris = LED_TRIS_1;
		saved_led_port = LED_PORT_1;
		SWITCH_TRIS_1 = 1;		// set direction to input
		//YL 22.12 added Nops (for delay to make the power-switch-off-button press always effective) - but it isn't yet...:
		Nop();
		Nop();
		switch_state = SWITCH_PORT_1;		
		LED_TRIS_1 = saved_led_tris;
		LED_PORT_1 = saved_led_port;
		break;
	case SWITCH_2:
		saved_led_tris = LED_TRIS_2;
		saved_led_port = LED_PORT_2;
		SWITCH_TRIS_2 = 1;		// set direction to input
		//YL 22.12 added Nops (for delay to make the power-switch-off-button press always effective) - but it isn't yet...:
		Nop();
		Nop();
		switch_state = SWITCH_PORT_2;		
		LED_TRIS_2 = saved_led_tris;
		LED_PORT_2 = saved_led_port;
		break;
	}
	
	return switch_state;
}
