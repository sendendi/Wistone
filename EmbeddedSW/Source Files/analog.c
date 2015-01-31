/*******************************************************************************

analog.c - handle internal ADC
==============================

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
this file contains function for operating internal ADC.
- inititialize the ADC after power up
- get sample from ADC, when needed

Battey and Charging status
==========================
- we use the single LED we have to indicate both charge and battery level status.
- charging:
	- when we detect that charge is on, we blink the LED with double blinks or triple blinks according to status
	- main state machine is implemented within Timer4
- Battery status:
	- when not in charging state, the LED blinking frequency indicates the battery level
	- lower level yields higher frequency
*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include "adc.h"
#include "system.h"
#include "error.h"			//Application
#include "p24FJ256GB110.h"	//Common
#include "analog.h"			//Devices

/*******************************************************************************
// analog_init()
// in Wistone we have two optional analog sources:
// - AN12 - Microphone - not supported at this stage
// - AN10 - Battery level
//
// initialize the internal ADC:
// - prepare for sampling
*******************************************************************************/
void analog_init(void)
{
	// Configure adc controls:
	AD1CON1	= 0x0000; // Sample when SAMP=1, convert when SAMP=0	
	AD1CON2 = 0x0000; // No channel scan, always use MUX-A
	AD1CON3 = 0x0A0A; // No timers set
	AD1PCFGL = 0xFBFF;	// set all pins to digital, except fot AN10:
	AD1CON1bits.ADON = 1; // turn ADC ON
}

/*******************************************************************************
// detect_analog_input()
// - Battery Level:
//   - read battery level value from ADC
//   - convert it to 10bits number
//   - then store in global variable: g_vbat_level
// input:
// - adc_stage - SAMPLE_STAGE, CONVERT_STAGE, UPDATE_STAGE
// output:
// global:
// - g_vbt_level - representing the value sampled at the voltage divider of the battery on Power Board 
// return:
// - 0 - on success
// - 1 - on error
/*******************************************************************************/
int detect_analog_input(int adc_stage) {
	static unsigned int	last_vbat_level = 500;

	//set ADC parameters
	AD1CHS  = 0x0A0A; // select the channel AN10 for both MUX-A and MUX-B. select Vr- (= GND in our case) for the negative reference
	// this section determines what action to take according to entry number
	switch (adc_stage) {
	case SAMPLE_STAGE:						// first enrty, sample
		AD1CON1bits.SAMP = 1;
		break;
	case CONVERT_STAGE:						// second enrty, convert
		AD1CON1bits.SAMP = 0; 	
		break;
	case UPDATE_STAGE:						// third enrty, global variables update
			// read the ADC value value:
			g_vbat_level = (unsigned int)ADC1BUF0;
			// "low pass filter" the value, to avoid changing due to momentary changes (alpha factor):
			g_vbat_level = ((g_vbat_level * (unsigned int)4) + (last_vbat_level * (unsigned int)6)) / (unsigned int)10;
			last_vbat_level = g_vbat_level;
		break;
	default:
		return(1);
	}

	return (0);
}
