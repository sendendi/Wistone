/*******************************************************************************

error.c - handle error events and messages
==========================================

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
this file contains function for handling errors.
- each possible error has a specific ID number and description string
- g_error is a global variable holding the last error number
*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include "command.h"		//Application
#include "error.h"			//Application
	
/***** GLOBAL VARIABLES: ******************************************************/

ErrType g_error; // global variable holding the error number

// the err messages must reflect the ErrType enum from error.h exactly!
static char *g_err_messages[] = {
	"No Error",
	"Unknown Error",
	"Unknown command",
	"Unknown sub command name",
	"Invalid parameter(s)",
	"Parameter out of range",
	"I2C bus is busy or not responding",
	"Device write transaction failed",
	"Device read transaction failed",
	"EEPROM boot set failed", 		//YL 17.9
	"EEPROM boot get failed", 		//YL 17.9
	"EEPROM write byte failed",
	"EEPROM read byte failed",
	"EEPROM write n bytes failed",	//YL 17.9
	"EEPROM read n bytes failed",	//YL 17.9
	"EEPROM format failed",
	"Temp init error",						
	"Temp read error",
	"Invalid number of parameters",
	"Number too big",	
	"Invalid number",	
	"Invalid LED state. Must be 0 or 1",
	"Invalid buzzer period. Must be 0 to 100",
	"RTC Init error",
	"RTC Get Time error",
	"RTC Invalid time",		
	"RTC Invalid date",		
	"RTC Invalid wakeup", 		//YL 15.9
	"RTC Invalid alarm set", 	//YL 15.9
	"SD init error", 
	"SD card detect error",	
	"SD write error",		
	"SD read error",		
	//"Invalid ADC channel", 	//YL 7.11	isn't used			
	"Invalid device",	
	"Invalid mode",			
	"Invalid destination",
	"Invalid sampler",			//YL 7.11
	"ACCMTR unknown ID",		//YL 10.8
	"ACCMTR register write failed", 	//TODO 15.8 ineffective	
	"ACCMTR register read failed", 		//TODO 15.8 ineffective	
	"ADS1282 unknown ID",			//YL 29.11
	"ADS1282 mossing power: 5v, 12v, -12v",			//YL 29.11
	""
};

/*******************************************************************************
// get_last_error_str()
// return the error description string representing the given error code
*******************************************************************************/
char *get_last_error_str(void)
{
	return g_err_messages[g_error];
}

/*******************************************************************************
// err()
// in case of multiple error occured for the same current message,
// make sure only first identified error is stored into g_error,
// and ignore the following errors found for the current message.
*******************************************************************************/
int err(ErrType err)
{
	// only first error catches
	if (!g_error) {
		g_error = err;
		m_write(g_err_messages[g_error]);	
		write_eol();
	}
	return -1;
}

/*******************************************************************************
// err_clear()
// mark that there is no error.
// this function is called before start handling new message, to start 
// with no error as default.
*******************************************************************************/
void err_clear(void)
{
	g_error = ERR_NONE;
}
