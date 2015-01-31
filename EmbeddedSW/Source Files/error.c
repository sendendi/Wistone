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
#include "command.h"		// Application
#include "error.h"			// Application
	
/***** GLOBAL VARIABLES: ******************************************************/
ErrType g_error; 			// global variable holding the error number

/*******************************************************************************
YL 5.8 editted the description:
* Table: 
*		g_err_messages:
* Description:
*		- holds all possible error messages
*		- must be in the same order as the enum ErrType
*******************************************************************************/
static char *g_err_messages[] = {
	"No Error",
	"Unknown Error",
	"Unknown Command",
	"Unknown Sub Command Name",
	"Invalid Parameter(s)",
	"Invalid Number of Parameters",
	"Number Too Big",	
	"Invalid Number",
	"Invalid LED State. Must be 0 or 1",
	"Invalid Buzzer Period. Must be 0 to 100",
	
	"I2C Bus is Busy or Not Responding",
	"I2C Device Write Failed",
	"I2C Device Read Failed",
	
	"EEPROM Boot Set Failed", 		
	"EEPROM Boot Get Failed", 		
	"EEPROM Write Byte Failed",
	"EEPROM Read Byte Failed",
	"EEPROM Write N Bytes Failed",	
	"EEPROM Read N Bytes Failed",	
	"EEPROM Format Failed",
	
	"Temp Init Error",						
	"Temp Read Error",	

	"RTC Init Error",
	"RTC Get Time Error",
	"RTC Invalid Time",		
	"RTC Invalid Date",		
	"RTC Invalid Wakeup", 		
	"RTC Invalid Alarm Set",
 	
	"SD Init Error", 
	"SD Card Detect Error",	
	"SD Write Error",		
	"SD Read Error",					
	"Invalid Device",
	
	"Invalid Mode",			
	"Invalid Communication",			// YL 5.8 communication instead of destination 
	"Invalid Sampler",
	
	"ACCMTR Unknown ID",		
	"ACCMTR Register Write Failed", 		
	"ACCMTR Register Read Failed", 		
	"ADS1282 Unknown ID",
	"ADS1282 Register Read Failed",
	"ADS1282 Missing Power: 5v, 12v, -12v",			
	
	// renamed and moved to TxRx: "NWK Unknown Destination",			// YL 4.8 invalid network address of the destination
	// renamed and moved to TxRx: "NWK Not Me",						// YL 23.7 
	""
};

/*******************************************************************************
* Function:
*		get_last_error_str()
* Description:
* 		Return the error description string representing the given error code
*******************************************************************************/
char *get_last_error_str(void) {

	return g_err_messages[g_error];
}

/*******************************************************************************
* Function:
*		err()
* Description:
* 		Write error message
* in case of multiple error occured for the same current message,
* make sure only first identified error is stored into g_error,
* and ignore the following errors found for the current message.
* Side effects:
*		- updates global g_error
*******************************************************************************/
int err(ErrType err) {

	// only first error matters: //YL 8.8 <- changed this to display all the errors
	if (!g_error) {
		g_error = err;
		m_write(g_err_messages[g_error]);
		write_eol();
		err_clear();	//YL 8.8 to reset g_error after error print
	}
	return -1;
}

/*******************************************************************************
* Function:
*		err_clear()
* Description:
* 		Mark that there is no error.
* this function is called before start handling new message, 
* to start with no error as default.
* Side effects:
*		- updates global g_error
*******************************************************************************/
void err_clear(void) {
	
	g_error = ERR_NONE;
}
