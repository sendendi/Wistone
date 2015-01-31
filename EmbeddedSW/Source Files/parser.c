/*******************************************************************************

parser.c - parse the input message
==================================

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
following are the functions for parsing incoming message.
*******************************************************************************/
#include "wistone_main.h"
//YL 6.8 moved to envelop only handle_msg/read/write #ifdef WISDOM_STONE

// YL 5.8 edited functions and DS descriptions

/***** INCLUDE FILES: *********************************************************/
#include <string.h>
#include "app.h"						// Application
#include "command.h"					// Application
#include "error.h"						// Application
#include "parser.h"						// Application
#include "system.h"						// Application
#include "HardwareProfile.h"			// Common
#include "misc_c.h"						// Common		// YL 27.8
#include "accelerometer.h"				// Devices
#include "ads1282.h"					// Devices
#include "analog.h"						// Devices
#include "eeprom.h"						// Devices
#include "flash.h"						// Devices
#include "lcd.h"						// Devices
#include "led_buzzer.h"					// Devices
#include "rtc.h"						// Devices			
#include "temp.h"						// Devices
#include "i2c.h"						// Protocols	// YL 30.8
//YL 4.8 #include "MiWi.h"				// TxRx			// YL 23.7 for myLongAddress (the address of the stone in the network)
#include "TxRx.h"						// TxRx			// YL 4.8 for isBroadcast

/***** GLOBAL VARIABLES: ******************************************************/

/*******************************************************************************
* Table: 
*		g_cmd_names:
* Description:
*		- holds all possible COMMANDs
*		- must be in the same order as the enum CmdTypes
*******************************************************************************/
char *g_cmd_names[] = {
	"w",
	"r",
	"eeprom",
	"rtc",
	"temp",
	"lcd",
	"sys",
	"flash",
	"ads",
	"accmtr",	// YL 15.9
	"app"
};

/*******************************************************************************
* Table: 
*		g_sub_cmd_names:
* Description:
*		- holds all possible SUB COMMANDs
*		- must be in the same order as the enum SubCmdTypes
*******************************************************************************/
char *g_sub_cmd_names[] = {
	"led",					
	"buzzer",
	"switch",
	"eeprom",
	"rtc",
	"temp",
	"flash",
	"lcd",
	"accmtr",
	"sboot",	// YL 17.9
	"gboot",	// YL 17.9
	"write",	// YL 17.9 EEPROM read/write N bytes
	"read",		// YL 17.9
	"format",
	"gver",
	"sid",
	"gpower",
	"spower", 
	"gerr",
	"stime",
	"sdate",
	"gtime",
	"gdate",
	"swakeup",	// YL 15.9
	"salarm",	// YL 15.9
	"gtemp",
	"clrscr",
	"start",
	"stop",
	"sleep",
	"shutdown",
	"config",	// YL 15.9 accelerometer
	"gcd",
	"minit",
	"gcap",			
	"wsector",		
	"rsector"		
};

/*******************************************************************************
* Table: 
*		g_dev_names:
* Description:
*		- holds all possible devices
*		- must be in the same order as the enum DevTypes
*******************************************************************************/
char *g_dev_names[] = {
	"led",
	"buzzer",
	"switch",
	"eeprom",
	"rtc",
	"temp",
	"flash",
	"lcd",
	"adc",
	"accmtr"	// YL 15.9 accelerometer, instead of sensor
};

/*******************************************************************************
* Table: 
*		g_mode_names:
* Description:
*		- holds all possible application modes
*		- must be in the same order as the enum ModeTypes
*******************************************************************************/
char* g_mode_names[] = {	
	"ss",
	"ts",
	"ost",
	"" 		// no "idle" cmd mode parameter  
};

/*******************************************************************************
* Table: 
*		g_comm_names:
* Description:
*		- holds all possible communication types
*		- must be in the same order as the enum CommTypes
*******************************************************************************/
char* g_comm_names[] = {   
	"usb",
	"wireless",
	"none"		 
};

/*******************************************************************************
* Table: 
*		g_samp_names:
* Description:
*		- holds all possible sampler types
*		- must be in the same order as the enum SampTypes
*******************************************************************************/
char* g_samp_names[] = {   
	"dual",
	"single"
};

/*******************************************************************************
* Table: 
*		default_addr:
* Description:
*		- holds default address for each device 
*		- the default will be used in case the <address> token is missing in 
*			low level WRITE/READ commands.
*		- when we don't allow missing address (e.g. in EEPROM), we assign it "-1".
*******************************************************************************/
long default_addr[] = {
	0,  // DEV_LED
	0,  // DEV_BUZZER
	0,  // DEV_SWITCH
	0, 	// DEV_EEPROM
	0,	// DEV_RTC
	//0,	// DEV_ADC // YL 8.12 isn't used
	0, 	// DEV_TEMP
	0, 	// DEV_FLASH
	0,	// DEV_LCD
	0	// DEV_ACCMTR
};

/*******************************************************************************
* Table: 
*		default_data:
* Description:
*		- holds default data field for each device 
*		- the default will be used in case the <data> token is missing in low level WRITE commands.
*		- when we don't allow missing data (e.g. in EEPROM), we assign it "-1".
*******************************************************************************/
long default_data[] = {
	0,  // DEV_LED
	0,  // DEV_BUZZER
	-1, // DEV_SWITCH
	-1, // DEV_EEPROM
	-1,	// DEV_RTC
	//-1,	// DEV_ADC // YL 8.12 isn't used
	-1, // DEV_TEMP
	-1, // DEV_FLASH
	-1,	// DEV_LCD
	-1	// DEV_ACCMTR
};

/***** INTERNAL PROTOTYPES: ***************************************************/
char*	skip_sep(char *p);
char*	skip_token(char *p);

/*******************************************************************************
* Function: 	
*		tokenize()
* Description:
*		Split command into parsable tokens.
* Parameters:
*		msg - input message to split into tokens.
*				the tokens are stored in a global g_tokens,
*				and the total number of the tokens is stored in a global g_ntokens  
* Return value:
*		None
* Side effects:
*		- Updates global variables: g_tokens and g_ntokens
*******************************************************************************/
void tokenize(char *msg) {

	static char	msg_copy[MAX_CMD_LEN];	// YL 20.9 added copy, so g_in_msg remain unchanged; 
	strcpy(msg_copy, msg);
	char *p = msg_copy;

	g_ntokens = 0;
	while (1) {
		p = skip_sep(p);
		if (!*p || (*p == '\r')) {
			*p = 0;
			return;
		}
		g_tokens[g_ntokens] = p;
		g_ntokens++;
		p = skip_token(p);
		if (!*p || (*p == '\r')) {
			*p = 0;
			return;
		}
		*p = 0;
		p++;
	}
	return;
}

/*******************************************************************************
* Function: 	
*		skip_sep()
* Description:
*		Skip delimiters (' ', '\t', ',') between the tokens inside the message.
* Parameters:
*		p - a pointer to a string that needs to skip the delimiters 
* Return value:
*		p - a pointer to a string after it skipped the delimiters 
*******************************************************************************/
char *skip_sep(char *p) {

	while ((*p == ' ') || (*p == '\t') || (*p == ',')) p++;
	return p;
}

/*******************************************************************************
* Function: 	
*		skip_token()
* Description:
*		Skip tokens to a nearest delimiter inside the message (' ', '\t', ',')
* Parameters:
*		p - a pointer to a string that needs to skip the tokens 
* Return value:
*		p - a pointer to a string after it skipped the tokens 
*******************************************************************************/
char *skip_token(char *p) {

	while (*p) {
		if ((*p == ' ') || (*p == '\t') || (*p == ','))
			return p;
		p++;
	}	
	return p;
}

/*******************************************************************************
* Function: 	
*		parse_long_num()
* Description:
*		Parse string to long number of 32 bits.
* Parameters:
*		str - input string.
*		Input string is translated into valid output number only
*		if it represents some value from [0 : (2^31 - 1)]
* 		(i.e positive half of signed long range [-2^31 : (2^31 - 1)]).
* Return value:
*		res:
*			- in case of valid input, res is a number from [0 : (2^31 - 1)].
*			- otherwise, res is (-1), and error string is printed.
* Side effects:
*		None
*******************************************************************************/
long parse_long_num(char *str) {

	long 	res = 0;
	long 	max_res = MAX_SIGNED_LONG / 10; 
	char 	*p = str;

	while ((*p >= '0') && (*p <= '9')) {			
		if ((res > max_res) || ((res == max_res) && (*p > '7'))) 
			return err(ERR_NUM_TOO_BIG);
		res = res * 10 + *p - '0';
		p++;
	}
	if (p == str)
		return err(ERR_INVALID_NUM);
	return res;
}

/*******************************************************************************
* Function: 	
*		parse_int_num()
* Description:
*		Parse string to int number of 16 bits.
* Parameters:
*		str - input string.
*		Input string is translated into valid output number only
*		if it represents some value from [0 : (2^15 - 1)]
* 		(i.e positive half of signed int range [-2^15 : (2^15 - 1)]).
* Return value:
*		- in case of valid input, a number from [0 : (2^15 - 1)].
*		- otherwise, (-1) is returned, and error string is printed.
* Side effects:
*		None
*******************************************************************************/
int parse_int_num(char *str) { //YL 5.8 rewritten

	// YL 5.8 was: return (0xFFFF & parse_long_num(str));
	
	long res = parse_long_num(str);
	
	// input str represents invalid long 
	// (too big - ERR_NUM_TOO_BIG, or negative - ERR_INVALID_NUM),
	// and therefore it is also invalid int:
	if (res == (-1)) 
		return (-1);
	
	// input str is bigger than max signed int:
	else if (res > MAX_SIGNED_INT)
		return err(ERR_INVALID_NUM);
	
	// return 16 bit int num:
	return (0xFFFF & res);
}

/*******************************************************************************
* Function: 	
*		parse_byte_num()
* Description:
*		Parse string to byte number of 8 bits.
* Parameters:
*		str - input string.
*		Input string is translated into valid output number only
*		if it represents some value from [0 : (2^8 - 1)]
* 		(i.e unsigned char range [0 : (2^8 - 1)]).
* Return value:
*		- in case of valid input, a number from [0 : (2^8 - 1)].
*		- otherwise, (-1) is returned, and error string is printed.
* Side effects:
*		None
* Note:
*		We return (-1) to indicate receiving invalid input,
*		therefore - the return-type (which is int) need to be casted to BYTE 
*******************************************************************************/
int parse_byte_num(char *str) { //YL 5.8 rewritten
	
	int res = parse_int_num(str);
	
	// input str represents invalid int
	// (too big - ERR_NUM_TOO_BIG, or negative - ERR_INVALID_NUM),
	// and therefore it is also invalid byte:
	if (res == (-1))
		return (-1);	
	
	// input str is bigger than max byte:
	else if (res > MAX_BYTE)
		return err(ERR_INVALID_NUM);
	
	// return 8 bit byte num:
	return (0xFF & res);
}

/*******************************************************************************
* Function: 	
*		parse_name()
* Description:
*		Find a given string inside a list of strings.
* Parameters:
*		name - the string we are looking for
*		namelist - the list of the names
*		listlen - the length of the list
* Return value:
*		i - the index of the string in the list if found, (-1) if not
* Side effects:
*		None
*******************************************************************************/
int parse_name(char *name, char **namelist, int listlen) {

	char 	*p1, *p2;
	int 	i;

	for (i = 0; i < listlen; i++) {
		p1 = name;
		p2 = namelist[i];
		while (*p2) {
			if (*p1 != *p2) break;
			p1++;
			p2++;
		}
		if (!*p2 && !*p1)
			return i;
	}
	return -1;
}

/*******************************************************************************
* Function: 	
*		parse_command()
* Description:
*		Look for the current command in the list of possible commands.
* Parameters:
*		name - the string we are looking for
* Return value:
*		the index of the string in the list if found, (-1) if not
* Side effects:
*		None
*******************************************************************************/
int parse_command(char *name) {

	return parse_name(name, g_cmd_names, N_COMMANDS);
}

/*******************************************************************************
* Function: 	
*		parse_sub_command()
* Description:
*		Look for the current sub command in the list of possible sub commands.
* Parameters:
*		name - the string we are looking for
* Return value:
*		the index of the string in the list if found, (-1) if not
* Side effects:
*		None
*******************************************************************************/
int parse_sub_command(char *name) {

	return parse_name(name, g_sub_cmd_names, N_SUB_COMMANDS);
}

/*******************************************************************************
* Function: 	
*		parse_device()
* Description:
*		Look for the current device name in the list of possible devices.
* Parameters:
*		name - the string we are looking for
* Return value:
*		the index of the string in the list if found, (-1) if not
* Side effects:
*		None
*******************************************************************************/
int parse_device(char *name) {

	int res = parse_name(name, g_dev_names, N_DEVICES);

	if (res < 0) {
		err_clear(); // make sure following error superseded any prev one
		return err(ERR_INVALID_DEVICE);
	}
	return res;
}

/*******************************************************************************
* Function: 	
*		parse_mode()
* Description:
*		Look for the current mode name in the list of possible app modes.
* Parameters:
*		name - the string we are looking for
* Return value:
*		the index of the string in the list if found, (-1) if not
* Side effects:
*		None
*******************************************************************************/
int parse_mode(char *name) {

	int res = parse_name(name, g_mode_names, N_MODES);

	if (res < 0) {
		err_clear(); // make sure following error superseded any prev one
		err(ERR_INVALID_MODE);
	}
	return res;
}

/*******************************************************************************
* Function: 	
*		parse_communication()
* Description:
*		look for the current communication type in the list of possible communication types 
* Parameters:
*		name - the string we are looking for
* Return value:
*		the index of the string in the list if found, (-1) if not
* Side effects:
*		None
*******************************************************************************/
int parse_communication(char *name) {

	int res = parse_name(name, g_comm_names, N_COMMUNICATIONS);

	if (res < 0) {
		err_clear(); // make sure following error superseded any prev one
		err(ERR_INVALID_COMM);
	}
	return res;
}

/*******************************************************************************
* Function: 	
*		parse_sampler()
* Description:
*		Look for the current sampler in the list of possible app samplers.
* Parameters:
*		name - the string we are looking for
* Return value:
*		the index of the string in the list if found, (-1) if not
* Side effects:
*		None
*******************************************************************************/
int parse_single_dual_mode(char *name) {

	int res = parse_name(name, g_samp_names, N_SAMPLERS);

	if (res < 0) {
		err_clear(); // make sure following error superseded any prev one
		err(ERR_INVALID_SAMP);
	}
	return res;
}

//YL 4.8 added handle_plug_msg...

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
#if defined COMMUNICATION_PLUG
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/*******************************************************************************
* Function: 	
*		handle_plug_msg()
* Description:
*		Parse the message that USB sent to the plug:
*		- performs some checks of input validity
*		- parses global g_curr_msg and removes the prefix with 
*			network address of the destination from g_curr_msg
* Parameters:
*		None 
* Return value:
*		dest:
*			- if the input is valid - the network address of the destination
*			- otherwise - (-1) and error print; TODO check - does m_write work OK for the plug?
* Side effects:
*		- tokenize() updates global variables: g_tokens and g_ntokens
*		- updates global variable: g_curr_msg
*		- updates global isAppStop if we received "app stop"
*******************************************************************************/
int handle_plug_msg(void) {

	char 	plug_msg[MAX_CMD_LEN], *p;
	int 	dest;
		
	strcpy(plug_msg, g_curr_msg);
	tokenize(plug_msg);

	if (g_ntokens < 2) {			
		return cmd_error(ERR_INVALID_PARAM_COUNT);
	}
	dest = parse_int_num(g_tokens[0]); 	
	if ((dest < 0) || (dest > MAX_NWK_ADDR_EUI0)) {
		return TxRx_PrintError(TXRX_NWK_UNKNOWN_ADDR);
	}
	if ((dest == PLUG_NWK_ADDR_EUI0) && (strcmp("reconnect", g_tokens[1]) != 0)) {		// so far - "reconnect" is the only plug command		
		return cmd_error(ERR_UNKNOWN_CMD);
	}
	if ((strcmp("app", g_tokens[1]) == 0) && (strcmp("stop", g_tokens[2]) == 0)) {		// the command is "app stop"
		isAppStop = TRUE;
	}

	// remove the prefix of g_curr_msg (the destination) (so only the cmd msg remains): // is more general form better?
	p = plug_msg;
	p = skip_sep(p);
	p = skip_token(p);
	strcpy(g_curr_msg, p); 
		
	return dest;
}

#endif	//COMMUNICATION_PLUG

//...YL 4.8

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
#if defined WISDOM_STONE // YL 6.8 moved here from the beginning of the file
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/*******************************************************************************
* Function: 	
*		handle_msg()
* Description:
*		- break the message into its tokens
*		- parse COMMAND (first token)
*		- according to COMMAND, call the relevant function
* Parameters:
*		msg - the message to handle
* Return value:
*		0 in case of success, (-1) and error print in case of failure
* Side effects:
*		- tokenize() updates global variables: g_tokens and g_ntokens
*******************************************************************************/
int handle_msg(char *msg) {	// YL TODO - make sure the command is executed only once

	int	cmd;
	int sub_cmd;
		
	// break the message into its tokens (ended by '\0') and check it:	
	tokenize(msg); 			
	if (!g_ntokens) 		
		return ERR_NONE;	
	if (g_ntokens < 2)		
		return cmd_error(ERR_INVALID_PARAM_COUNT);
	cmd = parse_command(g_tokens[0]); 
	if (cmd == -1)		
		return cmd_error(ERR_UNKNOWN_CMD);
	sub_cmd = parse_sub_command(g_tokens[1]);
	if (sub_cmd == -1)		
		return cmd_error(ERR_UNKNOWN_SUB_CMD);
	if ((cmd != CMD_SYS) || (sub_cmd != SUB_CMD_GERR))	
		err_clear();	
		
	// dispatch to the relevant function according to COMMAND:
	switch (cmd) {

	case CMD_WRITE:
		handle_write();
		break;

	case CMD_READ:
		handle_read();
		break;
	
	case CMD_SYS:
		handle_system(sub_cmd);		
		break;

	case CMD_EEPROM:
		handle_eeprom(sub_cmd);
		break;

	case CMD_FLASH:		
		handle_flash(sub_cmd);
		break;

	case CMD_RTC:
		handle_rtc(sub_cmd);		
		break;
	
	case CMD_TEMP:
		handle_temp(sub_cmd);
		break;
		
#ifdef LCD_INSTALLED
	case CMD_LCD:	
		handle_lcd(sub_cmd);
		break;
#endif // #ifdef LCD_INSTALLED
	
	case CMD_ADS:
//		handle_ads(sub_cmd);
		break;

	case CMD_ACCMTR:	//YL 15.9
		handle_accmtr(sub_cmd);
		break;
		
	case CMD_APP:
		handle_application(sub_cmd);	
		break;

	default:
		err(ERR_UNKNOWN_CMD);
		cmd_error(0);
		break;
	}	
	return ERR_NONE;		
}
	//YL 4.8 BYTE dst_addr; //YL 23.7
	//YL 4.8 commented the following because dst_addr was removed by the plug... 
	//YL 21.7 (BM)...
	//dst_addr is the address of the stone the command is sent to
	//g_tokens[0] starts with 2-digit dst_addr followed by cmd
	//dst_addr = (g_tokens[0][0] - '0') * 10 +(g_tokens[0][1] - '0');
	//if ((dst_addr < 0) || (dst_addr > MAX_NWK_ADDR))
	//	return cmd_error(TXRX_NWK_UNKNOWN_ADDR);
	//if (dst_addr != myLongAddress[0])
	//	return cmd_error(ERR_NWK_NOT_ME);
	//cmd = parse_command(g_tokens[0] + 2); 
	//...YL 21.7
	//#define MAX_NWK_ADDR 8	//YL 23.7 max address of the stone in the network; 
								//currently the network has 1 plug and 7 stones;
								//the address of the plug is 1, and of other stones 2-8 

	//...YL 4.8

/*******************************************************************************
* Function: 	
*		handle_write()
* Description:
*		 if first token was WRITE, then handle WRITE command message:
* 		- parse DEVICE
* 		- parse ADDRESS
* 		- parse DATA
* 		- according to device name, call the relevant function
* 		- if command failed, display error message, and return error code
*		- if command executed OK, display OK message and return 0.
* Parameters:
*		None
* Return value:
*		0 in case of success, (-1) and error print in case of failure
* Side effects:
*		None
*******************************************************************************/
int handle_write(void) {

	int		dev;
	long 	addr;  	
	long 	long_dat;	
	BYTE 	byte_dat;	
	int 	res = 0;

	// parse all tokens to: dev, addr, data:
	// [return (-1) on erred tokens]
	if ((g_ntokens < 2) || (g_ntokens > 4)) 
		return cmd_error(ERR_INVALID_PARAM_COUNT);
	dev = parse_device(g_tokens[1]);
	if (dev < 0)
		return cmd_error(ERR_INVALID_DEVICE);
	if (g_ntokens < 3) {
		addr = default_addr[dev];
	} else {
		addr = parse_long_num(g_tokens[2]);
		//YL 5.8 added checking...
		if (addr < 0)
			return cmd_error(ERR_INVALID_PARAM);
		//...YL 5.8
	}
	if (g_ntokens < 4) {
		long_dat = default_data[dev];
	} else {
		long_dat = parse_long_num(g_tokens[3]);
		//YL 5.8 added checking...
		if (long_dat < 0)
			return cmd_error(ERR_INVALID_PARAM);
		//...YL 5.8
		byte_dat = long_to_byte(long_dat);
	}
	
	// dispatch to the relevant function according to dev:
	switch (dev) {
		case DEV_EEPROM:
			res = eeprom_write_byte(addr, byte_dat); 
			break;
		
		case DEV_LED:
			res = set_led(byte_dat, addr); 
			break;

		case DEV_BUZZER:
			res = play_buzzer(byte_dat);
			break;

		case DEV_RTC:	
			res = device_write_i2c_ert(RTC_ADD, 1, &byte_dat, I2C_WRITE);	
			break;

		case DEV_FLASH:
			res = flash_write_byte(addr, byte_dat);
			break;
			
#ifdef LCD_INSTALLED
		case DEV_LCD:
			print_string((addr % X_DIM), (addr / X_DIM), g_tokens[3]);
			break;
#endif	//#ifdef LCD_INSTALLED

		case DEV_ADS:		//YL 7.11
			res = ads1282_reg_write(g_tokens[2]);	
			break;

		case DEV_ACCMTR:	//YL 15.9
			res = accmtr_reg_write(long_to_byte(addr), byte_dat);	
			break;
	}
	if (res < 0)
		return cmd_error(0);
	cmd_ok();
	return 0;
}

/*******************************************************************************
* Function: 	
*		handle_read()
* Description:
*		 if first token was READ, then handle READ command message:
* 		- parse DEVICE
* 		- parse ADDRESS
* 		- parse DATA
* 		- according to device name, call the relevant function
* 		- if command failed, display error message, and return error code
*		- if command executed OK, display OK message and return 0.
* Parameters:
*		None
* Return value:
*		0 in case of success, (-1) and error print in case of failure
* Side effects:
*		None
*******************************************************************************/
int handle_read(void) {

	int 	dev;
	long 	addr;			
	int 	res;
	BYTE 	read_byte;	
	char 	ads1282_reply[(ADS1282_MAX_NUM_OF_BYTES * 2) + 1];	// for max possible reply from ADS1282

	// parse all tokens to: dev, addr:
	// [return (-1) on erred tokens]
	
	if ((g_ntokens < 2) || (g_ntokens > 3)) 
		return cmd_error(ERR_INVALID_PARAM_COUNT);
	dev = parse_device(g_tokens[1]);
	if (dev < 0)
		return cmd_error(0);
	if (g_ntokens < 3) {
		addr = default_addr[dev];
	} else {
		addr = parse_long_num(g_tokens[2]);
	}
	if (addr < 0)
		return cmd_error(ERR_INVALID_PARAM);

	// dispatch to the relevant function according to dev:
	switch (dev) {
		case DEV_EEPROM:
			res = eeprom_read_byte(addr);		
			break;

		case DEV_SWITCH:
			res = get_switch(addr); 	// choose either switch #1 or #2
			break;

		case DEV_RTC:	
			// first, write the address we would like to read from:
			addr = addr & 0x000000FF;	// addr as uchar
			res = device_write_i2c_ert(RTC_ADD, 1, (BYTE*)&addr, I2C_READ); 		
			if (res == -1)
				break;
			// then, read the data byte:
			res = device_read_i2c_ert(RTC_ADD, 1, &read_byte, I2C_READ);		
			if (res == 0)
				res = read_byte;		
			break;

		case DEV_TEMP:	
			// first, write the address we would like to read from: 
			// [although temp sensor is set to temperature register by default]
			addr = TEMP_REG;
			res = device_write_i2c_ert(TEMP_ADD, 1, (BYTE*)&addr, I2C_READ); 
			// then, read the data byte: 
			res = device_read_i2c_ert(TEMP_ADD, 1, &read_byte, I2C_READ);		
			if (res == 0)
				res = read_byte;	
			break;

		case DEV_FLASH:		
			res = flash_read_byte(addr);
			break;

//		case DEV_LCD:
//			res = ();
//			break;

		case DEV_ADS:			
			res = ads1282_reg_read(g_tokens[2], ads1282_reply);
			if (res == 0)
				m_write(ads1282_reply);
			break;

		case DEV_ACCMTR:		
			res = accmtr_reg_read(long_to_byte(addr));
			break;
	}
	if (res < 0)
		return cmd_error(0); 
	m_write(int_to_str(res));	
	write_eol();
	cmd_ok();
	return 0;
}
#endif // #ifdef WISDOM_STONE
