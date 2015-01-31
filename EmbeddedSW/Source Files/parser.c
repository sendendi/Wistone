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
#ifdef WISDOM_STONE

/***** INCLUDE FILES: *********************************************************/
#include <string.h>
#include "app.h"						//Application
#include "command.h"					//Application
#include "error.h"						//Application
#include "parser.h"						//Application
#include "system.h"						//Application
#include "HardwareProfile.h"			//Common
#include "misc_c.h"						//Common //YL 27.8
#include "accelerometer.h"				//Devices
#include "ads1282.h"					//Devices
#include "analog.h"						//Devices
#include "eeprom.h"						//Devices
#include "flash.h"						//Devices
#include "lcd.h"						//Devices
#include "led_buzzer.h"					//Devices
#include "rtc.h"						//Devices			
#include "temp.h"						//Devices
#include "i2c.h"						//Protocols	//YL 30.8

/***** GLOBAL VARIABLES: ******************************************************/
// this table holds the possible COMMANDs
// must be in the same order as the enum CmdTypes
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
	"accmtr",	//YL 15.9
	"app"
};

// this table holds the possible SUB COMMANDs
// must be in the same order as the enum SubCmdTypes
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
	"sboot",	//YL 17.9
	"gboot",	//YL 17.9
	"write",	//YL 17.9 EEPROM read/write N bytes
	"read",		//YL 17.9
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
	"swakeup",	//YL 15.9
	"salarm",	//YL 15.9
	"gtemp",
	"clrscr",
	"start",
	"stop",
	"sleep",
	"shutdown",
	"config",	//YL 15.9 accelerometer
	"gcd",
	"minit",
	"gcap",			
	"wsector",		
	"rsector"		
};

// must be in the same order as the enum DevTypes
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
	"accmtr"	//YL 15.9 accelerometer, instead of sensor
};

// must be in the same order as the enum ModeTypes
char* g_mode_names[] = {	
	"ss",
	"ts",
	"ost",
	"" 		//no "idle" cmd mode parameter  
};

// must be in the same order as the enum DestTypes
char* g_dest_names[] = {   
	"usb",
	"wireless",
	"none"		//?  
};

// must be in the same order as the enum SampTypes //YL 7.11
char* g_samp_names[] = {   
	"dual",
	"single"
};

//YL 12.8 moved here default_addr and default_data

// this table holds default address for each device
// the default will be used in case the <address> token is missing in low level WRITE/READ commands.
// when we don't allow missing address (e.g. in EEPROM), we assign it "-1".
long default_addr[] = {
	0,  // DEV_LED
	0,  // DEV_BUZZER
	0,  // DEV_SWITCH
	0, 	// DEV_EEPROM
	0,	// DEV_RTC
	//0,	// DEV_ADC //YL 8.12 isn't used
	0, 	// DEV_TEMP
	0, 	// DEV_FLASH
	0,	// DEV_LCD
	0	// DEV_ACCMTR
};

// this table holds default data field for each device
// the default will be used in case the <data> token is missing in low level WRITE commands.
// when we don't allow missing address (e.g. in EEPROM), we assign it "-1".
long default_data[] = {
	0,  // DEV_LED
	0,  // DEV_BUZZER
	-1, // DEV_SWITCH
	-1, // DEV_EEPROM
	-1,	// DEV_RTC
	//-1,	// DEV_ADC //YL 8.12 isn't used
	-1, // DEV_TEMP
	-1, // DEV_FLASH
	-1,	// DEV_LCD
	-1	// DEV_ACCMTR
};

/***** INTERNAL PROTOTYPES: ***************************************************/
char*	skip_sep(char *p);
char*	skip_token(char *p);

/*******************************************************************************
// tokenize()
// split command into parsable tokens.
*******************************************************************************/
void tokenize(char *msg)
{
	static char msg_copy[MAX_CMD_LEN];	//YL 20.9 added copy, so g_in_msg remain unchanged; TODO - maybe some static strings are not neccessary
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
// skip_sep()
// skip delimiters between tokens inside message
*******************************************************************************/
char *skip_sep(char *p)
{
	while ((*p == ' ') || (*p == '\t') || (*p == ',')) p++;
	
	return p;
}

/*******************************************************************************
// skip_token()
// skip token to nearest delimiter inside message
*******************************************************************************/
char *skip_token(char *p)
{
	while (*p) {
		if ((*p == ' ') || (*p == '\t') || (*p == ','))
			return p;
		p++;
	}
	
	return p;
}

/*******************************************************************************
// parse_long_num()
// parse string to long num (signed).
// allow 32bit numbers
*******************************************************************************/
long parse_long_num(char *str) //YL 22.8 instead of parse_num; TODO - negative num parsing
{
	long res = 0;
	long max_res = MAX_LONG / 10; 
	char *p = str;

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
// parse_int_num()
// parse string to int num (signed).
*******************************************************************************/
int parse_int_num(char *str) //YL 22.8 instead of parse_num
{
	return (0xFFFF & parse_long_num(str));
}

/*******************************************************************************
// parse_byte_num()
// parse string to unsigned byte num.
*******************************************************************************/
BYTE parse_byte_num(char *str) //YL 28.8 instead of parse_num
{
	BYTE res = 0;
	char *p = str;

	while (*p != 0) {
		if (*p == '-')
			return err(ERR_INVALID_NUM);
		res = res * 10 + *p - '0';
		p++;
	}
	return res;
}

/*******************************************************************************
// parse_name()
// find a given string inside a list of strings.
// return the name sequence number in the list.
// if not found, return -1
*******************************************************************************/
int parse_name(char *name, char **namelist, int listlen)
{
	char *p1, *p2;
	int i;

	for (i=0; i<listlen; i++) {
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
// parse_command()
// look for the current command in the list of possible commands
*******************************************************************************/
int parse_command(char *name)
{
	return parse_name(name, g_cmd_names, N_COMMANDS);
}

/*******************************************************************************
// parse_sub_command()
// look for the current sub command in the list of possible sub commands
*******************************************************************************/
int parse_sub_command(char *name)
{
	return parse_name(name, g_sub_cmd_names, N_SUB_COMMANDS);
}

/*******************************************************************************
// parse_device()
// look for the current device name in the list of possible devices
*******************************************************************************/
int parse_device(char *name)
{
	int res = parse_name(name, g_dev_names, N_DEVICES);

	if (res < 0) {
		err_clear(); // make sure following error superseed any prev one
		return err(ERR_INVALID_DEVICE);
	}
	
	return res;
}

/*******************************************************************************
// parse_mode()
// look for the current mode name in the list of possible app modes
*******************************************************************************/
int parse_mode(char *name)  		
{
	int res = parse_name(name, g_mode_names, N_MODES);

	if (res < 0) {
		err_clear(); // make sure following error superseed any prev one
		err(ERR_INVALID_MODE);
	}
	
	return res;
}

/*******************************************************************************
// parse_destination()
// look for the current destination in the list of possible app destinations
*******************************************************************************/
int parse_destination(char *name) 	
{
	int res = parse_name(name, g_dest_names, N_DESTINATIONS);

	if (res < 0) {
		err_clear(); // make sure following error superseed any prev one
		err(ERR_INVALID_DEST);
	}
	
	return res;
}

/*******************************************************************************
// parse_sampler()
// look for the current sampler in the list of possible app samplers
*******************************************************************************/
int parse_single_dual_mode(char *name)
{
	int res = parse_name(name, g_samp_names, N_SAMPLERS);

	if (res < 0) {
		err_clear(); // make sure following error superseed any prev one
		err(ERR_INVALID_SAMP);
	}
	
	return res;
}

/*******************************************************************************
// handle_msg()
// - break the message into its tokens
// - parse COMMAND (first token)
// - according to COMMAND, call the relevant function
*******************************************************************************/
int handle_msg(char *msg)	
{
	int cmd;
	int sub_cmd;

	tokenize(msg); 			// break message into its tokens, ended by '\0'	
	if (!g_ntokens) 		// in case of empty message
		return ERR_NONE;	// assumed empty message isn't error
	if (g_ntokens < 2)		
		return cmd_error(ERR_INVALID_PARAM_COUNT);
	cmd = parse_command(g_tokens[0]);
	if (cmd == -1)		
		return cmd_error(ERR_UNKNOWN_CMD);
	sub_cmd = parse_sub_command(g_tokens[1]);
	if (sub_cmd == -1)		
		return cmd_error(ERR_INVALID_SUB_CMD);
	if ((cmd != CMD_SYS) || (sub_cmd != SUB_CMD_GERR))	
		err_clear();	
	
	// dispatch to the relevant function according to COMMAND
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

/*******************************************************************************
// handle_write()
// if first token was WRITE, then handle WRITE command message:
// - parse DEVICE
// - parse ADDRESS
// - parse DATA
// - according to device name, call the relevant function
// - if command failed, display error message, and return error code
// - if command executed OK, display OK message and return 0.
*******************************************************************************/
int handle_write(void)
{
	int dev;
	long addr;  	
	long long_dat;	
	BYTE byte_dat;	
	int res = 0;

	// parse all tokens to: dev, addr, data
	// return -1 on erred tokens
	if ((g_ntokens < 2) || (g_ntokens > 4)) 
		return cmd_error(ERR_INVALID_PARAM_COUNT);
	dev = parse_device(g_tokens[1]);
	if (dev < 0)
		return cmd_error(ERR_INVALID_DEVICE);
	if (g_ntokens < 3) {
		addr = default_addr[dev];
	} else {
		addr = parse_long_num(g_tokens[2]);
	}
	if (g_ntokens < 4) {
		long_dat = default_data[dev];
	} else {
		long_dat = parse_long_num(g_tokens[3]);
	}
	byte_dat = long_to_byte(long_dat);
	if (addr < 0)		 
		return cmd_error(ERR_INVALID_PARAM);

	// dispatch to the relevant function according to dev
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
// handle_read()
// if first token was READ, then handle READ command message:
// - parse DEVICE
// - parse ADDRESS
// - according to device name, call the relevant function
// - if command failed, display error message, and return error code
// - if command executed OK, display the result message and return 0.
*******************************************************************************/
int handle_read(void)
{
	int dev;
	long addr;			
	int res;
	BYTE read_byte;	
	char	ads1282_reply[(ADS1282_MAX_NUM_OF_BYTES * 2) + 1];		// for max possible reply from ADS1282

	// parse all tokens to: dev, addr
	// return -1 on erred tokens
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

	// dispatch to the relevant function according to dev
	switch (dev) {
		case DEV_EEPROM:
			res = eeprom_read_byte(addr);		
			break;

		case DEV_SWITCH:
			res = get_switch(addr); //choose either switch #1 or #2
			break;

		case DEV_RTC:	
			// first, write the address we would like to read from
			addr = addr & 0x000000FF;							//to treat addr as uchar
			res = device_write_i2c_ert(RTC_ADD, 1, (BYTE*)&addr, I2C_READ); 		
			if (res == -1)
				break;
			// then, read the data byte
			res = device_read_i2c_ert(RTC_ADD, 1, &read_byte, I2C_READ);		//to treat addr as uchar
			if (res == 0)
				res = read_byte;		
			break;

		case DEV_TEMP:	
			// first, write the address we would like to read from (although temp sensor is set to temperature register by default)
			addr = TEMP_REG;
			res = device_write_i2c_ert(TEMP_ADD, 1, (BYTE*)&addr, I2C_READ); 
			// then, read the data byte 
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

		case DEV_ADS:		//YL 7.11
			res = ads1282_reg_read(g_tokens[2], ads1282_reply);
			if (res == 0)
				m_write(ads1282_reply);
			break;

		case DEV_ACCMTR:	//YL 15.9
			res = accmtr_reg_read(long_to_byte(addr));
			break;
	}
	if (res < 0)
		return cmd_error(0); 
	m_write(uint_to_str(res));	//YL 11.11 instead of disp_num_to_term
	write_eol();
	cmd_ok();
	return 0;
}
#endif // #ifdef WISDOM_STONE
