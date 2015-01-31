#ifndef __PARSER_H__	
#define __PARSER_H__

#include "wistone_main.h"		
#include "GenericTypeDefs.h"

/***** DEFINE: ****************************************************************/
#define MAX_SIGNED_LONG 2147483647	// = (2^31 - 1), to allow 32 bit integers (long), [0 : (2^31 - 1)] //YL 5.8 was: 2147483647u to avoid warning
#define MAX_SIGNED_INT	32767		// = (2^15 - 1), to allow 16 bit integers, [0 : (2^15 - 1)]	//YL 5.8 added for parse_int_num
#define MAX_BYTE		255			// = (2^8 - 1), to allow 8 bit unsigned char, [0 : (2^8 - 1)] //YL 5.8 added for parse_byte_num
 
#define N_COMMANDS (sizeof(g_cmd_names)/sizeof(char*))
#define N_SUB_COMMANDS (sizeof(g_sub_cmd_names)/sizeof(char*))
#define N_DEVICES (sizeof(g_dev_names)/sizeof(char*))
#define N_MODES (sizeof(g_mode_names)/sizeof(char*))			
#define N_COMMUNICATIONS (sizeof(g_comm_names)/sizeof(char*))	//YL 5.8 was: N_DESTINATIONS
#define N_SAMPLERS (sizeof(g_samp_names)/sizeof(char*))	

typedef enum {
	CMD_WRITE = 0,
	CMD_READ,
	CMD_EEPROM,
	CMD_RTC,
	CMD_TEMP,
	CMD_LCD,
	CMD_SYS,
	CMD_FLASH,
	CMD_ADS,			//YL 7.11
	CMD_ACCMTR,	
	CMD_APP
} CmdTypes;

typedef enum {
	SUB_CMD_LED = 0,
	SUB_CMD_BUZZER,
	SUB_CMD_SWITCH,
	SUB_CMD_EEPROM,
	SUB_CMD_RTC,
	SUB_CMD_TEMP,
	SUB_CMD_FLASH,
	SUB_CMD_LCD,
	SUB_CMD_ACCMTR,
	SUB_CMD_SBOOT,	
	SUB_CMD_GBOOT,	
	SUB_CMD_WRITE,		//EEPROM read/write N bytes
	SUB_CMD_READ,	
	SUB_CMD_FORMAT,
	SUB_CMD_GVER,
	SUB_CMD_SID,
	SUB_CMD_GPOWER,
	SUB_CMD_SPOWER,	
	SUB_CMD_GERR,
	SUB_CMD_STIME,
	SUB_CMD_SDATE,
	SUB_CMD_GTIME,
	SUB_CMD_GDATE,
	SUB_CMD_SWAKEUP, 
	SUB_CMD_SALARM,  
	SUB_CMD_GTEMP,
	SUB_CMD_CLRSCR,
	SUB_CMD_START,
	SUB_CMD_STOP,
	SUB_CMD_SLEEP,
	SUB_CMD_SHUTDOWN,
	SUB_CMD_CONFIG,	 	//accelerometer
	SUB_CMD_GCD,
	SUB_CMD_MINIT,
	SUB_CMD_GCAP,		
	SUB_CMD_WSECTOR,	
	SUB_CMD_RSECTOR		
} SubCmdTypes;

typedef enum {
	DEV_LED = 0,
	DEV_BUZZER,
	DEV_SWITCH,
	DEV_EEPROM,
	DEV_RTC,
	DEV_TEMP,
	DEV_FLASH,
	DEV_LCD,
	DEV_ADS,
	DEV_ACCMTR		
} DevTypes;

typedef enum {		
	MODE_SS = 0,	
	MODE_TS,
	MODE_OST,
	MODE_IDLE	
} ModeTypes;

typedef enum { 	
	COMM_USB = 0,	
	COMM_WIRELESS,
	COMM_NONE	
} CommTypes;	//YL 5.8 was: DestTypes, DEST_USB, ...

typedef enum {
	SAMP_BOTH_1282_8451,
	SAMP_ONLY_8451
} SampTypes;

/***** FUNCTION PROTOTYPES: ***************************************************/
void 	tokenize(char *msg);
long 	parse_long_num(char *str); 	
int 	parse_int_num(char *str);
int 	parse_byte_num(char *str);	// YL 5.8 was: BYTE 	parse_byte_num(char *str);  		 							
int 	parse_name(char *name, char **namelist, int listlen);
int 	parse_command(char *name);
int 	parse_sub_command(char *name);	
int 	parse_device(char *name);
int 	parse_mode(char *name);  		
int 	parse_destination(char *name);
int		parse_single_dual_mode(char *name);
int 	handle_plug_msg(void);	// YL 4.8 added
#if defined WISDOM_STONE
int 	handle_msg(char *msg);			
int 	handle_write(void);
int 	handle_read(void);
#endif // #if defined WISDOM_STONE

#endif //__PARSER_H__
