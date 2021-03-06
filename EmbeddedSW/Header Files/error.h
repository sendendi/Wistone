#ifndef __ERROR_H__	
#define __ERROR_H__

/***** DEFINE: ****************************************************************/
#define ERROR_BUZZ_TIME 1		// 1KHz sound duration in [Sec], when error detected 

typedef enum {
	ERR_NONE = 0,
	ERR_UNKNOWN,
	ERR_UNKNOWN_CMD,			
	ERR_UNKNOWN_SUB_CMD,	
	ERR_INVALID_PARAM,
	ERR_INVALID_PARAM_COUNT,
	ERR_NUM_TOO_BIG,	
	ERR_INVALID_NUM,
	ERR_INVALID_LED_STATE,
	ERR_INVALID_BUZZ_PERIOD,
	
	ERR_I2C_BUSY,
	ERR_DEVICE_WRITE_I2C,
	ERR_DEVICE_READ_I2C,
	
	ERR_EEPROM_BOOT_SET,		
	ERR_EEPROM_BOOT_GET,		
	ERR_EEPROM_WRITE_BYTE,
	ERR_EEPROM_READ_BYTE,
	ERR_EEPROM_WRITE_N_BYTES,	
	ERR_EEPROM_READ_N_BYTES,	
	ERR_EEPROM_FORMAT,
	
	ERR_TEMP_INIT, 				
	ERR_GET_TEMP,

	ERR_RTC_INIT,
	ERR_RTC_GET_TIME,
	ERR_INVALID_STIME,			
	ERR_INVALID_SDATE,
	ERR_INVALID_SWAKEUP, 		
	ERR_INVALID_SALARM, 
	
	ERR_SDSPI_INIT,	
	ERR_SDSPI_CD,				
	ERR_SDSPI_WRITE,			
	ERR_SDSPI_READ, 			
	ERR_INVALID_DEVICE,
	
	ERR_INVALID_MODE, 		
	ERR_INVALID_COMM,			// YL 5.8 instead of ERR_INVALID_DEST
	ERR_INVALID_SAMP,
	
	ERR_ACCMTR_UNKNOWN_ID,			
	ERR_ACCMTR_REG_WRITE, 	
	ERR_ACCMTR_REG_READ, 
	ERR_ADS1282_UNKOWN_ID,		
	ERR_ADS1282_REG_READ,		
	ERR_ADS1282_MISSING_POWER,
	
	// renamed and moved to TxRx: ERR_NWK_UNKNOWN_ADDR,			// YL 4.8 invalid network address of the destination
	// renamed and moved to TxRx: ERR_NWK_NOT_ME,				// YL 23.7 the stone received a command that wasn't addressed to it
	ERR_MAX,
} ErrType;

extern ErrType g_error;

/***** FUNCTION PROTOTYPES: ***************************************************/
char *get_last_error_str(void);
int err(ErrType err);
void err_clear(void);

#endif //__ERROR_H__
