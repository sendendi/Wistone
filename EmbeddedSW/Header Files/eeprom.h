#ifndef __EEPROM_H__	
#define __EEPROM_H__

#include "GenericTypeDefs.h"

/***** DEFINE: ****************************************************************/

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//Uncomment ONE of the 2 following lines to choose appropriate EEPROM size:
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
#define EEPROM_32K
//#define EEPROM_128K

#if defined EEPROM_32K
	#define EEPROM_MEMORY_SIZE	32 * 1024u				// in bytes (u added to avoid warnings)
	#define EEPROM_PAGE_SIZE 	64		 				// in bytes
#elif defined EEPROM_128K
	#define EEPROM_MEMORY_SIZE	128 * 1024u				// in bytes (u added to avoid warnings)
	#define EEPROM_PAGE_SIZE 	128 					// in bytes
#endif // #if defined EEPROM_32K
#define EEPROM_DEVICE_ADDRESS   0x50					// 7bit address = 0b1010000 (1010 - control code, 000 - chip select) 
#define NUM_OF_EEPROM_PAGES 	EEPROM_MEMORY_SIZE / EEPROM_PAGE_SIZE
#define EEPROM_DEFAULT_VALUE	0x00
#define MAX_SN 					100 					// maximum number of Wistones instances
#define SN_ADDRESS 				EEPROM_MEMORY_SIZE - 1	// last address in EEPROM address space
#define ALARM_ADDRESS			EEPROM_MEMORY_SIZE - 2	// to indicate that the alarm was set
#define EUI_0_ADDRESS			EEPROM_MEMORY_SIZE - 3	// YL 6.4 the first byte of 8-byte globally unique hardware identifier (for MiWi); in 32K EEPROM the EUI address is: 32765
//boot table:
#define MAX_BOOT_CMD_LEN		EEPROM_PAGE_SIZE		
#define MAX_BOOT_ENTRY_NUM		9
#define MIN_BOOT_ENTRY_NUM		0					
#define BASE_BOOT_ADDRESS		0						
#define LAST_BOOT_ADDRESS		BASE_BOOT_ADDRESS + MAX_BOOT_ENTRY_NUM * EEPROM_PAGE_SIZE	

/***** FUNCTION PROTOTYPES: ***************************************************/
int eeprom_write_byte(long addr, BYTE dat);	//read/write byte may be omitted, but they are slightly different (SN_ADDRESS, activated by r/w) 
int eeprom_read_byte(long addr);
int eeprom_write_n_bytes(long addr, char* dat);				
int eeprom_read_n_bytes(long addr, char* dat, int len);		
int eeprom_boot_set(BYTE entry, char* dat);					
int eeprom_boot_get(BYTE entry);									
int eeprom_format(void);						
int handle_eeprom(int sub_cmd);						

#endif //__EEPROM_H__
