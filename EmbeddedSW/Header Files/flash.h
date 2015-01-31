#ifndef __FLASH_H__	
#define __FLASH_H__

#include "GenericTypeDefs.h"	

/***** DEFINE: ****************************************************************/
#define MAX_FLASH_ADDR 	2147483647u			//max address in 2GB flash (2^31 - 1) 	
#define FLASH_SECTOR_SZ 	512				//flash sector size in bytes	//512u to avoid warnings?
#define MAX_FLASH_SECTOR_ADDR ((MAX_FLASH_ADDR + 1) / FLASH_SECTOR_SZ)-1  

/***** FUNCTION PROTOTYPES: ***************************************************/
int init_flash(void);						
int flash_write_sector(DWORD sector_addr, BYTE* dat);	
int flash_read_sector(DWORD sector_addr, BYTE* dat);
int flash_write_byte(long addr, BYTE dat);
int flash_read_byte(long addr);						
int flash_format(void);
int handle_flash(int sub_cmd);
	
#endif
