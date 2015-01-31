#ifndef __SYSTEM_H__															
#define __SYSTEM_H__

/***** INCLUDE FILES: *********************************************************/
#include "eeprom.h"			//Devices

/***** DEFINE: ****************************************************************/
#define VBAT_STAT_MAX	650	//= 2^10 * 2v / 3.3v	// YL 8.11 was 600, changed after comparison to 10.2.13
#define VBAT_STAT_MIN	460	//= 2^10 * 1.5v / 3.3v	// YL 8.11 was 480, changed after comparison to 10.2.13
#define VBAT_STAT_UNIT  ((VBAT_STAT_MAX - VBAT_STAT_MIN) / 10)

extern char 		*g_version; 
extern char 		g_curr_boot_cmd[MAX_BOOT_CMD_LEN];
extern BOOL 		g_sn_write;
extern unsigned int	g_vbat_level;

/***** FUNCTION PROTOTYPES: ***************************************************/
void init_all(void);	
void init_power(void); 
void prepare_for_shutdown(void);		
int  handle_set_id(void);									
void write_ver(void);													
int  handle_system(int sub_cmd);	
int  handle_set_power_enable(void);				
void handle_get_power_status(void);	
	
#endif //__SYSTEM_H__
