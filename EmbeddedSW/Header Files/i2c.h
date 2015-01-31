#ifndef __I2C_H__					
#define __I2C_H__

#include "wistone_main.h"
#include "GenericTypeDefs.h"

/***** DEFINE: ****************************************************************/

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//Uncomment the  following line to choose HARDWARE I2C mode
//for ERT (EEPROM, RTC, TEMP) instead of default SOFTWARE mode:
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//#if defined WISDOM_STONE
#define HARDWARE_I2C_ERT	
//#endif // #if defined WISDOM_STONE

#define I2C_WRITE 	0	//write_before_read parameter to indicate whether the write is part of read or not
#define I2C_READ	1

/***** FUNCTION PROTOTYPES: ***************************************************/
void	 init_i2c(void);

// for ACCELEROMETER:

int 	 device_write_i2c_accmtr(BYTE address, int n, BYTE *data, BYTE write_before_read);  	
int		 device_read_i2c_accmtr(BYTE address,  int n, BYTE *data, BYTE write_before_read);   
inline 	 __attribute__((always_inline)) void 	start_i2c_accmtr(int is_restart);
inline	 __attribute__((always_inline)) void 	stop_i2c_accmtr(void);
inline 	 __attribute__((always_inline)) BYTE 	in_byte_i2c_accmtr(int is_last);
inline   __attribute__((always_inline)) int 	out_byte_i2c_accmtr(int dat);

// for ERT (EEPROM, RTC, TEMP):
int 	device_write_i2c_ert(BYTE address, int n, BYTE *data, BYTE write_before_read); 			
int 	device_read_i2c_ert(BYTE address,  int n, BYTE *data, BYTE write_before_read);	 		
void 	start_i2c_ert(int is_restart);
void 	stop_i2c_ert(void);
int 	in_byte_i2c_ert(int is_last);
int 	out_byte_i2c_ert(int dat);
void 	delay_i2c_ert(void);

#endif //__I2C_H__
