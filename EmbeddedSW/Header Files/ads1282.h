#ifndef __ADS1282_H__	
#define __ADS1282_H__
#include "wistone_main.h"
#include "p24FJ256GB110.h"
#include "GenericTypeDefs.h" 

/***** DEFINE: ****************************************************************/
// FLASH sector address for the first block of ADC samples
#define FLASH_SECTOR_ADS1282_OFFSET			(0x40000000 / FLASH_SECTOR_SZ)  // FLASH total size / 2 / sector_size
// ASD1282 internal addresses:
#define ID_REG 								0x00
#define ID_MASK								0xF0
#define CONFIG0_REG 						0x01
#define CONFIG1_REG 						0x02
#define REG_NUM_MASK						0x1F	//= 000n,nnnn - for the number of the registers to read/write
#define REG_R_MASK							0x3F	//= 001r,rrrr - for the starting address of the first register to read
#define REG_W_MASK							0x5F	//= 010r,rrrr - for the starting address of the first register to write
#define WAKEUP_1_CMD						0x00	//input1 wakeup
#define WAKEUP_2_CMD						0x01	//input2 wakeup
#define	STANDBY_1_CMD						0x02	//input1 standby	
#define	STANDBY_2_CMD						0x03	//input2 standby	

#define ADS1282_SAMP_SIZE					32		// single sample size in [bits]
#define ADS1282_MAX_NUM_OF_BYTES			11		// max num of registers to read at a single read operation

#define ADS1282_IE 							IEC3bits.INT3IE	//ads1282 PIC interrupt enable bit
#define ADS1282_IF							IFS3bits.INT3IF	//ads1282 PIC interrupt flag

extern BYTE g_ads1282_blk_buff[CYCLIC_BUFFER_SIZE * MAX_BLOCK_SIZE];
extern BYTE g_ads1282_is_blk_rdy[CYCLIC_BUFFER_SIZE];	
extern BOOL g_is_ads1282_active;

/***** FUNCTION PROTOTYPES: ***************************************************/
int	 init_ads1282(void);
int  ads1282_active(void);
void ads1282_standby(void);
int  ads1282_reg_read(char *command, char* reply);
int  ads1282_reg_write(char *command_data);
//int  handle_adc(int sub_cmd);	

#endif //#define __ADS1282_H__
