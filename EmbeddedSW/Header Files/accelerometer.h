#ifndef __ACCELEROMETER_H__	
#define __ACCELEROMETER_H__

#include "wistone_main.h"
#include "GenericTypeDefs.h" 

/***** DEFINE: ****************************************************************/
#define ACCMTR_DEVICE_ADDRESS_RD 	0b00111001 		//7bit address: device address - 001110; SA0 - 0
#define ACCMTR_DEVICE_ADDRESS_WR 	0b00111000 		//7bit address: device address - 001110; SA0 - 0
#define ACCMTR_DEVICE_ADDRESS		0b0011100  		//7bit address: device address - 001110; SA0 - 0
#define MMA8451_Q					0x1A			//accelerometer id
#define ACCMTR_SAMP_SIZE			6	    		//in bytes

#define ACC_IE 					IEC1bits.INT2IE		// accelerometer PIC interrupt enable bit
#define ACC_IF					IFS1bits.INT2IF		// accelerometer PIC interrupt flag - IFS<11> 
//Accelerometer registers:
#define FIFO_STATUS_REG			0x00
#define FIFO_HEAD_REG			0x01
#define FIFO_SETUP_REG			0x09
#define WHO_AM_I_REG			0x0D
#define CTRL_REG1 				0x2A
#define CTRL_REG2				0x2B
#define CTRL_REG4 				0x2D
#define CTRL_REG5 				0x2E
#define XYZ_DATA_CFG			0x0E				// AY 10.8 for scale mode
//Registers' masks:
#define ACTIVE_MASK     		0x01
#define DR_MASK					0x38				// CTRL_REG1<3-5>
#define MODS_MASK				0x03				// CTRL_REG2<0-1>
#define SMODS_MASK				0x18				// CTRL_REG2<3-4>
#define INT_EN_FIFO_MASK 		0x40				// CTRL_REG4<6>
#define INT_CFG_FIFO_MASK		0x40				// CTRL_REG5<6>
#define F_WMRK_MASK				0x3F				// FIFO_SETUP_REG<0-5>
#define F_MODE_MASK				0xC0				// FIFO_SETUP_REG<6-7>
#define FS_MASK					0x03				// XYZ_DATA_CFG<0-1>
#define F_OVF_MASK				0x80				// FIFO_STATUS_REG<7>
//Registers' settings and values:
//NOTE: when setting the following group of values take into consideration the place of the set bits in the register 
//		e.g. - to choose high resolution for sleep mode in CTRL_REG2, i.e - set CTRL_REG2<3-4> bits to 0b10, write 0b000,10,000 to CTRL_REG2
#define FIFO_WK_COUNT			0x15   				// FIFO event sample count watermark, currently set to 21
#define FIFO_FILL				0x80				// FIFO fill mode
//DATA RATES
#define DATA_RATE_100			0x18  				// data sampling rate at 100 Hz // YS 17.8
#define DATA_RATE_200			0x10  				// data sampling rate at 200 Hz // YS 17.8
#define DATA_RATE_400			0x08  				// data sampling rate at 400 Hz // YS 17.8
#define DEFAULT_DATA_RATE		DATA_RATE_400  		// data sampling rate range - 100:800 Hz; currently 400 Hz (was 0x190 ??)
//MODES
#define HR_WAKE					0x02				// high resolution oversampling wake mode
#define LOW_POWER_WAKE			0x03				// low power wake mode	
#define NORMAL_WAKE				0x00				// normal wake mode // YS 17.8
#define LOW_POWER_NOISE_WAKE	0x01				// low power and low noise wake mode // YS 17.8
#define HR_SLEEP				0x10				// high resolution sleep mode 
#define LOW_POWER_SLEEP			0x11				// low power sleep mode
//FIFO
#define FIFO_INT_EN				0x40				// FIFO interrupt enabled
#define FIFO_INT_DIS			0x00				// FIFO interrupt disabled
#define FIFO_INT1				0x40				// FIFO interrupt is routed to INT1 pin
//SCALING
#define FULL_SCALE_2G			0x00				// AY 10.8 scale mode
#define FULL_SCALE_4G			0x01				// YS 17.8 new scale mode
#define FULL_SCALE_8G			0x02				// YS 17.8 new scale mode
#define F_OVF_SET				0x80				// AY 10.8 overflow occurred

//CONFIG enumerator:
enum config_option {
	CONFIG_DATA_RATE = 1 ,
	CONFIG_SCALING = 2,
	CONFIG_MODE = 3
};

extern BYTE g_accmtr_blk_buff[CYCLIC_BUFFER_SIZE * MAX_BLOCK_SIZE];
extern BYTE g_accmtr_is_blk_rdy[CYCLIC_BUFFER_SIZE];	

/***** FUNCTION PROTOTYPES: ***************************************************/
void init_accmtr(void);
void accmtr_standby(void);
// YL 14.9 ...
// was: void accmtr_active(void);
int accmtr_active(void);
// ... YL 14.9
int  accmtr_reg_write(BYTE reg_addr, BYTE dat); 
int  accmtr_reg_read(BYTE reg_addr);
int  handle_accmtr(int sub_cmd);	

#endif // #ifndef __ACCELEROMETER_H__	
