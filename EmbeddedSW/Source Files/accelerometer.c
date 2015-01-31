/*******************************************************************************

accelerometer.c - handle accelerometer commands
===============================================

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
this file contains functions for operating the accelerometer.
using HW implemented I2C protocol.
- accelerometer ISR
- accelerometer active, standby
- register READ/WRITE 

********************************************************************************
we use Freescale MMA8451.Q accelorometer

*******************************************************************************/
#include "wistone_main.h"
#ifdef WISDOM_STONE

/***** INCLUDE FILES: *********************************************************/
#include "command.h"			//Application
#include "error.h"				//Application
#include "parser.h"				//Application 
#include "p24FJ256GB110.h"		//Common
#include "accelerometer.h"		//Devices
#include "i2c.h"				//Protocols
#include "usb_config.h"			//USB_UART
	
/***** GLOBAL VARIABLES: ******************************************************/
BYTE g_accmtr_blk_buff[CYCLIC_BUFFER_SIZE * MAX_BLOCK_SIZE];	//cyclic buffer space for SS and OST (TS uses only the first block of it) 	
BYTE g_accmtr_is_blk_rdy[CYCLIC_BUFFER_SIZE];					//1 - indicates that a complete lower g_accmtr_blk_buff half was filled by the sampler
WORD g_accmtr_blk_buff_w_ptr = 0;								//write pointer used by ISR, pointing to g_accmtr_blk_buff indices - from 0 to 2 x MAX_BLOCK_SIZE - 1 
BYTE g_accmtr_overflow_cntr;		
BYTE g_accmtr_blk_overflow_cntr;	

/***** INTERNAL PROTOTYPES: ***************************************************/
int  accmtr_config(void);	//YS 17.8
void accmtr_reg_set(BYTE reg_addr, BYTE in_reg_addr, BYTE setting); 

/*******************************************************************************
// _INT2Interrupt()
// accelerometer ISR
// acceletometer settings are such that the only source for interrupt is FIFO's watermark 
// for debug - we check overflow as well
*******************************************************************************/
void  __attribute__((interrupt, auto_psv)) _INT2Interrupt(void) //auto_psv attribute is added if an ISR references const variables or string literals using the constants-in-code memory model (?)
{
 	int	i;
	BYTE fifo_status_reg;										//accelerometer register with samples' info
	BYTE numOfSamplesAvailable;
	BYTE blockNum;
	
	//mask PIC accelerometer interrupt:
	ACC_IE = 0;													//disable accelerometer PIC interrupt
	ACC_IF = 0;													//accelerometer PIC interrupt flag is cleared, to indicate that the interrupt request has been accepted 
	
	//read accelerometer status reg:
	start_i2c_accmtr(0);
	out_byte_i2c_accmtr(ACCMTR_DEVICE_ADDRESS_WR);				//0 is added as last bit to indicate i2c write (i2c read sequence starts with writing the device address, followed by writing the address within the device to read from)
	out_byte_i2c_accmtr(FIFO_STATUS_REG);						//INT_SOURCE<6> - SRC_FIFO bit, is accelerometer interrupt read only bit, and it is cleared as a consequence of FIFO_STATUS_REG reading  
	start_i2c_accmtr(1);	
	out_byte_i2c_accmtr(ACCMTR_DEVICE_ADDRESS_RD);				//1 is added as last bit to indicate i2c read
	fifo_status_reg = in_byte_i2c_accmtr(1);
	stop_i2c_accmtr();
	// check for overflow bit:
	if (fifo_status_reg & F_OVF_MASK) 
		g_accmtr_overflow_cntr++; 
	// check for memory buffer overflow:
	#if (CYCLIC_BUFFER_SIZE == 2)
		if (g_accmtr_is_blk_rdy[0] & g_accmtr_is_blk_rdy[1])
			g_accmtr_blk_overflow_cntr++;
	#elif (CYCLIC_BUFFER_SIZE == 4)
		if (g_accmtr_is_blk_rdy[0] & g_accmtr_is_blk_rdy[1] & g_accmtr_is_blk_rdy[2] & g_accmtr_is_blk_rdy[3])
			g_accmtr_blk_overflow_cntr++;
	#elif (CYCLIC_BUFFER_SIZE == 8)
		if (g_accmtr_is_blk_rdy[0] & g_accmtr_is_blk_rdy[1] & g_accmtr_is_blk_rdy[2] & g_accmtr_is_blk_rdy[3] & g_accmtr_is_blk_rdy[4] & g_accmtr_is_blk_rdy[5] & g_accmtr_is_blk_rdy[6] & g_accmtr_is_blk_rdy[7])
			g_accmtr_blk_overflow_cntr++;
	#else
		#error "CYCLIC_BUFFER_SIZE should be 2,4 or 8"
	#endif
	
	//read samples from fifo:
	//extruct num of available samples:
	numOfSamplesAvailable = fifo_status_reg & 0x3F;	

	//since the watermark is the only interrupt source enabled, there is no need to check here if it is set
	//there are at least FIFO_WK_COUNT samples ready in the FIFO; we read exactly 6 x FIFO_WK_COUNT bytes (each sample consists of 6 bytes - 2 bytes for each axis)
	start_i2c_accmtr(0);
	out_byte_i2c_accmtr(ACCMTR_DEVICE_ADDRESS_WR);
	out_byte_i2c_accmtr(FIFO_HEAD_REG);
	start_i2c_accmtr(1);
	out_byte_i2c_accmtr(ACCMTR_DEVICE_ADDRESS_RD);
	for(i = 0; i < (ACCMTR_SAMP_SIZE * numOfSamplesAvailable) - 1; i++) {
		g_accmtr_blk_buff[g_accmtr_blk_buff_w_ptr] = in_byte_i2c_accmtr(0);
		g_accmtr_blk_buff_w_ptr++;	// increment the g_accmtr_blk_buff position by 1
		// check if reach slightly before end of block (512 byte):
		if ((g_accmtr_blk_buff_w_ptr & 0x01FF) == (MAX_BLOCK_SIZE - 8)) {					//writing 'x' to last 2 bytes
			blockNum = ((g_accmtr_blk_buff_w_ptr & (0xFE00)) >> 9);							//dividing g_accmtr_blk_buff_w_ptr by 512	
			// blockNum indicates what buffer was filled..
			g_accmtr_is_blk_rdy[blockNum] = 1; 												//notify that block[blockNum] was filled and ready to be sent through usb/wireless, or saved to flash
			g_accmtr_blk_buff[++g_accmtr_blk_buff_w_ptr] = g_accmtr_overflow_cntr;			//put in the 505 place of the current block the overflow status
			g_accmtr_blk_buff[++g_accmtr_blk_buff_w_ptr] = g_accmtr_blk_overflow_cntr; 		//put in the 506 place of the current block the overflow cyclic buffer status
			g_accmtr_overflow_cntr = 0;
			g_accmtr_blk_overflow_cntr = 0;
			g_accmtr_blk_buff_w_ptr = ((g_accmtr_blk_buff_w_ptr + 6) & (CYCLIC_BLOCK_MASK));//put the buffer pointer to the next block
		}
	}
	g_accmtr_blk_buff[g_accmtr_blk_buff_w_ptr] = in_byte_i2c_accmtr(1);						//read the last byte separately in order to stop I2C 
	stop_i2c_accmtr();
	g_accmtr_blk_buff_w_ptr++;																// increment the g_accmtr_blk_buff position by 1
	// check if reach slightly before end of block (512 byte):
	if ((g_accmtr_blk_buff_w_ptr & 0x01FF) == (MAX_BLOCK_SIZE - 8)) {						//writing 'x' to last 2 bytes
		blockNum = ((g_accmtr_blk_buff_w_ptr & (0xFE00)) >> 9);								//dividing g_accmtr_blk_buff_w_ptr by 512	
		// blockNum indicates what buffer was filled..
		g_accmtr_is_blk_rdy[blockNum] = 1; 													//notify that block[blockNum] was filled and ready to be sent through usb/wireless, or saved to flash
		g_accmtr_blk_buff[++g_accmtr_blk_buff_w_ptr] = g_accmtr_overflow_cntr; 				//put in the 505 place of the current block the overflow status
		g_accmtr_blk_buff[++g_accmtr_blk_buff_w_ptr] = g_accmtr_blk_overflow_cntr; 			//put in the 506 place of the current block the overflow cyclic buffer status
		g_accmtr_overflow_cntr = 0;
		g_accmtr_blk_overflow_cntr = 0;
		g_accmtr_blk_buff_w_ptr = ((g_accmtr_blk_buff_w_ptr + 6) & (CYCLIC_BLOCK_MASK));	//put the buffer pointer to the next block
	}
		
	//unmask PIC accelerometer interrupt
	ACC_IE = 1;		
}

/*******************************************************************************
// init_accmtr()
// initialize accelerometer:
//	- disable PIC and accelerometer interrupts to INT1
// 	- configure sensor for:
//		- FIFO watermark - the only reason for interrupt, routed to INT1 pin
//		- full (minimum) scale value range of is 2g by default (for maximum precision)	
*******************************************************************************/
void init_accmtr()	
{	
	BYTE i;
	int check = accmtr_reg_read(WHO_AM_I_REG);	

	// check for Accelerometer component existance:
	if (check != (0x00FF & MMA8451_Q)) {
		err(ERR_ACCMTR_UNKNOWN_ID);
		return;
	}
	//PIC Interrupt Registers:
	ACC_IE = 0;														//IEC1<11> - external interrupt #1 is disabled
	INTCON2bits.INT2EP = 1;											// set Interrupt edge polarity to neg-edge
	//Acclerometer Interrupt Registers:
	accmtr_reg_set(CTRL_REG4, INT_EN_FIFO_MASK, FIFO_INT_DIS);		//CTRL_REG4<6> - INT_EN_FIFO bit: 1 - FIFO interrupt enabled, 0 - FIFO interrupt disabled	
	//Acclerometer Data Registers:
	accmtr_standby();
	accmtr_reg_set(FIFO_SETUP_REG, F_MODE_MASK, FIFO_FILL);			//FIFO_SETUP_REG<6-7> - 2 F_MODE bits, set to 10 for FILL mode (FIFO stops accepting new samples when overflowed) 
	accmtr_reg_set(FIFO_SETUP_REG, F_WMRK_MASK, FIFO_WK_COUNT);		//FIFO_SETUP_REG<0-5> - 6 F_WMRK bits: FIFO event sample count watermark, currently set to 20 
	accmtr_reg_set(CTRL_REG1, DR_MASK, DEFAULT_DATA_RATE); 			//CTRL_REG1<3-5> - 3 DR bits: data rate selection; CTRL_REG1<1> - select the Output Data Rate (ODR) for acceleration samples.//YS 17.8
	//YL 15.8 accmtr_reg_set(CTRL_REG2, MODS_MASK, HR_WAKE);		//CTRL_REG2<0-1> - 2 MODS bits = 10 for high resolution (14 bits) oversampling mode in WAKE mode
	//YL 15.8 accmtr_reg_set(CTRL_REG2, SMODS_MASK, HR_SLEEP);		//CTRL_REG2<3-4> - 2 SMODS bits = 11 for high resolution in SLEEP mode
	accmtr_reg_set(CTRL_REG2, MODS_MASK, LOW_POWER_NOISE_WAKE);		//CTRL_REG2<0-1> - 2 MODS bits = 01 for low noise (14 bits) mode in WAKE mode//YL 15.8
	accmtr_reg_set(CTRL_REG2, SMODS_MASK, HR_SLEEP);				//CTRL_REG2<3-4> - 2 SMODS bits = 11 for high resolution in SLEEP mode//YL 15.8
	accmtr_reg_set(XYZ_DATA_CFG, FS_MASK, FULL_SCALE_2G);			//XYZ_DATA_CFG<0-1> - 2 FS bits = 00 for 2g full scale mode
	//Accelerometer Interrupt Registers: 
	accmtr_reg_set(CTRL_REG5, INT_CFG_FIFO_MASK, FIFO_INT1);	 	//CTRL_REG5<6> - INT_CFG_FIFO bit: 1 -  interrupt is routed to INT1 pin (0 - interrupt is routed to INT2 pin)
	// clear "double buffers" between accelerometer and RXTX (relevant for OST mode):
	for(i = 0; i < CYCLIC_BUFFER_SIZE ; i++){
		g_accmtr_is_blk_rdy[i] = 0;
	}
}
		
/*******************************************************************************
// accmtr_standby()
// accelerometer standby:
// CTRL_REG1<0> - ACTIVE bit: 1 - active, 0 - standby 
*******************************************************************************/
void accmtr_standby() 
{			
	int 	res;	
	BYTE 	curr_setting;
	
	res = accmtr_reg_read(CTRL_REG1); 								//read current value of control register #1
	if (res == -1) {
		err(ERR_ACCMTR_REG_READ);
		return;
	}
	curr_setting = res & 0xFF;
	if (accmtr_reg_write(CTRL_REG1, curr_setting & (~ACTIVE_MASK)))	//set standby mode by clearing the active bit
		err(ERR_ACCMTR_REG_WRITE);	
	ACC_IE = 0;	// AY 10.8
	ACC_IF = 0;	// AY 10.8
}

/*******************************************************************************
// accmtr_active() 
// accelerometer active:
// CTRL_REG1<0> - ACTIVE bit: 1 - active, 0 - standby 
*******************************************************************************/
void accmtr_active() 
{			
	unsigned int	i;
	int 			res;	
	BYTE 			curr_setting;
	
	// clear "double buffers" between accelerometer and RXTX (relevant for OST mode):
	for (i = 0; i < CYCLIC_BUFFER_SIZE; i++){
		g_accmtr_is_blk_rdy[i] = 0;
	}
	// reset error counters:
	g_accmtr_overflow_cntr = 0;
	g_accmtr_blk_overflow_cntr = 0;
	g_accmtr_blk_buff_w_ptr = 0;		
	accmtr_reg_set(CTRL_REG4, INT_EN_FIFO_MASK, FIFO_INT_EN);		//CTRL_REG4<6> - INT_EN_FIFO bit: 1 - FIFO interrupt enabled (0 - FIFO interrupt disabled)
	ACC_IF = 0;	// AY 10.8
	ACC_IE = 1;	// AY 10.8
	res = accmtr_reg_read(CTRL_REG1); 								//read current value of control register #1
	if (res == -1) {
		err(ERR_ACCMTR_REG_READ);
		return;
	}
	curr_setting = res & 0xFF;
	if (accmtr_reg_write(CTRL_REG1, curr_setting | ACTIVE_MASK)) 	//set active mode
		err(ERR_ACCMTR_REG_WRITE);	
}

/*******************************************************************************
// accmtr_reg_set()
// writes new setting to zeroed specified accelerometer register bits, considerring it's current setting
*******************************************************************************/
void accmtr_reg_set(BYTE reg_addr, BYTE mask, BYTE setting)
{
	int 	res;	
	BYTE 	curr_setting;	

	res = accmtr_reg_read(reg_addr);								 //reads current value of the register
	if (res == -1) {	
		err(ERR_ACCMTR_REG_READ);
		return;
	}
	curr_setting = res & 0xFF; 							
	curr_setting = (curr_setting & 0x00FF) & (~mask);				//zeros specified registers' bits
	if (accmtr_reg_write(reg_addr, curr_setting | setting))			//writes new setting to previously zeroed registers' bits
		err(ERR_ACCMTR_REG_WRITE);	
}

/*******************************************************************************
// accmtr_reg_write()
// write a single data Byte into specified accelerometer register address
// return:
// -  0   - on success
// - (-1) - on failure
*******************************************************************************/
int accmtr_reg_write(BYTE reg_addr, BYTE dat) 	//YL 15.9 added check to device write return value
{	
	BYTE data_to_write[2];
	BYTE device_addr = ACCMTR_DEVICE_ADDRESS;
	
	data_to_write[0] = reg_addr;
	data_to_write[1] = dat;
	if (device_write_i2c_accmtr(device_addr, 2, data_to_write, I2C_WRITE))
		return (-1);	//indicates failure
	return 0;
}

/*******************************************************************************
// accmtr_reg_read()
// read a single data Byte from specified accelerometer register address
// return:
// - 0~255 - on success
// - (-1)  - on failure
*******************************************************************************/
int accmtr_reg_read(BYTE reg_addr) 	//YL 15.9 added check to device read/write return value
{
	BYTE device_addr = ACCMTR_DEVICE_ADDRESS;
	BYTE read_byte;

	if (device_write_i2c_accmtr(device_addr, 1, &reg_addr, I2C_READ))
		return (-1);	//indicates failure
	if (device_read_i2c_accmtr(device_addr, 1, &read_byte, I2C_READ))
		return (-1);
	return (read_byte & 0x00FF); 	
}

/*******************************************************************************
// accmtr_config()
// handle a write to configure an accelorometer option.
*******************************************************************************/
int accmtr_config(void) //YS 17.8
{
	int 	res  = -1;
	BYTE	addr = parse_byte_num(g_tokens[2]);
	BYTE	data = parse_byte_num(g_tokens[3]);
		
	switch(addr) {
	case CONFIG_DATA_RATE:
		if (data == 1)
			accmtr_reg_set(CTRL_REG1, DR_MASK, DATA_RATE_100);
		else if (data == 2)
			accmtr_reg_set(CTRL_REG1, DR_MASK, DATA_RATE_200);
		else if (data == 4)
			accmtr_reg_set(CTRL_REG1, DR_MASK, DATA_RATE_400);
		else 
			return -1;
		res = 0;
		break;	
	case CONFIG_SCALING:
		if (data == 2)
			accmtr_reg_set(XYZ_DATA_CFG, FS_MASK, FULL_SCALE_2G);
		else if (data == 4)
			accmtr_reg_set(XYZ_DATA_CFG, FS_MASK, FULL_SCALE_4G);
		else if (data == 8)
			accmtr_reg_set(XYZ_DATA_CFG, FS_MASK, FULL_SCALE_8G);
		else 
			return -1;
		res = 0;
		break;	
	case CONFIG_MODE:
		if (data == 1)
			accmtr_reg_set(CTRL_REG2, MODS_MASK, NORMAL_WAKE);
		else if (data == 2) 
			accmtr_reg_set(CTRL_REG2, MODS_MASK, LOW_POWER_NOISE_WAKE);
		else if (data == 3) 
			accmtr_reg_set(CTRL_REG2, MODS_MASK, HR_WAKE);
		else if (data == 4) 
			accmtr_reg_set(CTRL_REG2, MODS_MASK, LOW_POWER_WAKE);
		else 
			return -1;
		res = 0;
		break;
	}
	return res;
}

/*******************************************************************************
// handle_accmtr()
// if first token was "accmtr", then handle accelerometer commands message:
// - according to sub command, call the relevant function
// - if command failed, display error message, and return error code
// - if command executed OK, display OK message and return 0.
*******************************************************************************/
int handle_accmtr(int sub_cmd) 
{
	int res = 0;

	// dispatch to the relevant sub command function according to sub command
	switch (sub_cmd) {
		case SUB_CMD_CONFIG:
			res = accmtr_config(); //YS 17.8
			break;
		default:
			err(ERR_INVALID_SUB_CMD);
			cmd_error(0);
			break;
	}
	if (res < 0)
		return cmd_error(0);
	cmd_ok();
	return 0;
}
#endif // #ifdef WISDOM_STONE
