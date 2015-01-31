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
we use Freescale MMA8451.Q accelerometer

*******************************************************************************/
#include "wistone_main.h"
#ifdef WISDOM_STONE

/***** INCLUDE FILES: *********************************************************/
#include "command.h"			// Application
#include "error.h"				// Application
#include "parser.h"				// Application 
#include "p24FJ256GB110.h"		// Common
#include "accelerometer.h"		// Devices
#include "i2c.h"				// Protocols
#include "usb_config.h"			// USB_UART
	
/***** GLOBAL VARIABLES: ******************************************************/
BYTE g_accmtr_blk_buff[CYCLIC_BUFFER_SIZE * MAX_BLOCK_SIZE]; // cyclic buffer space for SS and OST (TS uses only the first block of it) 	
BYTE g_accmtr_is_blk_rdy[CYCLIC_BUFFER_SIZE];				 // 1 - indicates that a complete lower g_accmtr_blk_buff half was filled by the sampler
WORD g_accmtr_blk_buff_w_ptr = 0;							 // write pointer used by ISR, pointing to g_accmtr_blk_buff indices - from 0 to 2 x MAX_BLOCK_SIZE - 1 
BYTE g_accmtr_overflow_cntr;		
BYTE g_accmtr_blk_overflow_cntr;	

/***** INTERNAL PROTOTYPES: ***************************************************/
int  accmtr_config(void);	// YS 17.8
void accmtr_reg_set(BYTE reg_addr, BYTE in_reg_addr, BYTE setting); 

/*******************************************************************************
* Function:
*		_INT2Interrupt()
* Description:
*		accelerometer ISR
* 		accelerometer settings are such that the only source for interrupt is FIFO's watermark 
* NOTE: for debug - we check overflow as well
*******************************************************************************/
void  __attribute__((interrupt, auto_psv)) _INT2Interrupt(void) {

 	int		i;
	BYTE 	fifo_status_reg;	
	BYTE 	numOfSamplesAvailable;
	BYTE 	blockNum;
	
	// mask PIC accelerometer interrupt:
	ACC_IE = 0;		// disable accelerometer PIC interrupt.											
	ACC_IF = 0;		// accelerometer PIC interrupt flag is cleared,
					// to indicate that the interrupt request has been accepted.											
	
	// read accelerometer status reg:
	start_i2c_accmtr(0);		// 0 (= ACCMTR_DEVICE_ADDRESS_WR) is added as last bit to indicate i2c write 
								// (i2c read sequence starts with writing the device address,
								// followed by writing the address within the device to read from).
								
	out_byte_i2c_accmtr(ACCMTR_DEVICE_ADDRESS_WR);		// INT_SOURCE<6> - SRC_FIFO bit, is accelerometer interrupt read only bit,
														// and it is cleared as a consequence of FIFO_STATUS_REG reading.
  	
	out_byte_i2c_accmtr(FIFO_STATUS_REG);						
	start_i2c_accmtr(1);
	out_byte_i2c_accmtr(ACCMTR_DEVICE_ADDRESS_RD);		// 1 (= ACCMTR_DEVICE_ADDRESS_RD) is added as last bit to indicate i2c read.		
	fifo_status_reg = in_byte_i2c_accmtr(1);			// fifo_status_reg is the accelerometer register with samples' info.
	stop_i2c_accmtr();
	
	// check for overflow bit:
	if (fifo_status_reg & F_OVF_MASK) 
		g_accmtr_overflow_cntr++; 
		
	// check for memory buffer overflow:
	#if (CYCLIC_BUFFER_SIZE == 2)
		if (g_accmtr_is_blk_rdy[0] & g_accmtr_is_blk_rdy[1])
			g_accmtr_blk_overflow_cntr++;
	#elif (CYCLIC_BUFFER_SIZE == 4)
		if (g_accmtr_is_blk_rdy[0] & g_accmtr_is_blk_rdy[1] & 
			g_accmtr_is_blk_rdy[2] & g_accmtr_is_blk_rdy[3])
			g_accmtr_blk_overflow_cntr++;
	#elif (CYCLIC_BUFFER_SIZE == 8)
		if (g_accmtr_is_blk_rdy[0] & g_accmtr_is_blk_rdy[1] & 
			g_accmtr_is_blk_rdy[2] & g_accmtr_is_blk_rdy[3] & 
			g_accmtr_is_blk_rdy[4] & g_accmtr_is_blk_rdy[5] & 
			g_accmtr_is_blk_rdy[6] & g_accmtr_is_blk_rdy[7])
			g_accmtr_blk_overflow_cntr++;
	#else
		#error "CYCLIC_BUFFER_SIZE should be 2,4 or 8"
	#endif
	
	// read the samples from the FIFO:
	// [since the watermark is the only interrupt source enabled,
	// there is no need to check here if it is set;
	// there are at least FIFO_WK_COUNT samples ready in the FIFO;
	// we read exactly 6 x FIFO_WK_COUNT bytes
	// (each sample consists of 6 bytes - 2 bytes for each axis)]
	
	numOfSamplesAvailable = fifo_status_reg & 0x3F;		// extract num of available samples
	start_i2c_accmtr(0);
	out_byte_i2c_accmtr(ACCMTR_DEVICE_ADDRESS_WR);
	out_byte_i2c_accmtr(FIFO_HEAD_REG);
	start_i2c_accmtr(1);
	out_byte_i2c_accmtr(ACCMTR_DEVICE_ADDRESS_RD);
	for (i = 0; i < (ACCMTR_SAMP_SIZE * numOfSamplesAvailable) - 1; i++) {
		g_accmtr_blk_buff[g_accmtr_blk_buff_w_ptr] = in_byte_i2c_accmtr(0);
		g_accmtr_blk_buff_w_ptr++;												// increment the g_accmtr_blk_buff position by 1
		// YL 9.12 ... was: 
		// if ((g_accmtr_blk_buff_w_ptr & 0x01FF) == (MAX_BLOCK_SIZE - 8)) {	// check if reach slightly before end of block (512 byte); writing 'x' to last 2 bytes
		// ... YL 9.12
		if ((g_accmtr_blk_buff_w_ptr & 0x01FF) == (LAST_DATA_BYTE + 1)) {		// check if last data byte was written; block tail starts with [RETRY | HW_OVERFLOW | SW_OVERFLOW] counters and is padded with 'x'
			blockNum = ((g_accmtr_blk_buff_w_ptr & (0xFE00)) >> 9);				// dividing g_accmtr_blk_buff_w_ptr by 512; blockNum indicates what buffer was filled
			g_accmtr_is_blk_rdy[blockNum] = 1;									// notify that block[blockNum] was filled and ready to be sent through usb/wireless, or saved to flash
			// YL 9.12 ... was:
			// g_accmtr_blk_buff[++g_accmtr_blk_buff_w_ptr] = g_accmtr_overflow_cntr;		// put in the 505 place of the current block the overflow status
			// g_accmtr_blk_buff[++g_accmtr_blk_buff_w_ptr] = g_accmtr_blk_overflow_cntr; 	// put in the 506 place of the current block the overflow cyclic buffer status			
			// ... YL 9.12
			g_accmtr_blk_buff[HW_OVERFLOW_LOCATION] = g_accmtr_overflow_cntr;		// the accelerometer overflow status	
			g_accmtr_blk_buff[SW_OVERFLOW_LOCATION] = g_accmtr_blk_overflow_cntr; 	// the cyclic buffer overflow status
			g_accmtr_overflow_cntr = 0;
			g_accmtr_blk_overflow_cntr = 0;
			// YL 9.12 ... was:
			// g_accmtr_blk_buff_w_ptr = ((g_accmtr_blk_buff_w_ptr + 6) & (CYCLIC_BLOCK_MASK));	// put the buffer pointer to the next block
			// ... YL 9.12
			g_accmtr_blk_buff_w_ptr = ((g_accmtr_blk_buff_w_ptr + BLOCK_TAIL_SIZE) & (CYCLIC_BLOCK_MASK));	// put the buffer pointer to the next block
		}
	}
	
	// read the last byte separately in order to stop I2C: // YL 9.12 same changes as above
	g_accmtr_blk_buff[g_accmtr_blk_buff_w_ptr] = in_byte_i2c_accmtr(1);						
	stop_i2c_accmtr();
	g_accmtr_blk_buff_w_ptr++;																
	if ((g_accmtr_blk_buff_w_ptr & 0x01FF) == (LAST_DATA_BYTE + 1)) {						
		blockNum = ((g_accmtr_blk_buff_w_ptr & (0xFE00)) >> 9);								
		g_accmtr_is_blk_rdy[blockNum] = 1; 													
		g_accmtr_blk_buff[HW_OVERFLOW_LOCATION] = g_accmtr_overflow_cntr; 				
		g_accmtr_blk_buff[SW_OVERFLOW_LOCATION] = g_accmtr_blk_overflow_cntr; 			
		g_accmtr_overflow_cntr = 0;
		g_accmtr_blk_overflow_cntr = 0;
		g_accmtr_blk_buff_w_ptr = ((g_accmtr_blk_buff_w_ptr + BLOCK_TAIL_SIZE) & (CYCLIC_BLOCK_MASK));	
	}
		
	// unmask PIC accelerometer interrupt:
	ACC_IE = 1;		
}

/*******************************************************************************
* Function:
*		init_accmtr()
* Description:
* 		initialize accelerometer:
*		- disable PIC and accelerometer interrupts to INT1
* 		- configure sensor for:
*			- FIFO watermark - the only reason for interrupt, routed to INT1 pin
*			- full (minimum) scale value range of is 2g by default (for maximum precision)	
*******************************************************************************/
void init_accmtr() {
	
	BYTE 	i;
	int 	check = accmtr_reg_read(WHO_AM_I_REG);	

	// check for Accelerometer component existence: // YL 9.12  NOTE - despite the following lines - the program gets stuck if the accelerometer isn't assembled
	if (check != (0x00FF & MMA8451_Q)) {
		err(ERR_ACCMTR_UNKNOWN_ID);
		return;
	}
	
	// PIC Interrupt Registers:
	ACC_IE = 0;					// IEC1<11> - external interrupt #1 is disabled
	INTCON2bits.INT2EP = 1;		// set Interrupt edge polarity to neg-edge										
	
	// Accelerometer Interrupt Registers:
	accmtr_reg_set(CTRL_REG4, INT_EN_FIFO_MASK, FIFO_INT_DIS);		// CTRL_REG4<6> - INT_EN_FIFO bit:
																	// 1 - FIFO interrupt enabled, 0 - FIFO interrupt disabled		
	
	// Accelerometer Data Registers:
	accmtr_standby();
	accmtr_reg_set(FIFO_SETUP_REG, F_MODE_MASK, FIFO_FILL);			// FIFO_SETUP_REG<6-7> - 2 F_MODE bits,
																	// set to 10 for FILL mode (FIFO stops accepting new samples when overflowed) 

	accmtr_reg_set(FIFO_SETUP_REG, F_WMRK_MASK, FIFO_WK_COUNT);		// FIFO_SETUP_REG<0-5> - 6 F_WMRK bits:
																	// FIFO event sample count watermark, currently set to 21

	accmtr_reg_set(CTRL_REG1, DR_MASK, DEFAULT_DATA_RATE);			// CTRL_REG1<3-5> - 3 DR bits: data rate selection; 
																	// CTRL_REG1<1> - select the Output Data Rate (ODR) for acceleration samples. // YS 17.8
	
	// YL 15.8 accmtr_reg_set(CTRL_REG2, MODS_MASK, HR_WAKE);		// CTRL_REG2<0-1> - 2 MODS bits = 10 for high resolution (14 bits) oversampling mode in WAKE mode
	
	// YL 15.8 accmtr_reg_set(CTRL_REG2, SMODS_MASK, HR_SLEEP);		// CTRL_REG2<3-4> - 2 SMODS bits = 11 for high resolution in SLEEP mode	
	
	accmtr_reg_set(CTRL_REG2, MODS_MASK, LOW_POWER_NOISE_WAKE);		// CTRL_REG2<0-1> - 2 MODS bits = 01 for low noise (14 bits) mode in WAKE mode // YL 15.8
	
	accmtr_reg_set(CTRL_REG2, SMODS_MASK, HR_SLEEP);				// CTRL_REG2<3-4> - 2 SMODS bits = 11 for high resolution in SLEEP mode // YL 15.8
	
	accmtr_reg_set(XYZ_DATA_CFG, FS_MASK, FULL_SCALE_2G);			// XYZ_DATA_CFG<0-1> - 2 FS bits = 00 for 2g full scale mode
	
	// Accelerometer Interrupt Registers: 
	accmtr_reg_set(CTRL_REG5, INT_CFG_FIFO_MASK, FIFO_INT1);		// CTRL_REG5<6> - INT_CFG_FIFO bit:
																	// 1 -  interrupt is routed to INT1 pin (0 - interrupt is routed to INT2 pin)
	
	// clear "double buffers" between accelerometer and TxRx (for OST only):
	for (i = 0; i < CYCLIC_BUFFER_SIZE ; i++) {
		g_accmtr_is_blk_rdy[i] = 0;
	}
}
		
/*******************************************************************************
* Function:
*		accmtr_standby()
* Description:
* 		accelerometer standby:
*		CTRL_REG1<0> - ACTIVE bit: 1 - active, 0 - standby 
*******************************************************************************/
void accmtr_standby() {	
		
	int 	res;	
	BYTE 	curr_setting;
	
	// read current value of control register #1:
	res = accmtr_reg_read(CTRL_REG1); 								
	if (res == -1) {
		err(ERR_ACCMTR_REG_READ);
		return;
	}
	curr_setting = res & 0xFF;
	
	// set standby mode by clearing the active bit:
	if (accmtr_reg_write(CTRL_REG1, curr_setting & (~ACTIVE_MASK)))	
		err(ERR_ACCMTR_REG_WRITE);	
	ACC_IE = 0;	// AY 10.8
	ACC_IF = 0;	// AY 10.8
}

/*******************************************************************************
* Function:
*		accmtr_active()
* Description:
* 		accelerometer active:
*		CTRL_REG1<0> - ACTIVE bit: 1 - active, 0 - standby 
* Return value:
*		-  0   - on success
*		- (-1) - on failure
*******************************************************************************/
// YL 14.9 ...
// was: void accmtr_active() {
int accmtr_active() {	
// ... YL 14.9		
	unsigned int	i;
	int 			res;	
	BYTE 			curr_setting;
	// YL 14.9 ... added to avoid preceding if accelerometer isn't assembled; NOTE - despite the following lines - the program gets stuck if the accelerometer isn't assembled
	int 			check = accmtr_reg_read(WHO_AM_I_REG);	

	// check for accelerometer component existence:
	if (check != (0x00FF & MMA8451_Q)) {
		return err(ERR_ACCMTR_UNKNOWN_ID);
	}
	// ... YL 14.9
	
	// clear "double buffers" between accelerometer and TxRx (for OST only):
	for (i = 0; i < CYCLIC_BUFFER_SIZE; i++) {
		g_accmtr_is_blk_rdy[i] = 0;
	}
	
	// reset error counters:
	g_accmtr_overflow_cntr = 0;
	g_accmtr_blk_overflow_cntr = 0;
	g_accmtr_blk_buff_w_ptr = 0;

	accmtr_reg_set(CTRL_REG4, INT_EN_FIFO_MASK, FIFO_INT_EN);	// CTRL_REG4<6> - INT_EN_FIFO bit:
																// 1 - FIFO interrupt enabled (0 - FIFO interrupt disabled)		
	ACC_IF = 0;	// AY 10.8
	ACC_IE = 1;	// AY 10.8
	
	// read current value of control register #1:
	res = accmtr_reg_read(CTRL_REG1); 								
	if (res == -1) {
		// YL 14.9 ...
		// was:
		// err(ERR_ACCMTR_REG_READ);		
		// return;
		return err(ERR_ACCMTR_REG_READ);
		// ... YL 14.9
	}
	curr_setting = res & 0xFF;
	
	// set active mode:
	if (accmtr_reg_write(CTRL_REG1, curr_setting | ACTIVE_MASK)) {	
		// YL 14.9 ...
		// was: err(ERR_ACCMTR_REG_WRITE);
		return err(ERR_ACCMTR_REG_WRITE);
		// ... YL 14.9
	}
	// YL 14.9 ...
	return 0;
	// ... YL 14.9
}

/*******************************************************************************
* Function:
*		accmtr_reg_set()
* Description:
* 		writes new setting to zeroed specified accelerometer register bits,
*		considering it's current setting
*******************************************************************************/
void accmtr_reg_set(BYTE reg_addr, BYTE mask, BYTE setting) {

	int 	res;	
	BYTE 	curr_setting;	

	// read current value of the register:
	res = accmtr_reg_read(reg_addr);								 
	if (res == -1) {	
		err(ERR_ACCMTR_REG_READ);
		return;
	}
	curr_setting = res & 0xFF;
	
	// zero specified registers' bits:
	curr_setting = (curr_setting & 0x00FF) & (~mask);
	
	// write new setting to previously zeroed registers' bits:	
	if (accmtr_reg_write(reg_addr, curr_setting | setting))			
		err(ERR_ACCMTR_REG_WRITE);	
}

/*******************************************************************************
* Function:
*		accmtr_reg_write()
* Description:
* 		write a single data Byte into specified accelerometer register address
* Return value:
*		-  0   - on success
*		- (-1) - on failure
*******************************************************************************/
int accmtr_reg_write(BYTE reg_addr, BYTE dat) {	//YL 15.9 added check to device write return value
	
	BYTE data_to_write[2];
	BYTE device_addr = ACCMTR_DEVICE_ADDRESS;
	
	data_to_write[0] = reg_addr;
	data_to_write[1] = dat;
	if (device_write_i2c_accmtr(device_addr, 2, data_to_write, I2C_WRITE))
		return (-1);	
	return 0;
}

/*******************************************************************************
* Function:
*		accmtr_reg_read()
* Description:
* 		read a single data Byte from specified accelerometer register address
* Return value:
*		-  0~255 - on success
*		- (-1) 	 - on failure
*******************************************************************************/
int accmtr_reg_read(BYTE reg_addr) {	//YL 15.9 added check to device read/write return value

	BYTE device_addr = ACCMTR_DEVICE_ADDRESS;
	BYTE read_byte;

	if (device_write_i2c_accmtr(device_addr, 1, &reg_addr, I2C_READ))
		return (-1);	
	if (device_read_i2c_accmtr(device_addr, 1, &read_byte, I2C_READ))
		return (-1);
	return (read_byte & 0x00FF); 	
}

/*******************************************************************************
* Function:
*		accmtr_config()
* Description:
* 		handle a write to configure an accelerometer option.
*******************************************************************************/
int accmtr_config(void) { //YS 17.8

	int res  = -1;
	
	// YL 5.8 changed parse_byte_num...
	// BYTE	addr = parse_byte_num(g_tokens[2]);
	// BYTE	data = parse_byte_num(g_tokens[3]);
	BYTE 	addr, data;
	
	// parse g_tokens[2] and return (-1) if it represents invalid byte: 
	res = parse_byte_num(g_tokens[2]);
	if (res == (-1)) {
		return (-1);
	}
	addr = 0xFF & res;
	
	// parse g_tokens[3] and return (-1) if it represents invalid byte:
	data = parse_byte_num(g_tokens[3]);
	if (res == (-1)) {
		return (-1);
	}
	data = 0xFF & res;
	//...YL 5.8
	
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
* Function:
*		handle_accmtr()
* Description:
* 		if first token was "accmtr", then handle accelerometer commands message:
*			- according to sub command, call the relevant function
*			- if command failed, display error message, and return error code
*			- if command executed OK, display OK message and return 0.
*******************************************************************************/
int handle_accmtr(int sub_cmd) {

	int res = 0;

	// dispatch to the relevant sub command function according to sub command:
	switch (sub_cmd) {
		case SUB_CMD_CONFIG:
			res = accmtr_config(); //YS 17.8
			break;
		default:
			err(ERR_UNKNOWN_SUB_CMD);
			cmd_error(0);
			break;
	}
	if (res < 0)
		return cmd_error(0);
	cmd_ok();
	return 0;
}
#endif // #ifdef WISDOM_STONE
