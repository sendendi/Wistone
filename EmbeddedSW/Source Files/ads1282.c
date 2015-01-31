/*******************************************************************************

ads1282.c - device driver for ADS1282 
==========-==========================

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
this file contains functions for operating the ADC: Texas Instruments ADS1282
using SW implemented semi-SPI protocol.
- register READ/WRITE 
- ...

*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include "wistone_main.h"
#include "error.h"				//Application
#include "HardwareProfile.h"	//Common
#include "ads1282.h"			//Devices

/***** GLOBAL VARIABLES: ******************************************************/
BYTE g_ads1282_blk_buff[CYCLIC_BUFFER_SIZE * MAX_BLOCK_SIZE];		//cyclic buffer space for SS and OST (TS uses only the first block of it) 	
BYTE g_ads1282_is_blk_rdy[CYCLIC_BUFFER_SIZE];						//1 - indicates that a complete lower g_ads1282_blk_buff half was filled by the sampler
WORD g_ads1282_blk_buff_w_ptr;										//write pointer used by ISR, pointing to g_ads1282_blk_buff indices - from 0 to 2 x MAX_BLOCK_SIZE - 1 
BYTE g_ads1282_blk_overflow_cntr;
BOOL g_is_ads1282_active = FALSE;

/***** INTERNAL PROTOTYPES: ***************************************************/
void ads1282_byte_delay(void);

/*******************************************************************************
// _INT3Interrupt()
// ADS1282 DataReady Interrupt handler - read single DWORD sample into block buffer
//
// occurs when data word (32bit) of a single sample is ready inside the ADC.
// triggered by DRDY pin (active low)
// - generate 32 pulses of sclk:
//		- sclk = 1
//		- sample <- din
//		- additional delay for minimum sclk pulse width of 500nSec
//		- sclk = 0
//		- additional delay to have minimum sclk cycle
// - insert the sample as 4 Bytes into the block buffer
// - update wprt cyclicly and check for overflow
// - if reached end of block (504 bytes)
//		- update block ready flag
//		- add overflow information
*******************************************************************************/
void  __attribute__((interrupt, auto_psv)) _INT3Interrupt(void) { //YL 8.12 TODO PSV model not specified for '_INT3Interrupt'
	int			i;
	BYTE		blk_num;
	DWORD		sample;											//32-bit unsigned

	ADS1282_IE = 0;												// disable ADS1282 DRDY interrupt
	ADS1282_IF = 0;													// DRDY interrupt flag is cleared, to indicate that the interrupt request has been accepted 

	// generate 32 cycles of sclk, and sample Din into a single DWORD sample:
	sample = 0x00000000;
	for (i = 0; i < ADS1282_SAMP_SIZE; i++) {
		ADS1282_SCLK_LAT = 1;
		sample = (sample << 1) | DOUT_1282_MCU_PORT;
		Nop();
		Nop();
		ADS1282_SCLK_LAT = 0;
		Nop();
		Nop();
		Nop();
		Nop();
	}
	// insert the sample as 4 single Bytes into the block buffer:
	g_ads1282_blk_buff[g_ads1282_blk_buff_w_ptr    ] = (sample & 0xFF000000) >> 24;
	g_ads1282_blk_buff[g_ads1282_blk_buff_w_ptr + 1] = (sample & 0x00FF0000) >> 16;
	g_ads1282_blk_buff[g_ads1282_blk_buff_w_ptr + 2] = (sample & 0x0000FF00) >> 8;
	g_ads1282_blk_buff[g_ads1282_blk_buff_w_ptr + 3] =  sample & 0x000000FF;

	// check for overflow:
	#if (CYCLIC_BUFFER_SIZE == 2)
		if (g_ads1282_is_blk_rdy[0] & g_ads1282_is_blk_rdy[1])
			g_ads1282_blk_overflow_cntr++;
	#elif (CYCLIC_BUFFER_SIZE == 4)
		if (g_ads1282_is_blk_rdy[0] & g_ads1282_is_blk_rdy[1] & g_ads1282_is_blk_rdy[2] & g_ads1282_is_blk_rdy[3])
			g_ads1282_blk_overflow_cntr++;
	#elif (CYCLIC_BUFFER_SIZE == 8)
		if (g_ads1282_is_blk_rdy[0] & g_ads1282_is_blk_rdy[1] & g_ads1282_is_blk_rdy[2] & g_ads1282_is_blk_rdy[3] & g_ads1282_is_blk_rdy[4] & g_ads1282_is_blk_rdy[5] & g_ads1282_is_blk_rdy[6] & g_ads1282_is_blk_rdy[7])
			g_ads1282_blk_overflow_cntr++;
	#else
		#error "CYCLIC_BUFFER_SIZE should be 2,4 or 8"
	#endif

	// update wptr cyclicly:
	g_ads1282_blk_buff_w_ptr = g_ads1282_blk_buff_w_ptr + 4; //sample size is 4 bytes
	// check for block completion:
	if ((g_ads1282_blk_buff_w_ptr & 0x01FF) >= (MAX_BLOCK_SIZE - 8)) {				//if reached max num of bytes in block	
		blk_num = ((g_ads1282_blk_buff_w_ptr & (0xFE00)) >> 9);						//dividing g_ads1282_blk_buff_w_ptr by 512	
		g_ads1282_is_blk_rdy[blk_num] = 1; 											//notify that block[blk_num] was filled and ready to be sent through usb/wireless, or saved to flash
		g_ads1282_blk_buff[g_ads1282_blk_buff_w_ptr + 2] = g_ads1282_blk_overflow_cntr; 			//put in the 506 place of the current block the overflow cyclic buffer status
		g_ads1282_blk_overflow_cntr = 0;
		g_ads1282_blk_buff_w_ptr = ((g_ads1282_blk_buff_w_ptr + 8) & (CYCLIC_BLOCK_MASK));	//put the buffer pointer to the next block
	}

	ADS1282_IE = 1;													// enable ADS1282 DRDY interrupt
}

/*******************************************************************************
// init_ads1282()
// initialize the ADS1282 and ints environment
// this is done on Wistone power-up sequence, when ADS1282 should be turned off.
// therefore, we don't need to init its registers.
// - check for presence (WHO_AM_I)
// - switch ADS1282 to standby mode
// - init signals and GPIOs
// - init global variables and flags
// - disable interupt
*******************************************************************************/
int init_ads1282(void) {
	
	//YL 8.12...
	//BYTE		who_am_i;
	char		who_am_i;
	char*		id_reg = "2000";
	char*		id_reg_reply;
	//...YL 8.12
	
	// check for ADS1282 component presence:
	//YL 8.12 instead of the commented:...
	//who_am_i = ads1282_reg_read(ID_REG, 1); 
	//if ((who_am_i & ID_MASK) == 0x00) { //4 upper bits in ID_REG are read only ID bits
		//return(err(ERR_ADS1282_UNKOWN_ID));
	if (ads1282_reg_read(id_reg, id_reg_reply) == 0) {
		who_am_i = ((id_reg_reply[0]) & ID_MASK); //4 upper bits in ID_REG are read only ID bits
		if (who_am_i = 0x00) 
			return(err(ERR_ADS1282_UNKOWN_ID));
	}
	else
		return err((ERR_ADS1282_REG_READ));
	//...YL 8.12

	g_is_ads1282_active = FALSE;		// flag used in Timer4 ISR to generate SYNC signal
	// init GPIOs values:
	DOUT_1282_MCU_TRIS = 1;		// Dout for ADS1282 is the Din for the PIC	
	DIN_MCU_1282_TRIS  = 0;		// Din for PIC is the Dout of the ADS1282
	ADS1282_SCLK_TRIS  = 0;
	ADS1282_SCLK_LAT   = 0;
	ADS1282_SYNC_TRIS  = 0;
	ADS1282_SYNC_LAT   = 0;
	ADS1282_PWDN_TRIS  = 0;
	ADS1282_RESET_TRIS = 0;
	ADS1282_PWDN_LAT   = 1;			// power-up: power done mode disable (active low)
	ADS1282_RESET_LAT  = 1;			// unreset: resetn disable (active low)

	// ADS1282 DRDY Interrupt Service Routine:
	ADS1282_IE = 0;					// IEC3<5> - external interrupt #3 is disabled
	INTCON2bits.INT3EP = 1;			// set Interrupt edge polarity to neg-edge
	ADS1282_IF = 0;					// clear flag
}

/*******************************************************************************
// ads1282_active()
// when received "app start" command, activate the ADS1282
//
// - check existance of required power sources: 5v, 12v, -12v
// - configure internal registers for defaults
// - enable INT3 ISR to handle sample reads
// - init gobal variables
*******************************************************************************/
int ads1282_active(void)
{
	char		reply[(ADS1282_MAX_NUM_OF_BYTES * 2) + 1];

	// mke sure we have the required power turned on: 12v, -12v, 5v
	if ((PWR_EN_M12V != 1) || (PWR_EN_12V != 1) || (PWR_EN_5V != 1));
		return(err(ERR_ADS1282_MISSING_POWER));
	// init global variables:
	g_ads1282_blk_overflow_cntr = 0;
	g_ads1282_blk_buff_w_ptr = 0;
	// check the existance of the ADS1282 chip in the system;
	// read ID register: RREG addr: 0, len: 1 
	if (ads1282_reg_read("2000", reply) != 0)
		return(err(ERR_ADS1282_UNKOWN_ID));
	// sanity check: verify that the ID is not all zero or all one bits...
	if ((reply[0] == '0') || (reply[0] == 'F'))
		return(err(ERR_ADS1282_UNKOWN_ID));
	// configure internal registers to defaults:
	// ...  example: ads1282_write_reg("4101AABB") - write AA to addr 0x01, write BB to addr 0x02
	// (no need to configure registers since we use the default mode of operation for the ADS1282

	// enable INT3 ISR:
	ADS1282_IF = 0;						
	ADS1282_IE = 1;						// enable ISR for ADS1282

	return(0);
}

/*******************************************************************************
// ads1282_standby()
// - de-activate required power sources: 5v, 12v, -12v
*******************************************************************************/
void ads1282_standby(void)
{
	ADS1282_IE = 0;						// disable ISR for ADS1282
	// inform user about power saving:
	m_write("ADS1282 is not active. Consider turn power off: 5v, 12v, -12v.\r\n");
}

/*******************************************************************************
// ads1282_reg_write()
// write byte sequence to ADS1282
// the byte sequence contains: commad (+ data)
// notes:
// - a delay of 24 f_clk cycles (6 uSec) is required between each byte transaction
// - sclk minimum cycle is 2 f_clk (0.5 uSec)
// - ADS1282 samples the data at the rising edge of Sclk
// input:
// - string, null terminated - command (+ data) - HEX represented command bytes and data bytes.
//		(normaly 2 bytes for command, and the rest of bytes for data, represented by x2 HEX characters)
// return:
// -  0   - on success
// - (-1) - on failure
*******************************************************************************/
int ads1282_reg_write(char *command_data)
{	
	int			i;
	BYTE		current_byte, current_bit;

	// go over the command_data string:
	i = 0;
	while ((command_data[i] != 0) && (i < ((ADS1282_MAX_NUM_OF_BYTES + 2) + 2 + 1))) {

		// extruct current byte:
		// IMPORTANT: make sure this code takes at least 6uSec
		// extruct and convert first HEX digit (MSB), allow upper and lower case:
		if ((command_data[i] >= 'a') && (command_data[i] <= 'f'))
			current_byte = (command_data[i] - 'a') << 4;
		else if ((command_data[i] >= 'A') && (command_data[i] <= 'F'))
			current_byte = (command_data[i] - 'A') << 4;
		else if ((command_data[i] >= '0') && (command_data[i] <= '9'))
			current_byte = (command_data[i] - '0') << 4;
		else
			return(-1);
		// extruct and convert second HEX digit (LSB), allow upper and lower case:
		if ((command_data[i] >= 'a') && (command_data[i] <= 'f'))
			current_byte = current_byte | (command_data[i] - 'a');
		else if ((command_data[i] >= 'A') && (command_data[i] <= 'F'))
			current_byte = current_byte | (command_data[i] - 'A');
		else if ((command_data[i] >= '0') && (command_data[i] <= '9'))
			current_byte = current_byte | (command_data[i] - '0');
		else
			return(-1);
		
		// transmit the current byte:
		for (i = 0; i < 8; i++) {
			DIN_MCU_1282_LAT = (current_byte >> 7) & 0x01;
			current_byte = current_byte << 1;					// this command is placed here for Tsetup
			ADS1282_SCLK_LAT = 1;
			Nop();												// "high" time of Sclk
			Nop();
			Nop();
			Nop();
			ADS1282_SCLK_LAT = 0;
			Nop();												// "low" time of Sclk
			Nop();
		}
		
		// inter-byte delay of 24 f_clk cycles:
		// since we have the above code for "ectruct current byte", 
		// we only need to complete it to 24 clks x 0.25 uSec = 6 uSec:
		// TO DO: need to add / remove some Nop()...
		Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();

		i = i + 2;
	}

	return 0;
}

/*******************************************************************************
// ads1282_reg_read()
// read byte sequence from ADS1282
// the number of bytes to read is extructed from the command
// - a delay of 24 f_clk cycles (6 uSec) is required between each byte transaction
// - sclk minimum cycle is 2 f_clk (0.5 uSec)
// - ADS1282 samples the data at the rising edge of Sclk
// - ADS1282 output the data at the falling edge of Sclk
// input:
// - string, null terminated - command - HEX represented command bytes:
//		001rrrrr 000nnnnn represented as: HHHH (4 HEX digits)
// output:
// - string, null terminated - command - HEX represented reply
// return:
// - 0~255 - on success
// - (-1)  - on failure
*******************************************************************************/
int  ads1282_reg_read(char *command, char* reply)
{
	BYTE		MSnibble, LSnibble, reply_len, i, j, current_byte;
 	char 		dec2hex[17] = "0123456789ABCDEF";		// decimal to Hex conversion LUT
	// first, we issue the command HEX sequence so we can use the reg_write function:
	if (ads1282_reg_write(command) != 0)
		return(-1);

	// now, we expect to read the replyed bytes, as we generate Sclk
	// extruct the number of expected bytes in reply from the second command byte
	if ((command[2] >= 'a') && (command[2] <= 'f'))
		MSnibble = (command[2] - 'a') << 4;
	else if ((command[2] >= 'A') && (command[2] <= 'F'))
		MSnibble = (command[2] - 'A') << 4;
	else if ((command[2] >= '0') && (command[2] <= '9'))
		MSnibble = (command[2] - '0') << 4;
	else
		return(-1);
	MSnibble = MSnibble & 0x10;
	// extruct and convert second HEX digit (LSB), allow upper and lower case:
	if ((command[3] >= 'a') && (command[3] <= 'f'))
		LSnibble = command[3] - 'a';
	else if ((command[3] >= 'A') && (command[3] <= 'F'))
		LSnibble = command[2] - 'A';
	else if ((command[3] >= '0') && (command[3] <= '9'))
		LSnibble = command[3] - '0';
	else
		return(-1);
	// add 1 to the number (look at the datasheet):
	reply_len = (MSnibble | LSnibble) + 1;
	
	// read the reply of reply_len byte:
	for (i = 0; i < reply_len; i++) {
		// read a single byte:
		// generate 8 cycles of sclk, and sample Din into current_byte:
		current_byte = 0x00;
		for (j = 0; j < 8; j++) {
			ADS1282_SCLK_LAT = 1;
			current_byte = (current_byte << 1) | DOUT_1282_MCU_PORT;
			Nop();
			Nop();
			ADS1282_SCLK_LAT = 0;
			Nop();
			Nop();
			Nop();
			Nop();
		}
		// convert the current_byte to 2 HEX digits, and append to reply string:
		reply[ i * 2     ] = dec2hex[(current_byte & 0xF0) >> 4];		// MS nibble
		reply[(i * 2) + 1] = dec2hex[ current_byte & 0x0F      ];		// LS nibble
		// complete to delay of 24 f_clks => 6 uSec
		for (j = 0; j < 8; j++)
			Nop();
	}		
	// append end of string after the HEX digits:
	reply[reply_len * 2] = 0;

	return (0); 	
}

/*******************************************************************************
// handle_ads1282()
*******************************************************************************/
/*
int  handle_ads1282(int sub_cmd) {
	int res = 0;

	// dispatch to the relevant sub command function according to sub command
	switch (sub_cmd) {
		case SUB_CMD_ON:
			break;
		
		case SUB_CMD_OFF:
			break;

		default:
			err(ERR_UNKNOWN_SUB_CMD);
			cmd_error(0);
			break;
	}
	if (res < 0)
		return cmd_error(0);
	cmd_ok();

	return(0);
}
*/
