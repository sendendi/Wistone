/*******************************************************************************

app.c - handle application tasks
=================================

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
this file contains functions for handling application tasks.
- start sampling/storing/transmiting according to:
	- <mode> = SS - Sample and Store mode for num_of_blocks x 0.5KB samples
	- <mode> = TS - Transmit Samples mode for num_of_blocks x 0.5KB samples
	- <mode> = OST - Online Sample and Transmit mode for num_of_blocks x 0.5KB samples
- app stop
- app sleep
using HW implemented SPI1 module in PIC to access the FLASH.
*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
#include "wistone_main.h"
#ifdef WISDOM_STONE 
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

#include "app.h"				//Application
#include "command.h"			//Application
#include "error.h"				//Application
#include "parser.h"				//Application
#include "misc_c.h"				//Common	
#include "p24FJ256GB110.h"		//Common	
#include "accelerometer.h"		//Devices
#include "ads1282.h"			//Devices	
#include "flash.h"				//Devices
#include "rtc.h"				//Devices
#include "TxRx.h"				//TxRx - Application
#include "TimeDelay.h"			//TxRx - Common
//#include "P2P.h"				//TxRx - Protocols
#include "Compiler.h"
#include "wistone_usb.h"

/***** GLOBAL VARIABLES: ******************************************************/
int 	g_mode = MODE_IDLE;
int 	g_destination = DEST_NONE; 			
long 	g_num_of_blocks;
long	g_start_sector_addr;
long 	g_accmtr_sector_addr_ptr;
long	g_ads1282_sector_addr_ptr;
long	g_sector_addr_ptr;
int 	g_single_dual_mode;
BYTE 	g_accmtr_next_printed_blk;
BYTE 	g_ads1282_next_printed_blk;
long	g_accmtr_num_of_blocks;

/***** INTERNAL PROTOTYPES: ***************************************************/
int 	handle_application_start(void);	
void 	handle_application_stop(void);	
int 	handle_active_mode(void); 
void 	send_start_block(void);
int 	sampler_start(void);		
void 	sampler_stop(void);	
void 	handle_application_sleep(void); 	

/******************************************************************************
* Function:
*		void init_block_buffers()
*
* Description:
*      This is the primary user interface function to initialize the block buffers
*	   that are used for the different apllication modes (Store samples, transmit samples
*	   and online sample transmit). 
*	   The number of block buffers should be choosen in CYCLIC_BUFFER_SIZE. You can
*	   choose this value to be either 2 or 4. There is a trade-off when choosing
*	   this value - on the one hand, 4 will consume much more space, and on the 
*	   other hand it will prevent from sampling to get lost..
*	   do the same for both ADC and ACCMTR buffers.
* Example:
*      <code>
*      Hardware_Init();
*	   init_block_buffres();
*      </code>
*
* Parameters:
*	   None.
*
* Return value: 
*	   None.
*
******************************************************************************/
void init_block_buffers(){

	BYTE 	i;
	int		j;

	// clear block-ready flags:
	for (i = 0; i < CYCLIC_BUFFER_SIZE; i++) {	//every block_buffer should point to the appropirate place of the cyclic_buffer
		g_accmtr_is_blk_rdy[i] = 0;				//none of the blocks are ready to print in the beggining
		g_ads1282_is_blk_rdy[i] = 0;
	}
	// clear block buffer data, and trailer:
	for (i = 0; i < CYCLIC_BUFFER_SIZE; i++) {
		for (j = (MAX_BLOCK_SIZE - 8); j < MAX_BLOCK_SIZE; j++) {	
			g_accmtr_blk_buff[(i * MAX_BLOCK_SIZE) + j] = 'x'; 	//pad the buffers with 'x'-s.
			g_ads1282_blk_buff[(i * MAX_BLOCK_SIZE) + j] = 'x';
		}
		g_accmtr_blk_buff[(i * MAX_BLOCK_SIZE) + 505] = 0;		//place zero where we save the overflow counters
		g_accmtr_blk_buff[(i * MAX_BLOCK_SIZE) + 506] = 0;
		g_ads1282_blk_buff[   (i * MAX_BLOCK_SIZE) + 505] = 0;
		g_ads1282_blk_buff[   (i * MAX_BLOCK_SIZE) + 506] = 0;
	}
	// cyclic pointers for the next block to print:
	g_accmtr_next_printed_blk = 0;
	g_ads1282_next_printed_blk = 0;
}

/*******************************************************************************
// handle_SS()
// handle Sample and Store mode:
// - copy blocks filled by sampler into FLASH; 
// - stop copying if all assigned FLASH SS memory is full or if all blocks were copied.
// - count Accelerometer blocks, and igore counting ADC blocks (since sample frequency is same)
*******************************************************************************/
void handle_SS(void)
{	
	// go cyclicly over the is_blk_ready[] flags of the blocks
	// for each ready blocks, write it to FLASH
	// start with ADC blocks:
	while (g_ads1282_is_blk_rdy[g_ads1282_next_printed_blk] == 1) {						// if the next_block is filled by the sampler, then it is ready to be saved to the flash.
		flash_write_sector(g_ads1282_sector_addr_ptr, &g_ads1282_blk_buff[MAX_BLOCK_SIZE * g_ads1282_next_printed_blk]); 		//transmit block from memory buffer //.... TODO: should be checked? what should be done if flash_write fails? 
		g_ads1282_is_blk_rdy[g_ads1282_next_printed_blk] = 0;							// the next_block was saved to the flash, therefore the sampler can use it again.
		g_ads1282_next_printed_blk = (g_ads1282_next_printed_blk + 1) % (CYCLIC_BUFFER_SIZE);	// g_ads1282_next_printed_blk points to the next block..
		g_ads1282_sector_addr_ptr++;
	}
	// continue with Accmtr blocks:
	while (g_accmtr_is_blk_rdy[g_accmtr_next_printed_blk] == 1) {					// if the next_block is filled by the sampler, then it is ready to be saved to the flash.
		flash_write_sector(g_accmtr_sector_addr_ptr, &g_accmtr_blk_buff[MAX_BLOCK_SIZE * g_accmtr_next_printed_blk]); 		//transmit block from memory buffer //.... TODO: should be checked? what should be done if flash_write fails? 
		g_accmtr_is_blk_rdy[g_accmtr_next_printed_blk] = 0;						// the next_block was saved to the flash, therefore the sampler can use it again.
		g_accmtr_next_printed_blk = (g_accmtr_next_printed_blk + 1) % (CYCLIC_BUFFER_SIZE);	// g_accmtr_next_printed_blk points to the next block..
		g_accmtr_sector_addr_ptr++;
		// check if completed requested number of blocks:
		g_accmtr_num_of_blocks--;
		if (g_accmtr_num_of_blocks <= 0)
			handle_application_stop();
	}
}	

/*******************************************************************************
// handle_TS()
// handle Transmit Samples mode:
// read and transmit samples stored in FLASH.
*******************************************************************************/
void handle_TS(void)
{ 
	int				i, res = 0;
	static BOOL		toggleTS = TRUE; 									//toggle reading and transmitting each main iteration
	TXRX_ERRORS		status;
	
	if (toggleTS == TRUE) {
		res = flash_read_sector(g_sector_addr_ptr, g_accmtr_blk_buff);	//read 1 block from FLASH into start of the buffer
		DelayMs(100); //YS 10.11 - mail instruction 
		if (res < 0) {													//retry read from FLASH
			res = flash_read_sector(g_sector_addr_ptr, g_accmtr_blk_buff);//read 1 block from FLASH into start of the buffer
			if (res < 0) { 												//if still fails, fill buffer with FF
				for (i = 0; i < FLASH_SECTOR_SZ; i++) 					//if the reading failed - transmit block filled with 0xFF 
					g_accmtr_blk_buff[i] = 0xFF;						// indicating FLASH read failure
			}
		}
		g_sector_addr_ptr++;
	}
	else {
		if (g_destination == DEST_WIRELESS) {				
			g_accmtr_blk_buff[504] = blockTryTxCounter;					//put the number of transmission needed in the previous block..
			blockTryTxCounter = 0;//YS 25.1
			status = TxRx_SendData(g_accmtr_blk_buff, MAX_BLOCK_SIZE);
			while (status != TXRX_NO_ERROR){//YS 17.11
				TxRx_printError(status);
				//TxRx_Init(TRUE);//YS 5.10 //YS 17.11
				status = TxRx_SendData(g_accmtr_blk_buff, MAX_BLOCK_SIZE);//YS 17.11
			}
			//YS 25.1
		}
		else // DEST_USB
			b_write(g_accmtr_blk_buff, MAX_BLOCK_SIZE);				
		g_num_of_blocks--;
	}
	if (g_num_of_blocks <= 0)
		handle_application_stop();
	toggleTS = !toggleTS;	
}

/*******************************************************************************
// handle_OST()
// handle Online Sample and Transmit mode:
// - go over the blocks and check if ready
// - if ready, try to TX the block up to two times
// - if second time failed, skip current block
// - do the same for Acceleroter and ADS1282
*******************************************************************************/
void handle_OST(void)
{	 
	TXRX_ERRORS		status;
	
	// go over all the ADC blocks that are in the cyclic buffer, starting from the g_accmtr_next_printed_blk.
	while (g_ads1282_is_blk_rdy[g_ads1282_next_printed_blk] == 1) { 			//if the next_block is filled by the sampler, then it is ready to be saved to the flash.
		if (g_destination == DEST_WIRELESS) {						//if the destination of the sending is wireless
			g_ads1282_blk_buff[(g_ads1282_next_printed_blk * MAX_BLOCK_SIZE) + 504] = blockTryTxCounter;	//put the number of transmission needed in the previous block..
			blockTryTxCounter = 0;//YS 25.1
			status = TxRx_SendData(&g_ads1282_blk_buff[g_ads1282_next_printed_blk * MAX_BLOCK_SIZE], MAX_BLOCK_SIZE);	//send the data through the wireless.
			while (status != TXRX_NO_ERROR) {
				TxRx_printError(status);
				//TxRx_Init(TRUE); 	//YS 5.10 //YS 17.11
				status = TxRx_SendData(&g_accmtr_blk_buff[g_ads1282_next_printed_blk * MAX_BLOCK_SIZE], MAX_BLOCK_SIZE);//YS 17.11
			}
			//YS 25.1
		}		
		else{//DEST_USB
			b_write(&g_ads1282_blk_buff[g_ads1282_next_printed_blk * MAX_BLOCK_SIZE], MAX_BLOCK_SIZE);			//else print it through the usb.
		}
		g_ads1282_is_blk_rdy[g_ads1282_next_printed_blk] = 0;
		g_ads1282_next_printed_blk = (g_ads1282_next_printed_blk + 1) % (CYCLIC_BUFFER_SIZE);					//g_ads1282_next_printed_blk points to the next block..
	}
	
	// go over Accmtr blocks:
	while (g_accmtr_is_blk_rdy[g_accmtr_next_printed_blk] == 1) { 		//if the next_block is filled by the sampler, then it is ready to be saved to the flash.
		if (g_destination == DEST_WIRELESS) {						//if the destination of the sending is wireless
			g_accmtr_blk_buff[(g_accmtr_next_printed_blk * MAX_BLOCK_SIZE) + 504] = blockTryTxCounter;	//put the number of transmission needed in the previous block..
			blockTryTxCounter = 0;//YS 25.1
			status = TxRx_SendData(&g_accmtr_blk_buff[g_accmtr_next_printed_blk * MAX_BLOCK_SIZE], MAX_BLOCK_SIZE);	//send the data through the wireless.
			while (status != TXRX_NO_ERROR) {//YS 17.11
				TxRx_printError(status);
				//TxRx_Init(TRUE); 	//YS 5.10 //YS 17.11
				status = TxRx_SendData(&g_accmtr_blk_buff[g_accmtr_next_printed_blk * MAX_BLOCK_SIZE], MAX_BLOCK_SIZE);//YS 17.11
			}
			//YS 25.1
		}		
		else {//DEST_USB
			b_write(&g_accmtr_blk_buff[g_accmtr_next_printed_blk * MAX_BLOCK_SIZE], MAX_BLOCK_SIZE);	//else print it through the usb.
		}
		g_accmtr_num_of_blocks--;
		g_accmtr_is_blk_rdy[g_accmtr_next_printed_blk] = 0;
		g_accmtr_next_printed_blk = (g_accmtr_next_printed_blk + 1) % (CYCLIC_BUFFER_SIZE);				//g_accmtr_next_printed_blk points to the next block..
		if(g_accmtr_num_of_blocks <= 0){							//if we sampled the amount of blocks needed, then stop the sampler and go to IDLE mode.
			handle_application_stop();
			return;
		}
	}
}

/*******************************************************************************
// handle_application()
// if first token was "app", then handle application commands message:
// - according to sub command, call the relevant function
// - if command failed, display error message, and return error code
// - if command executed OK, display OK message and return 0.
*******************************************************************************/
int handle_application(int sub_cmd)	
{
	// dispatch to the relevant sub command function according to sub command
	switch (sub_cmd) {
		case SUB_CMD_START:
			g_boot_seq_pause = TRUE; // pause reading next boot commands (if any...)
			if (handle_active_mode())
				return(-1);
			break;
		case SUB_CMD_STOP:
			g_mode = MODE_IDLE;
			handle_application_stop();
			cmd_ok();
			break;
		case SUB_CMD_SLEEP:
			handle_application_sleep();
			cmd_ok();
			break;
		case SUB_CMD_SHUTDOWN:
			prepare_for_shutdown();
			cmd_ok();
			break;
	}

	return(0);
}

/*******************************************************************************
// handle_active_mode()
// dispatch to relevant handling function according to appropriate mode
// general app start cmd structure: app start <mode> <start sector address> <num of blocks> <destination> 
*******************************************************************************/
int handle_active_mode(void) 
{
	if ((g_ntokens != 5) && (g_ntokens != 6))					//app start may be called only with 6 params //YL 7.11
		return (err(ERR_INVALID_PARAM_COUNT));
	g_mode = parse_mode(g_tokens[2]);
	g_num_of_blocks = parse_long_num(g_tokens[3]);
	init_block_buffers();
	switch (g_mode) {
	case MODE_OST:
		g_destination = parse_destination(g_tokens[4]);
		g_single_dual_mode = parse_single_dual_mode(g_tokens[5]);	// single sensor or dual sensors to sample
		g_accmtr_num_of_blocks = g_num_of_blocks;
		break;
	case MODE_TS:
		g_start_sector_addr = parse_long_num(g_tokens[4]);		//g_start_sector_addr is relevant only in TS and SS modes
		g_destination = parse_destination(g_tokens[5]);
		g_sector_addr_ptr = g_start_sector_addr;
		break;
	case MODE_SS:
		g_start_sector_addr = parse_long_num(g_tokens[4]);		//g_start_sector_addr is relevant only in TS and SS modes
		g_single_dual_mode = parse_single_dual_mode(g_tokens[5]);	// single sensor or dual sensors to sample
		g_accmtr_num_of_blocks = g_num_of_blocks;
		g_accmtr_sector_addr_ptr = g_start_sector_addr;					// Accelermeter storage starts at the required sector number
		g_ads1282_sector_addr_ptr = FLASH_SECTOR_ADS1282_OFFSET;				// ADS1282 storage starts always at FLASH_SIZE / 2
		break;
	default:
		break; // should not get here...
	}	
	// sanity checks:
	if ((g_start_sector_addr < 0) || (g_num_of_blocks <= 0) || ((g_destination < 0) && (g_mode != MODE_SS)) || (g_single_dual_mode < 0)) {
		g_mode = MODE_IDLE;	//default
		return(err(ERR_INVALID_PARAM));
	}

	if (handle_application_start() != 0)
		return(-1);

	return(0);
}

/*******************************************************************************
// handle_application_start()
*******************************************************************************/
int handle_application_start(void)	
{
	send_start_block();				// we generate a single header block even when dual mode is used
	if (g_mode == MODE_SS || g_mode == MODE_OST) {
		if (sampler_start() != 0)
			return(-1);
	}
}

/*******************************************************************************
// handle_application_stop()
// general app stop cmd structure: app stop
// do not change the following strings prefixes, since the GUI is looking for them.
*******************************************************************************/
void handle_application_stop(void)	
{
	if (g_mode == MODE_SS || g_mode == MODE_OST)
		sampler_stop();
	
	write_eol();
	switch (g_mode) {
		case MODE_SS:
			m_write ("SSCOMPLETED: completed storing the requested num of blocks");
			break;
		case MODE_TS:
			m_write ("TSCOMPLETED: completed transmitting the requested num of blocks");
			break;
		case MODE_OST:
			m_write	("OSTCOMPLETED: completed transmitting the requested num of blocks");
			break;
		case MODE_IDLE:
			m_write ("APPSTOPPED: application stopped");
			break;
	}
	write_eol();
	g_mode = MODE_IDLE;				//switch to idle when handle active mode is complete
	g_boot_seq_pause = FALSE; 	//resume reading next boot commands (if any...)
}

/******************************************************************************
* Function:
*		void handle_application_sleep()
*
* Description:
*      This function is the primary funcion for handling app sleep command.
*	   This function puts all the peripherial devices to the lowest ppower mode,
*	   and finnaly puts the MCU to sleep. 
*	   The MCU wakes up when an interrupt comes from the MRF or RTC. If the interrupr
*	   comes from the MRF, then we check that the command that is received is 
*	   "app wake".
*
* Parameters:
*	   None.
*
* Return value: 
*	   None.
*
******************************************************************************/
void handle_application_sleep(){
	TXRX_ERRORS TxRxStatus;

	m_write("Going to sleep...\r\n");
	// TODO: Put all the devices to sleep..
	sampler_stop(); // Puts the sampler to standby mode - its lowest mode
	// TxRx_SetLowPowerMode();
	// The watch-dog timer is off  (in OSC1 configoration register FWDTEN_OFF)

	while (1){	// while we not received "app wake"
		// Puts the MCU to sleep
		Sleep();
	
		m_write("I have waken up for some reason\r\n");
		// Next code will occur only if there was an interrupt (or reset, but then all the system is reset)
		// This interrupt can be from eithr MRF or rtc (to check if RTC can produce inerrupt)
			
		DelayMs(1000); // Time to stabilze and get the data..
		
		// check for commands from TXRX
		if (MiApp_MessageAvailable()){
			TxRxStatus = TxRx_PeriodTasks();
			if (TxRxStatus != TXRX_NO_ERROR){
				TxRx_printError(TxRxStatus);
				Reset();
				return;
			}
			if (strcmp(g_in_msg, "app wake") == 0) { //YL 11.11 strcmp instead of strcmp_ws
				// TxRx_SetHighPowerMode();	
				m_write("\r\nWaking up..\r\n");
				return;
			}		
		}
	}	
}
/*******************************************************************************
// send_start_block()
// transmit opening block that envelop the transmitted data
*******************************************************************************/
void send_start_block(void) 
{
	TimeAndDate		tad;

	// generate header block with the information about the comming session:
	strcpy(g_accmtr_blk_buff, "HEADER-BLOCK: ");		// title - should not change it, since receiver looks for it
	strcat(g_accmtr_blk_buff, " <> start Sector: ");
	strcat(g_accmtr_blk_buff, long_to_str(g_accmtr_sector_addr_ptr));
	strcat(g_accmtr_blk_buff, " <> num of Blocks: ");
	strcat(g_accmtr_blk_buff, long_to_str(g_num_of_blocks));
	strcat(g_accmtr_blk_buff, " <> Active sensors: ");
	if (g_single_dual_mode == SAMP_BOTH_1282_8451)
		strcat(g_accmtr_blk_buff, "both ADS1282 and MMA8451Q");
	else
		strcat(g_accmtr_blk_buff, "MMA8451Q only");
	strcat(g_accmtr_blk_buff, " <> Start time: ");
	rtc_get_time_date(&tad);
	strcat(g_accmtr_blk_buff, byte_to_str(tad.date.day));
	strcat(g_accmtr_blk_buff, "/");
	strcat(g_accmtr_blk_buff, byte_to_str(tad.date.month));
	strcat(g_accmtr_blk_buff, "/");
	strcat(g_accmtr_blk_buff, byte_to_str(tad.date.year));
	strcat(g_accmtr_blk_buff, " - ");
	strcat(g_accmtr_blk_buff, byte_to_str(tad.time.hour));
	strcat(g_accmtr_blk_buff, ":");
	strcat(g_accmtr_blk_buff, byte_to_str(tad.time.minute));
	strcat(g_accmtr_blk_buff, ":");
	strcat(g_accmtr_blk_buff, byte_to_str(tad.time.second));
	strcat(g_accmtr_blk_buff, " <> CTRL_REG1: ");
	strcat(g_accmtr_blk_buff, uint_to_str((unsigned int)accmtr_reg_read(CTRL_REG1)));
	strcat(g_accmtr_blk_buff, " <> CTRL_REG2: ");
	strcat(g_accmtr_blk_buff, uint_to_str((unsigned int)accmtr_reg_read(CTRL_REG2)));
	strcat(g_accmtr_blk_buff, " <> XYZ_DATA_CFG: ");
	strcat(g_accmtr_blk_buff, uint_to_str((unsigned int)accmtr_reg_read(XYZ_DATA_CFG)));

	// transmit the header block:
	if (g_mode == MODE_SS) {
		flash_write_sector(g_accmtr_sector_addr_ptr, g_accmtr_blk_buff); 	//transmit block from memory buffer
		g_accmtr_sector_addr_ptr++;
	}
	else { 		// MODE_TS, MODE_OST
		if (g_destination == DEST_WIRELESS)
			TxRx_SendData(g_accmtr_blk_buff, MAX_BLOCK_SIZE);
		else  	// DEST_USB
			b_write(g_accmtr_blk_buff, MAX_BLOCK_SIZE);
	}
}

/*******************************************************************************
// sampler_start()
*******************************************************************************/
int sampler_start(void) {	
	accmtr_active();
	if (g_single_dual_mode == SAMP_BOTH_1282_8451) {
		if (ads1282_active() != 0)
			return(-1);
		g_is_ads1282_active = TRUE;
	}

	return(0);
}

/*******************************************************************************
// sampler_stop()
*******************************************************************************/
void sampler_stop(void)	{
	accmtr_standby();
	if (g_single_dual_mode == SAMP_BOTH_1282_8451) {
		ads1282_standby();
		g_is_ads1282_active = FALSE;
	}
}

#endif // #ifdef WISDOM_STONE

//YS 5.1.13 start
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
#if defined COMMUNICATION_PLUG
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

#include "app.h"				//Application

/******************************************************************************
// runPlugCommand()
// if it is a plug command, run it, otherwise don't.
// returns a boolean to indicate if it was a valid plug command.
*******************************************************************************/
BOOL runPlugCommand(){
	if (strcmp(g_curr_msg, "plug reconnect") == 0){//currently the only plug command
			if(MiApp_ResyncConnection(0, 0xFFFFFFFF)){
				m_write ("RECONNECTED: network connection recovery successful!");
				write_eol();
			}else{
				m_write ("NOT-CONNECTED: couldn't recover network connection!");
				write_eol();				
			}
			cmd_ok();
			return TRUE;
	}
	return FALSE;
}

#endif // #ifdef COMMUNICATION_PLUG

//YS 5.1.13 end