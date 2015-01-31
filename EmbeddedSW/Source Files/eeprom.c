/*******************************************************************************

eeprom.c - handle EEPROM read and write
========================================

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
this file contains functions for operating the EEPROM.
- format the memory
- READ / WRITE sequence of bytes
using SW implemented I2C protocol.

********************************************************************************
currently we use 32 KByte EEPROM of ATMEL244/24C 256N;
the code enables using 128 KByte EEPROM of Microchip 24FC1025-I/SM (as mentioned in SW doc).

 - EEPROMS characteristics:
							ATMEL			Microchip
 - memory size:				32 KBytes		128 KBytes
 - page size: 				64 Bytes		128 Byte  
 - number of pages:			512				1024
 - random word address:		15 bits			17 bits

*******************************************************************************/
#include "wistone_main.h"
//#ifdef WISDOM_STONE //YL 23.4 commented and moved to let the plug use the eeprom too 

/***** INCLUDE FILES: *********************************************************/
#include <string.h>			//YL 6.8 to use strcpy
#include "ports.h"
#include "command.h"		//Application
#include "error.h"			//Application
#include "parser.h"			//Application	
#include "system.h"			//Application	//YL for boot global vars
#include "eeprom.h"			//Devices 	 
#include "i2c.h"			//Protocols	
#include "TimeDelay.h"		//TxRx - Common

/***** GLOBAL VARIABLES: ******************************************************/	
char 	g_eeprom_buffer[EEPROM_PAGE_SIZE];	//YL 20.9 temporary buffer //YL 8.12 TODO

/***** INTERNAL PROTOTYPES: ***************************************************/
char* 	get_string(void);		//YL 19.9

/*******************************************************************************
// eeprom_write_byte()
// write a single data Byte into EEPROM at a given address
*******************************************************************************/
int eeprom_write_byte(long addr, BYTE dat)		
{
	char data_to_write[3]; //YL 20.9 was BYTE
	BYTE device_addr = EEPROM_DEVICE_ADDRESS;		//to allow using 128K EEPROM
	
#ifdef EEPROM_128K
	BYTE block_addr = 0, ; 							//to allow using 128K EEPROM
	block_addr = addr & 0x00010000;					//to get 17-th bit 
	block_addr = block_addr << 3;					//pick upper/lower block in 128K EEPROM address space by assigning block_addr bit to 3-rd from right bit in device_addr  //pick upper/lower block in 128K EEPROM address space by assigning block_addr bit to 3-rd from right bit in device_addr  // YL 26.2 
	device_addr = device_addr & block_addr;					
#endif //EEPROM_128K
	if (addr > EEPROM_MEMORY_SIZE || addr <= LAST_BOOT_ADDRESS){	//to avoid accessing illegal addrs
		err(ERR_EEPROM_WRITE_BYTE);
		return -1;
	}

	if (addr == SN_ADDRESS && g_sn_write == FALSE){ //YL 11.11 added g_sn_write
		err(ERR_EEPROM_WRITE_BYTE);
		return -1;
	}				
	data_to_write[0] = (addr >> 8) & 0x000000FF; 	//high address //YL 28.9	
	data_to_write[1] = addr & 0x000000FF; 			//low address
	data_to_write[2] = dat; 						//data byte
	if (device_write_i2c_ert(device_addr, 3, (BYTE*)data_to_write, I2C_WRITE)){ // YL 14.4 added casting to avoid signedness warning										
		err(ERR_EEPROM_WRITE_BYTE);					//2 error cases: EEPROM didn't ack, or failing to write after 5 trials
		return -1;
	}
	DelayMs(20); 									//need at least 20ms delay between writes
	return 0;
}

/*******************************************************************************
// eeprom_read_byte()
// read a single data Byte from EEPROM at a given address
*******************************************************************************/
int eeprom_read_byte(long addr)		
{
	char data_to_write[2]; //YL 20.9 was BYTE
	BYTE device_addr = EEPROM_DEVICE_ADDRESS;
	BYTE read_byte;
	int res = 0;
	
#ifdef EEPROM_128K
	BYTE block_addr = 0; 							//to allow using 128K EEPROM
	block_addr = addr & 0x00010000;					//to get 17-th bit
	block_addr = block_addr << 3;					//pick upper/lower block in 128K EEPROM address space by assigning block_addr bit to 3-rd from right bit in device_addr  
	device_addr = device_addr & block_addr;	
#endif //EEPROM_128K
	if (addr > EEPROM_MEMORY_SIZE)					//to avoid accessing illegal addrs
		return err(ERR_EEPROM_READ_BYTE);
	data_to_write[0] = (addr >> 8) & 0x000000FF;	//high address //YL 28.9	
	data_to_write[1] = addr & 0x000000FF; 			//low address
	// first, write the address we would like to start read from
	if (device_write_i2c_ert(device_addr, 2, (BYTE*)data_to_write, I2C_READ)){ // YL 14.4 added casting to avoid signedness warning
		err(ERR_EEPROM_WRITE_BYTE);					//EEPROM acks accepting the address by 0
		return -1;
	}
	// then, read the data byte
	if (device_read_i2c_ert(device_addr, 1, &read_byte, I2C_READ)){	
		err(ERR_EEPROM_READ_BYTE);					//the only case for error is failing to read after 5 trials	
		return -1;
	}
	res = read_byte & 0x00FF; 
	return res;
}

#ifdef WISDOM_STONE //YL 23.4 moved to let the plug use the eeprom too

/*******************************************************************************
// eeprom_format()
// write entire EEPROM memory with EEPROM_DEFAULT_VALUE,
// besides the last address.
// use PAGE WRITE in order to speed up.
*******************************************************************************/
int eeprom_format(void)
{
	int i = 0, j = 0;
	BYTE device_addr = EEPROM_DEVICE_ADDRESS;
	device_addr = (device_addr << 1) + 0;			//including '0' lsd to specify writing
	int high_addr, low_addr;						//of first address word in each eeprom page
	BYTE dat = EEPROM_DEFAULT_VALUE;				//data to write
	
	// YL 22.8 ... to avoid erasing data like EUI_0
	// was: while (i < NUM_OF_EEPROM_PAGES){
	while (i < NUM_OF_EEPROM_PAGES - 1){
	// ... YL 22.8
		start_i2c_ert(0);
		high_addr = ((i * EEPROM_PAGE_SIZE) >> 8) & 0x00FF; //YL 27.9 was: (i * EEPROM_PAGE_SIZE) & 0xFF00;
		low_addr = (i * EEPROM_PAGE_SIZE) & 0x00FF;
		if (out_byte_i2c_ert(device_addr) || out_byte_i2c_ert(high_addr) || out_byte_i2c_ert(low_addr))
			goto eeprom_format_err;
		j = 0;
		while (j < EEPROM_PAGE_SIZE && (i * EEPROM_PAGE_SIZE + j) != SN_ADDRESS){ // avoid SN_ADDRESS
			if (out_byte_i2c_ert(dat))
				goto eeprom_format_err;
			j++;		//next eeprom byte
		}
		stop_i2c_ert();
		DelayMs(20); 	//needs at least 20ms delay between page writes
		i++;			//next eeprom page 
	}
	return 0;
eeprom_format_err:
	stop_i2c_ert();
	return err(ERR_EEPROM_FORMAT);
}

/*******************************************************************************
// eeprom_write_n_bytes()
// write data string into EEPROM starting with given address
// assumes that the data string is null terminated
*******************************************************************************/
int eeprom_write_n_bytes(long addr, char* dat)	//YL 17.9
{
	char data_to_write[EEPROM_PAGE_SIZE + 2];		//max data_to_write length includes 2 address bytes which precede the "dat" sequence
	BYTE device_addr = EEPROM_DEVICE_ADDRESS;		//to allow using 128K EEPROM
	int len = 0;
	int i = 0;
		
#ifdef EEPROM_128K 
	BYTE block_addr = 0, ; 							//to allow using 128K EEPROM
	block_addr = addr & 0x00010000;					//to get 17-th bit 
	block_addr = block_addr << 3;					//pick upper/lower block in 128K EEPROM address space by assigning block_addr bit to 3-rd from right bit in device_addr  //pick upper/lower block in 128K EEPROM address space by assigning block_addr bit to 3-rd from right bit in device_addr  // YL 26.2 
	device_addr = device_addr & block_addr;					
#endif //EEPROM_128K
	data_to_write[0] = (addr >> 8) & 0x000000FF; 	//high address //YL 28.9
	data_to_write[1] = addr & 0x000000FF; 			//low address
	for (i = 2; i < (EEPROM_PAGE_SIZE + 2); i++)
		data_to_write[i] = '\0';					//after 2 first address bytes - data_to_write contains initially an empty 64-bytes sequence (to make sure the entry is completely erased)
	if (dat[0] != '\0') 
		strcpy(&data_to_write[2], dat);			//if dat string isn't empty then it is appended after 2 first address bytes (if dat string is empty, then it is meant to erase 64-bytes boot table entry) 				
	len = EEPROM_PAGE_SIZE + 2;						//len includes 2 first address bytes, and is the same - whether the string is empty or not	
	if (device_write_i2c_ert(device_addr, len, (BYTE*)data_to_write, I2C_WRITE))	// YL 14.4 added casting to avoid signedness warning						
		return err(ERR_EEPROM_WRITE_N_BYTES);			
	DelayMs(20); 									//need at least 20ms delay between writes
	
	return 0;
}

/*******************************************************************************
// eeprom_read_n_bytes()
// read n bytes of data string from given EEPROM address 
// and display it to the terminal -  
// if the requested length is longer than (EEPROM_PAGE_SIZE - 1) (for '\0')
// then every time only (EEPROM_PAGE_SIZE - 1) characters are displayed
// and only the last (EEPROM_PAGE_SIZE - 1) characters are returned
*******************************************************************************/
int eeprom_read_n_bytes(long addr, char* data_to_read, int len)	 //YL 17.9 no length restriction on sequential reading (like on page write)
{
	char data_to_write[2];
	BYTE device_addr = EEPROM_DEVICE_ADDRESS;
	int i = 0, j = 0, k = 0;
	
	for (i = 0; i < EEPROM_PAGE_SIZE; i++)			//clear data_to_read which is g_curr_boot_cmd buffer (if the command is "boot get"), or the eeprom_buffer (if the command is "eeprom read"); this needs to be done because in case of "eeprom read" any non blank sequence in the data_to_read buffer is displayed, and it might be some left overs (unless cleared)
		data_to_read[i] = '\0';	
	
#ifdef EEPROM_128K  
	BYTE block_addr = 0; 							//to allow using 128K EEPROM
	block_addr = addr & 0x00010000;					//to get 17-th bit
	block_addr = block_addr << 3;					//pick upper/lower block in 128K EEPROM address space by assigning block_addr bit to 3-rd from right bit in device_addr  
	device_addr = device_addr & block_addr;	
#endif //EEPROM_128K
	data_to_write[0] = (addr >> 8) & 0x000000FF; 	//high address
	data_to_write[1] = addr & 0x000000FF; 			//low address	
	i = len / EEPROM_PAGE_SIZE;						//every time only EEPROM_PAGE_SIZE characters are displayed (or less)
	j = len % EEPROM_PAGE_SIZE;
	while (i) {	 									//display all blocks of EEPROM_PAGE_SIZE (64 bytes)
		// first, write the address we would like to start read from 
		if (device_write_i2c_ert(device_addr, 2, (BYTE*)data_to_write, I2C_READ))	// YL 14.4 added casting to avoid signedness warning //there is no restriction on length, but the buffer that is used for reading is statically allocated, so before we read a new block - 2 addresses (device_addr and data_to_write) need to be sent first
			return err(ERR_EEPROM_READ_N_BYTES);						
		// then, read the data string
		if (device_read_i2c_ert(device_addr, EEPROM_PAGE_SIZE, (BYTE*)data_to_read, I2C_READ)) 	// YL 14.4 added casting to avoid signedness warning
			return err(ERR_EEPROM_READ_N_BYTES);
		k = 0;
		while (k < EEPROM_PAGE_SIZE) {						//display all non-blank character sequences in current eeprom page, use "; " to separate the sequences //YL 10.10
			while (data_to_read[k] == '\0' && k < EEPROM_PAGE_SIZE) 	//display non-blank character sequence
				k++;
			// YL 29.12 ... added data_to_print for more effective wireless transmissions
			char data_to_print[EEPROM_PAGE_SIZE + 1];
			if (k < EEPROM_PAGE_SIZE) {
				// was:
				//m_write(&data_to_read[k]); 				
				//k += strlen(&data_to_read[k]); 	
				//m_write("; ");
				strcpy(data_to_print, &data_to_read[k]);
				strcat(data_to_print, "; ");
				m_write(data_to_print);
				k += strlen(&data_to_read[k]);
				// ... YL 29.12
			}
		}
		addr += EEPROM_PAGE_SIZE;							//the new address to continue reading from after reading a block of EEPROM_PAGE_SIZE (64 bytes)
		data_to_write[0] = (addr >> 8) & 0x000000FF; 		//high address
		data_to_write[1] = addr & 0x000000FF; 				//low address
#ifdef EEPROM_128K  
		block_addr = addr & 0x00010000;						//to get 17-th bit
		block_addr = block_addr << 3;						//pick upper/lower block in 128K EEPROM address space by assigning block_addr bit to 3-rd from right bit in device_addr  
		device_addr = device_addr & block_addr;	
#endif //EEPROM_128K
		i--;
	} 
	if (j) {												//display the remaining bytes (less than EEPROM_PAGE_SIZE)
		// first, write the address we would like to start read from
		if (device_write_i2c_ert(device_addr, 2, (BYTE*)data_to_write, I2C_READ)) // YL 14.4 added casting to avoid signedness warning
			return err(ERR_EEPROM_READ_N_BYTES);				
		// then, read the data string
		if (device_read_i2c_ert(device_addr, j, (BYTE*)data_to_read, I2C_READ))	 // YL 14.4 added casting to avoid signedness warning
			return err(ERR_EEPROM_READ_N_BYTES);		
		data_to_read[j] = '\0';								//append '\0' to displayed string 
		k = 0;
		while (k < j) {										//display all non-blank character sequences in current eeprom page, use "; " to separate the sequences //YL 10.10	
			while (data_to_read[k] == '\0' && k < j)		//display non-blank character sequence
				k++;
			// YL 29.12 ... added data_to_print for more effective wireless transmissions
			char data_to_print[EEPROM_PAGE_SIZE + 1];				
			if (k < j) {
				// was:
				//m_write(&data_to_read[k]);				
				//k += strlen(&data_to_read[k]); 
				//m_write("; ");
				strcpy(data_to_print, &data_to_read[k]);
				strcat(data_to_print, "; ");
				m_write(data_to_print);
				k += strlen(&data_to_read[k]);
				// ... YL 29.12				
			}
		}
	}
	write_eol();
	return 0;
}

/*******************************************************************************
// eeprom_boot_set()
// write boot cmd into specified cmd table entry (which needs to be converted to 
// address within the EEPROM)
// if no data string was sent - eeprom_boot_set erases specified entry
*******************************************************************************/
int eeprom_boot_set(BYTE entry, char* dat)	 //YL 17.9
{
	long address = 0;
		
	address = BASE_BOOT_ADDRESS + entry * EEPROM_PAGE_SIZE;		//convert boot table entry number (0,... 9) to address within the EEPROM	
	if (eeprom_write_n_bytes(address, dat))						//write dat string that includes terminal '\0' (maybe followed by invalid characters)
		return err(ERR_EEPROM_BOOT_SET);
	return 0;
}

/*******************************************************************************
// eeprom_boot_get()
// read boot cmd from specified cmd table entry (which needs to be converted to 
// address within the EEPROM) into global g_curr_boot_cmd string 
// if boot cmd's length is 0 - the entry is empty
*******************************************************************************/
int eeprom_boot_get(BYTE entry)	//YL 17.9
{
	long address = 0;
	
	address = BASE_BOOT_ADDRESS + entry * EEPROM_PAGE_SIZE;					//convert boot table entry number (0,... 9) to address within the EEPROM
	if (eeprom_read_n_bytes(address, g_curr_boot_cmd, MAX_BOOT_CMD_LEN))	//read the command + '\0' + invalid following characters up to MAX_BOOT_CMD_LEN bytes
		return err(ERR_EEPROM_BOOT_GET);
	return 0;
}


/*******************************************************************************
// get_string()
// get the string that needs to be written as a part of "eeprom sboot/write" commands
// and adds terminal '\0' to it
*******************************************************************************/
char* get_string(void)		//YL 19.9
{
	int i = 0, j = 0;
	
	g_eeprom_buffer[0] = '\0';									//initialize eeprom buffer as empty string
	if (parse_long_num(g_tokens[2]) == -1) 						//third parameter is not a number
		return g_eeprom_buffer;									//return an empty string
	
	i = 0;
	while ((g_in_msg[i] == ' ') || (g_in_msg[i] == '\t')) 		//skip the spaces until the first string which is "eeprom"
		i++;
	i += 6;							//"eeprom"	
	
	while ((g_in_msg[i] == ' ') || (g_in_msg[i] == '\t'))		//skip the spaces until the second string which is "sboot" or "write"
		i++;
	i += 5; 						//"sboot" or "write"
		
	while ((g_in_msg[i] == ' ') || (g_in_msg[i] == '\t')) 		//skip the spaces until the third string which is a number
		i++;
	i += strlen(g_tokens[2]); 	//number
	
	if (*(g_in_msg + i)) {		 	//just in case the remaining string is not empty
		while ((g_in_msg[i] == ' ') || (g_in_msg[i] == '\t')) 	//skip the spaces until the first non-white character of the remaining g_in_msg 
			i++;
		j = strlen(g_in_msg + i);
		if (j > EEPROM_PAGE_SIZE - 3) {							//in case the remaining g_in_msg is longer than 61 bytes
			m_write("note: only first 61 bytes are written");	//warning 
			write_eol(); 
			g_in_msg[i + (EEPROM_PAGE_SIZE - 3)] = '\0';		//take only the first 61 bytes of it, append '\0' as #62 bit (2 additional bytes are for address within the EEPROM that would be added later)
		}
		else {
			g_in_msg[i + j] = '\0';								//append '\0'					
		}
		strcpy(g_eeprom_buffer, &g_in_msg[i]);		
	}
	
	return g_eeprom_buffer;
}

/*******************************************************************************
// handle_eeprom()
// if first token was "eeprom", then handle EEPROM commands message:
// - according to sub command, call the relevant function
// - if command failed, display error message, and return error code
// - if command executed OK, display OK message and return 0.
*******************************************************************************/
int handle_eeprom(int sub_cmd)
{
	int res = 0;
	long address = 0;
	unsigned int len = 0;
	char* dat;
	
	if (sub_cmd != SUB_CMD_FORMAT && g_ntokens < 3) 			//at least 3 parameters are expected by any EEPROM command besides FORMAT
		return err(ERR_INVALID_PARAM_COUNT);
		
	if (sub_cmd != SUB_CMD_FORMAT) {							//the first parameter of every command besides FORMAT is an address within the EEPROM
		address = parse_long_num(g_tokens[2]);
		if ((address < 0) || (address >= SN_ADDRESS)) 			//avoid serial number override			
			return err(ERR_INVALID_PARAM);
	}
	
	// dispatch to the relevant sub command function according to sub command
	switch (sub_cmd) {
		case SUB_CMD_FORMAT:
			res = eeprom_format();
		break;
		
		case SUB_CMD_SBOOT:										
			if (address < MIN_BOOT_ENTRY_NUM || 				//boot table entry number (0,... 9)
				address > MAX_BOOT_ENTRY_NUM) 		
				return err(ERR_INVALID_PARAM);
			dat = get_string();									//empty string is legal too - when an entry needs to be erased			
			res = eeprom_boot_set((0xFF & address), dat);
		break;
		
		case SUB_CMD_GBOOT:			
			if (address < MIN_BOOT_ENTRY_NUM || 				
				address > MAX_BOOT_ENTRY_NUM) 					//boot table entry number (0,... 9)
				return err(ERR_INVALID_PARAM);
			res = eeprom_boot_get(0xFF & address);	
		break;
				
		case SUB_CMD_WRITE:										
			if ((address <= LAST_BOOT_ADDRESS) || 				//avoid boot table override
			   (address + EEPROM_PAGE_SIZE >= SN_ADDRESS) || 	//avoid serial number override	
			   (address % EEPROM_PAGE_SIZE != 0))				//the address should be a multiple of 64 //YL 11.10
			   return err(ERR_INVALID_PARAM);
			dat = get_string();	
			if (dat[0] != '\0')												
				res = eeprom_write_n_bytes(address, dat);		//proceed only if the string isn't empty		
		break;
		
		case SUB_CMD_READ:
			if (g_ntokens != 4) 			
				return err(ERR_INVALID_PARAM_COUNT);
			len = parse_int_num(g_tokens[3]);
			if ((len <= 0) ||									
			   (address % EEPROM_PAGE_SIZE != 0))				//the address should be a multiple of 64 //YL 11.10
				return err(ERR_INVALID_PARAM);
			res = eeprom_read_n_bytes(address, g_eeprom_buffer, len);	
		break;
				
		default:
			return cmd_error(ERR_UNKNOWN_SUB_CMD);
		break;
	}
	if (res < 0)
		return cmd_error(0);

	cmd_ok();
	return 0;
}

#endif //WISDOM_STONE

