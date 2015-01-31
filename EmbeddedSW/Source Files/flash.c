/*******************************************************************************

flash.c - handle micro-SD card read and write
=============================================

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
this file contains functions for operating the FLASH card.
- format the memory
- READ / WRITE sequence of bytes 
- get Card Detect
- get card capacity 	
using HW implemented SPI1 module in PIC.
*******************************************************************************/
#include "wistone_main.h"
#ifdef WISDOM_STONE

/***** INCLUDE FILES: *********************************************************/
#include "ports.h"	
#include "command.h"		//Application		
#include "error.h"			//Application
#include "parser.h"			//Application
#include "misc_c.h"			//Common	//YL 11.11 to remove disp_num_to_term
#include "flash.h"			//Devices	
#include "SD-SPI.h"			//Protocols

/***** INTERNAL PROTOTYPES: ***************************************************/
int 	flash_media_init(void);	
int 	flash_get_card_detect(void);
DWORD 	flash_get_capacity(void);

/*******************************************************************************
// init_flash()
// - initialize the FLASH card IO
// - initialize the media
*******************************************************************************/
int init_flash(void)
{
	MDD_SDSPI_InitIO();
	return (flash_media_init());
}

/*******************************************************************************
// flash_media_init()
// initialize the FLASH card, and gets back media information
*******************************************************************************/	
int flash_media_init(void)
{
	MEDIA_INFORMATION *mi;

	mi = MDD_SDSPI_MediaInitialize();
	if (mi->errorCode != ERR_NONE)
		return err(ERR_SDSPI_INIT);
	return 0;
}

/*******************************************************************************
// flash_get_card_detect()
// get the value of Card Detect	
*******************************************************************************/
int flash_get_card_detect(void)
{
	int	res = 0;
	
	res = MDD_SDSPI_MediaDetect();
	if (res == 0) 	
		return err(ERR_SDSPI_CD);		//the card wasn't detected
	return 0;
}

/*******************************************************************************
// flash_get_capacity()
// displays to terminal the value of current FLASH card capacity (in sectors)
*******************************************************************************/
DWORD flash_get_capacity(void)
{
	DWORD res = 0;
	
	if (flash_get_card_detect() != 0)
		return err(ERR_SDSPI_CD);
	res = MDD_SDSPI_ReadCapacity(); 
	m_write(long_to_str(res)); //YL 11.11 instead of disp_num_to_term
	write_eol();
	return 0;
}

/*******************************************************************************
// flash_write_sector()
// writes a string to start of FLASH sector, without changing the rest.
*******************************************************************************/
int flash_write_sector(DWORD sector_addr, BYTE* dat)
{
	int res = 0;
	
	if (sector_addr > MAX_FLASH_SECTOR_ADDR)
		return err(ERR_INVALID_PARAM);
	res = MDD_SDSPI_SectorWrite(sector_addr, dat, 1); //3-rd param = 1 to allow write to zero sector (MBR) too
	if (res == FALSE)
		return err(ERR_SDSPI_WRITE);
	return 0;
}

/*******************************************************************************
// flash_read_sector()
// reads whole sector into dat buffer
*******************************************************************************/
int flash_read_sector(DWORD sector_addr, BYTE* dat)
{
	int res = 0;
	
	if (sector_addr > MAX_FLASH_SECTOR_ADDR)
		return err(ERR_INVALID_PARAM);
	res = MDD_SDSPI_SectorRead(sector_addr, dat);
	if (res == FALSE)
		return err(ERR_SDSPI_READ);
	return 0;
}

/*******************************************************************************
// flash_write_byte()
// writes byte into specified address in flash
*******************************************************************************/
int flash_write_byte(long addr, BYTE dat)
{
	
	BYTE 	buff[FLASH_SECTOR_SZ];		
	int 	res;
	DWORD 	sector_addr;			
	int 	in_sector_addr;

	res = 0;
	sector_addr = (addr >> 9) & 0x003FFFFF;		//to get sector address - divide by 512 (sector size); max 22 bits are valid
	in_sector_addr = addr & 0x000001FF; 		//to get the address within specified sector - by modulo 512 
	res = flash_read_sector(sector_addr, buff);
	if (res == -1)
		return (-1);
	buff[in_sector_addr] = dat;
	res = flash_write_sector(sector_addr, buff);
	if (res == -1)
		return -1;
	return 0;
}

/*******************************************************************************
// flash_read_byte()
// reads single byte from specified address in flash
*******************************************************************************/
int flash_read_byte(long addr)
{
	BYTE 	buff[FLASH_SECTOR_SZ];	
	int 	res;
	DWORD 	sector_addr;		
	int 	in_sector_addr;

	res = 0;
	sector_addr = (addr >> 9) & 0x003FFFFF;		//to get sector address - divide by 512 (sector size); max 22 bits are valid
	in_sector_addr = addr & 0x000001FF; 		//to get the address within specified sector - by modulo 512 
	
	res = flash_read_sector(sector_addr, buff);
	if (res == -1)
		res = -1;
	else
		res = 0x00FF & buff[in_sector_addr];
	return res;
}

/*******************************************************************************
// flash_format()
// reset the FLASH memory with zeros
*******************************************************************************/
int flash_format(void) 
{	
	DWORD i;
	DWORD sector_addr = 1;	//AY - start formatting at zero sector didn't work 
	BYTE zero_buff[FLASH_SECTOR_SZ] = {0};	//maybe this doesn't fill whole array with 0
	int res;	
		
	m_write("note: flash format might take over an hour to complete"); 
	write_eol();
	for (i = 1; i < MAX_FLASH_SECTOR_ADDR; i++) {	//AY - start formatting at zero sector didn't work 
		res = flash_write_sector(sector_addr++, zero_buff);
		// YL 22.8 ... to speed format up
		// was: m_write(".");
		if (i%100 == 0) {
			m_write(".");
		}
		// ... YL 22.8
		if (res != 0)
			return err(ERR_EEPROM_FORMAT);
	}
	return 0;
}

/*******************************************************************************
// handle_flash()
// if first token was "flash", then handle FLASH commands message:
// - according to sub command, call the relevant function
// - if command failed, display error message, and return error code
// - if command executed OK, display OK message and return 0.
*******************************************************************************/
int handle_flash(int sub_cmd)	
{
	int res = 0;
	// YL 22.8 ... clear buff
	// was: BYTE buff[FLASH_SECTOR_SZ];
	BYTE buff[FLASH_SECTOR_SZ] = {0};
	// ... YL 22.8
	DWORD sector_addr = 0;

	// dispatch to the relevant sub command function according to sub command
	switch (sub_cmd) {
		case SUB_CMD_GCD:
			res = flash_get_card_detect();
			break;

		case SUB_CMD_MINIT:
			res = flash_media_init();
			break;
		
		case SUB_CMD_GCAP:		
			res = flash_get_capacity();
			break;
		
		case SUB_CMD_WSECTOR:
			sector_addr = parse_long_num(g_tokens[2]); 
			if (sector_addr < 0)
				return -1;
			res = flash_write_sector(sector_addr, (BYTE*)g_tokens[3]); 
			break;
			
		case SUB_CMD_RSECTOR:
			sector_addr = parse_long_num(g_tokens[2]); 
			if (sector_addr < 0)
				return -1;		
			res = flash_read_sector(sector_addr, buff); 
			m_write((char*)buff);	// YL 14.4 added casting to avoid signedness warning
			write_eol();	
			break;
	
		case SUB_CMD_FORMAT:
			res = flash_format();
			break;
			
		default:
			err(ERR_UNKNOWN_SUB_CMD);
			cmd_error(0);
			break;
	}
	if (res == 0)
		cmd_ok();
	return 0;
}
#endif // #ifdef WISDOM_STONE
