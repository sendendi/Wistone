/*******************************************************************************

command.c
=========

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
this is the main auxiliary file that contains various utilities, like - delay,
handle read and write, write msgs to terminal, etc.
*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include <string.h>						//YL 6.8 to use strlen\cmp\cpy
#include "command.h"					//Application
#include "error.h"						//Application
#include "parser.h"						//Application
#include "system.h"						//Application
#include "wistone_main.h"				//Application
#include "eeprom.h"						//Devices	//YL 18.9 for boot cmds	
#include "led_buzzer.h"					//Devices
#include "rtc.h"						//Devices	//YL 18.9 for RTC wakeup source	
#ifdef USBCOM							
#include "wistone_usb.h"				//USB_UART
#include "usb.h"						//USB_UART
#elif defined RS232COM
#include "rs232.h"						//USB_UART
#endif // #ifdef USBCOM
#include "TxRx.h"						//TxRx - Application
#include "SymbolTime.h"					//TxRx - Application
#include "HardwareProfileTxRx.h"		//TxRx - Common
#include "TimeDelay.h"					//TxRx - Common
//#include "P2P.h"						//TxRx - Protocols

/***** GLOBAL VARIABLES: ******************************************************/
char		*g_tokens[MAX_TOKENS];				// array holding pointers to the tokens (each ended by '\0') 
int			g_ntokens; 							// number of tokens in current message
char		g_in_msg[MAX_CMD_LEN]; 				// global string holding the received message, copied from g_curr_msg
CommTypes 	g_usb_or_wireless_print;			// indicate where to send the response to. updated according to where we received the message from
BYTE 		g_character_array[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'}; // used for USB send HEX numbers //YL 8.12
BYTE 		g_is_cmd_received;
BOOL		g_boot_seq_pause = FALSE;			// indicates wait before read next boot command

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
#if defined WISDOM_STONE
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/*******************************************************************************
// exec_message_command()
// - USB:
//   - perform periodic tasks
//   - if message received, handle it
// - RXTX:
//   - perform periodic tasks
//   - if message received, handle it
// - BOOT:
//   - if woke up due to RTC event - take next command from Config table (Boot_command).
*******************************************************************************/
int exec_message_command(void)
{
	static BYTE boot_cmd_addr = 0;
		
#ifdef USBCOM 					
	USB_STATUS status = USB_ReceiveDataFromHost(); 	// periodic USB tasks and check for incoming data
	if (status == USB_RECEIVED_DATA) {				// indicates there is a message in the USB
		g_usb_or_wireless_print = COMM_USB;
		// After getting the message, handle the message
		strcpy(g_in_msg, g_curr_msg);				
		handle_msg(g_in_msg);
	}
#endif  // #ifdef USBCOM 
	// Command could be received through the Wireless, while we were waiting for ack 
	// (for example, we got "app stop" while sending blocks and expecting acks), therefore
	// "g_is_cmd_received" flag indicates that during this period a message was received.
	if (g_is_cmd_received == 1) {
		handle_msg(g_in_msg);
		g_is_cmd_received = 0;
	}	
	if (MiApp_MessageAvailable()) {					// check for commands from TXRX
		g_usb_or_wireless_print = COMM_WIRELESS;
		TXRX_ERRORS status = TxRx_PeriodTasks();	// TXRX periodic tasks and check for incoming data
		// After getting the message, handle the message
		if (status != TXRX_NO_ERROR) {
			TxRx_PrintError(status);
			// YL 16.8 ...
			// Reset(); meanwhile reconnect doesn't work
			// ... YL 16.8
		}
		handle_msg(g_in_msg);
	}
	if (g_rtc_wakeup == TRUE && g_boot_seq_pause == FALSE) {	//YL 18.9
		if (eeprom_boot_get(boot_cmd_addr)) {
			g_sleep_request = 1;					// since EEPROM is faulty, we have nothing to do but go to sleep...
			return -1;
		}
		if (g_curr_boot_cmd[0] == '\0') { 			// we have reached first non valid boot table entry
			g_sleep_request = 1;
			return 0;
		}
		handle_msg(g_curr_boot_cmd);
		boot_cmd_addr++;
		if (boot_cmd_addr >= MAX_BOOT_ENTRY_NUM) {
			g_sleep_request = 1;					// we have reached the end of boot table (after reading all table entries)
			return 0;
		}
	}
	return 0;
} 

#endif //WISDOM_STONE

/*******************************************************************************
// cmd_ok()
// send back OK message 
*******************************************************************************/
void cmd_ok(void)
{
	m_write("ok");
	write_eol();
}

/*******************************************************************************
// cmd_error()
// display back an error message, according to errid.
// also, play sound on buzzer.
*******************************************************************************/
int cmd_error(int errid)
{
	if (errid > 0)
		err(errid);
	else
		err(ERR_UNKNOWN);
	//play_buzzer(ERROR_BUZZ_TIME); //YL 20.8 annoying....
	return -1;
}

/*******************************************************************************
// write_eol()
// send prompt
*******************************************************************************/
void write_eol(void)
{
	m_write("\r\nWISTONE> ");
}

/*******************************************************************************
// b_write()
// given a block of data with "len" chars
// write it, char by char, to the output channel (USB/UART). 
*******************************************************************************/
void b_write(BYTE* block_buffer, int len) 
{		
#ifdef RS232COM
	int i = 0;
	for (; i < len; i++)
		rs232_putc(block_buffer[i]);
#elif defined USBCOM
	USB_WriteData(block_buffer, len); // YS ~10.10 no need for 4 byte conversion
#endif // RS232COM
}

/*******************************************************************************
// m_write()
// given a ready to send message, write it, char by char, to the output channel:
// - wired USB/UART print
// - wireless MRF print 
*******************************************************************************/
void m_write(char *str)	// AY 7.8
{
	WORD strLength = strlen(str); 

#if defined WISDOM_STONE	
	if ((g_usb_or_wireless_print == COMM_USB) && (g_usb_connected == TRUE)) { // YS 17.8
	#if defined RS232COM
		char *p = str;
		while ((*p) != '\0') {
			rs232_putc(*p);
			p++;
		}
	#elif defined USBCOM
		USB_WriteData((BYTE*)str, strLength); // YL 14.4 added casting to avoid signedness warning
	#endif // RS232COM, USBCOM
	}	
	else if (g_usb_or_wireless_print == COMM_WIRELESS) {
		DelayMs(20);	
		m_TxRx_write((BYTE*)str);  	// YL 14.4 added casting to avoid signedness warning
		DelayMs(20);
	}

#elif defined COMMUNICATION_PLUG // AY if it is communication_plug, we write only to the usb		
	#ifdef RS232COM
	char *p = str;
	while ((*p) != '\0') {
		rs232_putc(*p);
		p++;
	}
	#elif defined USBCOM
	USB_WriteData((BYTE*)str, strLength); // YL 14.4 added casting to avoid signedness warning
	#endif // RS232COM, USBCOM
#endif // WISDOM_STONE
}

#if defined DEBUG_PRINT	// YL 12.1
	
	#define DEBUG_BUFFER_LEN 400

	char 	debug_buffer[DEBUG_BUFFER_LEN];
	int		write_index = 0;
	int 	read_index = 0;

/******************************************************************************
// void m_write_debug(char *str) 
// save g_usb_or_wireless_print and assign USB to it, use m_write to print the str 
******************************************************************************/	
void m_write_debug(char *str) 	
{	
	int len = strlen(str);
	int i = 0;
	
	for ( ; i < len; i++) {
		debug_buffer[(write_index++) % DEBUG_BUFFER_LEN] = str[i];
	}
}

void put_char_debug(void)
{	
	char char_to_print[2] = "_";
	static int i = 0;
	
	if (i == 10) {
		if (read_index != write_index) {	
			CommTypes dest_backup = g_usb_or_wireless_print;
			g_usb_or_wireless_print = COMM_USB;	
			char_to_print[0] = debug_buffer[(read_index++) % DEBUG_BUFFER_LEN];
			m_write(char_to_print);
			g_usb_or_wireless_print = dest_backup;
		}
		i = 0;
	}
	i++;
}

#endif // DEBUG_PRINT

/*******************************************************************************
// blink_led() - for debug
*******************************************************************************/
void blink_led() 
{
	set_led(1, LED_2); //blink LED#2
	DelayMs(40); 
	set_led(0, LED_2);
}

/*******************************************************************************
// blink_buzz() - for debug
*******************************************************************************/
void blink_buzz(void)  //YL 7.5
{
	#if defined (WISDOM_STONE)
		play_buzzer(1);	
	#elif defined (COMMUNICATION_PLUG)
		blink_led();
	#endif
}

/*****************************************************************************/
// PrintChar()
/*****************************************************************************/
//YL 13.5 all following - AY

// ABYS: Functions we need to use.
 void PrintChar(BYTE toPrint){
	#if defined (USBCOM)
		if (g_usb_connected == TRUE){ //YS 17.8
		    BYTE PRINT_VAR;
		    PRINT_VAR = toPrint;
		    toPrint = (toPrint >> 4) & 0x0F;
		    USB_WriteData(&g_character_array[toPrint], 1);
		    toPrint = (PRINT_VAR) & 0x0F;			
		    USB_WriteData(&g_character_array[toPrint], 1);
		}
	#endif
    return;
}

/*****************************************************************************/
// PrintDec()
/*****************************************************************************/
 void PrintDec(BYTE toPrint){
	#if defined (USBCOM)
		if (g_usb_connected == TRUE){ //YS 17.8
	    	USB_WriteData(&g_character_array[toPrint/10], 1);
	    	USB_WriteData(&g_character_array[toPrint%10], 1);
		}
	#endif
}

/*****************************************************************************/
// ConsolePut
/*****************************************************************************/
 void ConsolePut(BYTE c){
	#if defined (USBCOM)
		if (g_usb_connected == TRUE){ //YS 17.8
			USB_WriteData(&c , 1);
		}
	#endif
}
