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
#include "error.h"						//Application
#include "parser.h"						//Application
#include "system.h"						//Application
#include "wistone_main.h"				//Application
#include "misc_c.h"						//Common
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
char		g_in_msg[MAX_CMD_LEN]; 				// global string holding the recieved message, copied from g_curr_msg
DestTypes 	g_usb_or_wireless_print;			// indicate where to send the response to. updated according to where we received the message from
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
	USB_STATUS status = USB_ReceiveData(); 			// periodic USB tasks and check for incoming data
	if (status == USB_RECEIVED_DATA) {				// indicates there is message in the USB
		g_usb_or_wireless_print = DEST_USB;
		// After getting the message, handle the message
		strcpy(g_in_msg, g_curr_msg);				// YL 11.11 strcpy instead of strcpy_ws
		handle_msg(g_in_msg);
	}
#endif  // #ifdef USBCOM 
	// Command could be received through the Wireless, while we were waiting for ack (For
	// Example, we got "app stop" while sending blocks and expecting acks, therefore,
	// the "g_is_cmd_received flag indicates that during this period, a message was received.
	if (g_is_cmd_received == 1){
		handle_msg(g_in_msg);
		g_is_cmd_received = 0;
	}
	if (MiApp_MessageAvailable()){					// check for commands from TXRX
		g_usb_or_wireless_print = DEST_WIRELESS;
		TXRX_ERRORS status = TxRx_PeriodTasks();	// TXRX periodic tasks and check for incoming data
		// After getting the message, handle the message
		if (status != TXRX_NO_ERROR){
			TxRx_printError(status);
			Reset();
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

#endif // #if defined WISDOM_STONE

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
char num2char_2(unsigned int num)	//AY 7.8
{
	switch(num){
		case 0:	return '0';
		case 1:	return '1';
		case 2:	return '2';
		case 3: return '3';
		case 4:	return '4';
		case 5: return '5';
		case 6: return '6';
		case 7:	return '7';	
		case 8:	return '8';
		case 9: return '9';	
	}
	return NULL;
}

void our_num2str_2(char *str, char num)	//AY 7.8
{	
	unsigned char temp_num = (unsigned char)num;
	unsigned int temp = 0;
	int i;
	for (i = 0; i <= 2;  i++){
		temp = temp_num % 10;
		str[2 - i] = num2char_2(temp) ;
		temp_num = temp_num / 10;
	} 
}

void b_write(BYTE * block_buffer, int len) {	

#ifdef RS232COM
	int i = 0;
	for (; i < len; i++)
		rs232_putc(block_buffer[i]);
#elif defined USBCOM
	USB_WriteData(block_buffer,len); //YS ~10.10 no need for 4 byte conversion	
#endif // #ifdef RS232COM
}

/*******************************************************************************
// m_write()
// given a ready to send message, write it, char by char, to the output channel:
// - wired USB/UART print
// - wireless MRF print 
*******************************************************************************/
void m_write(char *str)	//AY 7.8
{
	WORD strLength = strlen_ws(str); //YL 11.11 strlen instead of strlen_ws

#if defined WISDOM_STONE	
	if ((g_usb_or_wireless_print == DEST_USB) && (g_usb_connected==TRUE)){ //YS 17.8
	#if defined RS232COM
		char *p = str;
		while ((*p) != '\0') {
			rs232_putc(*p);
			p++;
		}
	#elif defined USBCOM
		USB_WriteData(str, strLength);
	#endif // #ifdef RS232COM
	}	
	else if (g_usb_or_wireless_print == DEST_WIRELESS){
		DelayMs(20);
		m_TxRx_write(str);
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
	USB_WriteData(str,strLength);
	#endif // #ifdef RS232COM
#endif // #if defined WISDOM_STONE
}

/*******************************************************************************
// blink_led() - for debug
*******************************************************************************/
void blink_led() {
	set_led(1, LED_2); //blink LED#2
	DelayMs(2000); 
	set_led(0, LED_2);
	DelayMs(2000);
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
		    toPrint = (toPrint>>4)&0x0F;
		    USB_WriteData(&g_character_array[toPrint], 1);
		    toPrint = (PRINT_VAR)&0x0F;
			
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
