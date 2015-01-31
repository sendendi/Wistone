/*******************************************************************************

Wisdom Stone - under road sensing system

********************************************************************************

	Authors:		Boaz M, Yulia L
					Technion
					
	Date:			2011, Sept 16
	Version:		1.0
	Compiled using:	MPLAB C30
	Target device:	PIC24FJ256GB110
	
********************************************************************************
	
	Revision History:
	=================
 ver 1.00.0 date: 1.10.11
		- Initial revision
 ver 1.01.0 date: 29.10.11
		- Joint work
 ver 1.02.0 date: 8.8.12
		- Overall joint code review
 ver 1.03.0 date: 21.9.12
		- BOOT table
 ver 1.05.0 date: 10.2.13 - Wistone stage 1: P2P
		- Final version of stage 1
		- P2P implementation by Yossi and Amnon
		- Debugged and working
 ver 1.06.0 date: 23.9.13 - P2P replaced with MiWi
		- Initial MiWi version
		- MiWi implementation by Yulia
		- Debugged and working			
		
********************************************************************************
	General:
	========
this is the main file of the top level application for Wisdom Stone System.
this project can be compiled with two main compiler flags:
- WISDOM_STONE
- COMMUNICATION_PLUG

other complilation flags:
- USBCOM - use USB as the wired communication channel
- RS232  - use the UART as the wired communication channel

WISDOM_STONE //YL editted the description
============
the main process of the stone waits for a command to be recieved from: 
- USB / UART, or 
- RF TXRX (depends on the #define used)
then the main calls the relevant function to handle the command.

COMMUNICATION_PLUG //YL editted the description
==================
the main process of the plug transmits:
- commands and acknowledgements from the USB to the stone
- data and acknowledgements from the stone to the USB
(hence the plug is basically USB <-> RF converter)
*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include "wistone_main.h"					// Application
#include "app.h"							// Application
#include "command.h"						// Application
#include "system.h"							// Application	
#include "parser.h"							// Application
#include "HardwareProfileRemappable.h"		// Common
#include "misc_c.h"							// Common
#include "p24FJ256GB110.h"					// Common
#include "lcd.h"							// Devices
#include "usb.h"							// USB
#include "wistone_usb.h"					// USB
#include "TxRx.h"							// TxRx - Application
#include "Compiler.h"
#include "MCHP_API.h"						// TxRx - Application

/***** GLOBAL CONFIGURATIONS: *************************************************/
// Note that main clk is 20MHz (Fcy = 10MHz) //YL 32MHz, 16MHz
// USB 96 MHz PLL Prescaler Select bits: bit 14-12: 100 = Oscillator input divided by 5 (20 MHz input)
_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2 & WDTPS_PS1024 & FWPSA_PR32) 
_CONFIG2( PLL_96MHZ_ON & IESO_OFF & FCKSM_CSDCMD & POSCMOD_HS & OSCIOFNC_OFF & FNOSC_PRIPLL & PLLDIV_DIV5)// & IOL1WAY_ON)

/***** GLOBAL VARIABLES: ******************************************************/
BYTE	g_sleep_request = 0;				// flag: "1" - need to goto_sleep
char 	g_curr_msg[MAX_CMD_LEN]; 			// the message string recieved from USB port
BOOL	g_usb_connected; 					// YS 17.8 - was usb connected successfully

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
// Wisdom-Stone Application main loop
#if defined WISDOM_STONE
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/*******************************************************************************
//main()
//wait for message using polling method, then handle it; execute the loop until
//sleep_request command is received.
// - according to mode (if not IDLE):
//		- SS: sample and store block in FLASH, or
//		- TS: read from FLASH and transmit block, or
//		- OST: sample and transmit block on-line
//	- handle USB periodical tasks (we use interrupt mode)
//	- handle command (if received)
//	- handle power maintenance
//	- refresh LCD screen
*******************************************************************************/
int main(void) {
	
	//YL 5.8 - moved: PPS_config(); to init_all  	//YS 17.8	// remappable pins configs
	init_all();							// init entire system's components
	while (!g_sleep_request) { 			// continue until "app sleep" command received
		if (g_mode == MODE_SS)  		// Store and Sample
			handle_SS();
		else if (g_mode == MODE_TS) 	// Transmit Samples
			handle_TS();
		else if (g_mode == MODE_OST) 	// Online Sample and Transmit
			handle_OST();
	
		exec_message_command();			// execute commands received from: USB/RX/Boot
#ifdef LCD_INSTALLED
		refresh_screen(); 				// periodically, copy screen 4 x 16 memory buffer to LCD
		//DelayMs(1);					// to be used only when nothing is activated except for the LCD
#endif // LCD_INSTALLED	
	}
	prepare_for_shutdown();				// actions needed before going to sleep
	return(0);							// should never get here...
} 

#endif // WISDOM_STONE 


//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
// Communication Plug Application main loop
#if defined COMMUNICATION_PLUG
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/***** GLOBAL VARIABLES: ******************************************************/
//YL 4.8 was: BYTE isAppStop; <- moved isAppStop to TxRx					

/*******************************************************************************
// main()
// - while not TXRX error found
//   - if received something from USB then transmit it to TXRX
//	 - using periodic tasks in TXRX, check if received something from there and transmit it to USB
*******************************************************************************/
int main(void) {	  //YL 21.4 int main() instead of void main()
  
    //YL 5.8 - moved: PPS_config(); to init_all 	//YS 17.8	// remappable pins configs 
	init_all();	
	
	while (1) {
#if defined (USBCOM)
	USB_STATUS usbStatus = USB_ReceiveData();

	//YL 4.8... the backup is next to main
	if (usbStatus == USB_RECEIVED_DATA) {
		// parse the data from USB, to see if the command is for the plug, 
		// and if so - execute it; otherwise - we consider it "stone" command (including the case of illegal string) 
		BOOL isPlugCommand = TxRx_ExecuteIfPlugCommand();
		if (isPlugCommand == FALSE) {
			// send the command to the stone
			while (TxRx_SendCommand((BYTE*)g_curr_msg) != TXRX_NO_ERROR) {
				// keep calling TxRx_Init(TRUE) 
				// (to reset the network without resetting the data counters) 
				// as long as TxRx_SendCommand fails to transmit the command to the stone
				
				// YL 16.8 ... meanwhile reconnect doesn't work
				// TxRx_Init(TRUE);
				// ... YL 16.8
			}
		}
	}
	//...YL 4.8		
#endif // USBCOM
	// YL handle the messages that the stone sends to the plug
	TxRx_PeriodTasks();	 //YS 25.1													
	}
	//YL 5.8 commented: while (1);
	return(0); //YL 21.4 added to avoid warning in case of void main()
}
#endif // COMMUNICATION_PLUG

//YL 4.8 backup...
// if (usbStatus == USB_RECEIVED_DATA) {
	// strcpy(g_in_msg, g_curr_msg);
	
	// if (strcmp(g_curr_msg, "app stop") == 0) {								// if we received app stop, do not print the blocks from now on.		
		// isAppStop = 1;														// notify that app stop was received, and do not print the next blocks that will be received.
	// }
	// else {
		// isAppStop = 0;
	// }
	// // YS 5.1.13
	// if (!runPlugCommand()) {													//YL runPlugCommand: (g_curr_msg = "plug reconnect") ? MiApp_ResyncConnection : FALSE
		// while (TxRx_SendCommand((BYTE*)g_in_msg) != TXRX_NO_ERROR) { 		//YS 5.10 checking if resync should run //YS 17.11 //YL 21.4 added casting to g_in_msg 
			// TxRx_Init(TRUE); //YS 17.11										
		// }
	// }
//}
//...YL 4.8

