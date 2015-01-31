/*******************************************************************************

Wisdome Stone - under road sensing system

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
		
********************************************************************************
	General:
	========
this is the main file of the top level application for Wisdome Stone System.
this project can be compiled with two main compiler flags:
- WISDOM_STONE
- COMMUNICATION_PLUG

Other complilation flags:
- USBCOM - use USB as the wired communication channel
- RS232  - use the UART as the wired communication channel

WISDOM_STONE
============
the main process waits for a command to be recieved from USB / UART, or from RF TXRX
(depends on the #define used).
it parses the command, and goto the relevant function.

COMMUNICATION_PLUG
==================
it can be used for the plug (USB <-> RF converter) or as the wistone itself.

It is based on the sample application that demonstrate the rich features
of MiWi(TM) Development Evniroment (DE). This demo should be used 
with FeatureDemoNode2 together. In this demo, following MiWi(TM) 
DE features has been implemented:
 - Network Freezer
     This application demonstrate how to invoke Network Freezer
     feature that restore the previous network configurations
     after reset or power cycle.
 - Active Scan 
     This application will do an active scan to allocate all PANs
     running in the neighborhood and choose the PAN that share the 
     same PAN identifier to establish connection.
 - Energy Scan
     If no existing PAN that matches the desired PAN identifier, this
     application will find a channel with least noise among all the
     supported channels.
 - Indirect Message Feature 
     This application is able to store the message to the device
     with radio off during idle and deliever the message when that 
     device wakes up and asking for it. This application is also
     capable of delivering broadcast message to each individual 
     device with radio off during idle
 - Frequency Agility 
     As a frequency agility starter, this application is able to 
     decide the optimal channel and change operating channel. It
     also has to broadcast and let other devices to change channel.
 
Change History:
Rev   Date         Author    Description
0.1   03/01/2008   yfy       Initial revision
3.1   05/28/2010   yfy       MiWi DE 3.1

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
//	- handle commad (if received)
//	- handle power maintenance
//	- refresh LCD screen
*******************************************************************************/
int main(void)
{	
	PPS_config(); //YS 17.8				// remappable pins configs
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
#endif // #ifdef LCD_INSTALLED	
	}
	prepare_for_shutdown();				// actions needed before going to sleep
	return(0);								// should never get here...
} 

#endif // #if defined WISDOM_STONE 


//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
// Communication Plug Application main loop
#if defined COMMUNICATION_PLUG
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/***** GLOBAL VARIABLES: ******************************************************/
BYTE isAppStop;

/*******************************************************************************
// main()
// - while not RXTX error found
//   - if received something from USB then transmit it to TXRX
//	 - using periodic tasks in TXRX, check if received something from there and transmit it to USB
*******************************************************************************/
void main(void)
{   
    PPS_config();	//YS 17.8		// remappable pins configs
	init_all();	
	
	while (1) {
#if defined (USBCOM)
		USB_STATUS usbStatus = USB_ReceiveData();

		if (usbStatus == USB_RECEIVED_DATA) {
			strcpy(g_in_msg, g_curr_msg);
			if (strcmp(g_curr_msg, "app stop") == 0) {	// If we received app stop, do not print the blocks from now on..		//YL 11.11 strcmp instead of strcmp_ws
				isAppStop = 1;	// notify that app stop was received, and do not print the next blcks that will bbe received..
			}
			else {
				isAppStop = 0;
			}
			//YS 5.1.13
			if (!runPlugCommand()) {
				while (TxRx_SendCommand(g_in_msg) != TXRX_NO_ERROR) {//YS 5.10 checking if resync should run//YS 17.11
					TxRx_Init(TRUE); //YS 17.11
				}
			}
		}
#endif // #if defined (USBCOM)
		
		//TXRX_ERRORS status = TxRx_PeriodTasks();//YS 25.1
		TxRx_PeriodTasks();//YS 25.1
		/*if (status != TXRX_NO_ERROR) {
			TxRx_printError(status);
			TxRx_Init(TRUE); //YS 17.11
		}*/ //YS 25.1
	}
	while (1);
}
#endif // #if defined COMMUNICATION_PLUG
