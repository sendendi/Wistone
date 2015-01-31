/*********************************************************************
 *                                                                    
 * Software License Agreement                                         
 *                                                                    
 * Copyright © 2007-2010 Microchip Technology Inc.  All rights reserved.
 *
 * Microchip licenses to you the right to use, modify, copy and distribute 
 * Software only when embedded on a Microchip microcontroller or digital 
 * signal controller and used with a Microchip radio frequency transceiver, 
 * which are integrated into your product or third party product (pursuant 
 * to the terms in the accompanying license agreement).   
 *
 * You should refer to the license agreement accompanying this Software for 
 * additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY 
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY 
 * WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A 
 * PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE 
 * LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, 
 * CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY 
 * DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO 
 * ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, 
 * LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, 
 * TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT 
 * NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.             
 *                                                                    
 *********************************************************************/
#ifndef __TX_RX_H_
#define __TX_RX_H_

#include "wistone_main.h"
/************************ VARIABLES ********************************/
#define SCAN_SPEED 					10
#define TXRX_HEADER_SIZE 			3
#define TXRX_TRAILER_SIZE 			4
// YL 2.9 ... isn't used
// was uncommented
// #define RETRY_TRANSMISSION_TIMES 	5
// ... YL 2.9
#define RESYNC_THRESHOLD 			3 	// YS 5.10 number of failed TXRX actions before a resync/reset occurs
// YL 4.8 ...
// YL 18.8 ...
#define MAX_NWK_SIZE				2				// the number of devices in the network (including the plug) 
// ... YL 18.8
#define MAX_ACK_LENGTH				60  			// 60 bits
#define MAX_NWK_ADDR_EUI0			MAX_NWK_SIZE 	// the max address of a device in the network
// YL 20.8 ...
// YL 24.9 ...
// was: #define PLUG_NWK_ADDR				1
#define PLUG_NWK_ADDR_EUI0			MAX_NWK_SIZE	// the EUI0 network address of the plug (the plug has the highest EUI[0])
#define PLUG_NWK_ADDR_EUI1			EUI_1			// the EUI1 network address of the plug (constant)
// ... YL 24.9
#define NWK_STARTER_ADDR_EUI0		1				// the network address of the stone that calls MiApp_StartConnection
// ... YL 20.8
#define BROADCAST_NWK_ADDR			0				// the network address for broadcast command

// ... YL 4.8
// YL 14.8 ...
#define MSG_INF_LENGTH				1   // 1 byte with ack and type information of the message
#define MSG_LEN_LENGTH				2   // 2 bytes with the length of the message
// ... YL 14.8

// If block ack is preferable, define the following:
#define ENABLE_BLOCK_ACK	// YL TODO - add if defined ENABLE_ACK

// Next are defines of times until timeout.. To change here the timing, confused between timing of message and packet..
#define TIMEOUT_RECEIVING_MESSAGE							250 * ONE_MILI_SECOND 	//YS 25.1
#define TIMEOUT_RETRYING_RECEIVING_PACKET 					2 * ONE_SECOND 			//YS 25.1
// YL 18.8 isn't used: #define TIMEOUT_SENDING_MESSAGE		250 * ONE_MILI_SECOND 	//YS 25.1
#define TIMEOUT_RESENDING_PACKET 							2 * ONE_SECOND 			//YS 25.1
// ... YL 17.8

// YL 24.9 ...
#define TIMEOUT_NWK_STARTING			50 * ONE_SECOND		// timeout for the starter stone to join the network
#define TIMEOUT_NWK_JOINING				100 * ONE_SECOND	// timeout for the rest to join the network
#define TIMEOUT_NWK_ESTABLISHMENT		TIMEOUT_NWK_JOINING	// the plug waits maximum TIMEOUT_NWK_JOINING (maximum time each stone attempts to join the network) 
// ... YL 24.9

// YL 2.9 ... added #ifdef
// was: extern BYTE blockTryTxCounter;
#if defined ENABLE_RETRANSMISSION
	extern BYTE blockTryTxCounter;
#endif
// ... YL 2.9

// YL 4.8 ... 
extern BOOL isBroadcast;					// we received broadcast command
extern BOOL isAppStop;						// we received "app stop" command
// ... YL 4.8

/************************ VARIABLES ********************************/

// Next is the possible TxRx status
typedef enum {
   	TXRX_NO_ERROR = 0,                  // No errors
    // YL 19.8 isn't used: TXRX_NO_CONNECTIONS_FOUND,          // The requested device is not present
	// YL 19.8 isn't used: TXRX_CANNOT_JOIN_TO_NETWORK,
    // YL 19.8 isn't used: TXRX_CANNOT_INITIALIZE,             // Cannot initialize MRF
	TXRX_NO_PACKET_RECEIVED,
	TXRX_PARTIAL_PACKET_RECEIVED,
	TXRX_UNABLE_SEND_PACKET,
	TXRX_WRONG_ACK_SEQ,
	TXRX_WRONG_DATA_SEQ,
	TXRX_WRONG_PACKET_LENGTH,
	TXRX_WRONG_BLOCK_TYPE,
	TXRX_RECEIVED_UNKNOWN_PACKET,
	TXRX_RECEIVED_INVALID_PACKET,
	TXRX_RECEIVED_INVALID_TRAILER,
	TXRX_NWK_UNKNOWN_ADDR,				// YL 4.8 invalid network address of the destination
	TXRX_NWK_NOT_ME,					// YL 23.7 the stone received a command that wasn't addressed to it
	TXRX_ERROR_MAX,						// YL 14.8 
} TXRX_ERRORS;


/************************ FUNCTIONS ********************************/
void MRFInit();

/******************************************************************************
* Function:
*		void TxRx_Init(BOOL justResetNetwork)
*
* Description:
*      This is the primary user interface function to initialize the TxRx module.
*	   This function should be called after the	hardware initialization before 
*	   any other TxRx interface can be called.
*
* Example:
*      <code>
*      HardwareInit();
*      TxRx_Init();
*      </code>
*
* Parameters:
*	   BOOL justResetNetwork - to indicate if we only want to reset the networking stack, and not data counters (for network faults)
*
* Return value: 
*	   None
*
******************************************************************************/
void TxRx_Init(BOOL justResetNetwork); //YS 17.11


/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_PeriodTasks()
*
* Description:
*      This is the primary user interface function to perform period tasks.
*	   This function checks whether there is available message, and if so,
*	   it tries to get the whole packet, and perform the appropriate action,
*	   depends on if it runs at the stone or at the plug. 
*
* Example: 	void main(void)
*    		{
*        		HardwareInit()
*        		while(1)
*        		{
*            		TXRX_ERRORS status = TxRx_PeriodTasks();
*            		if((status != TXRX_NO_ERROR)
*            		{
*                		// Error handling
*                		continue;   //go back to the top of the while loop
*            		}
*            		else
*            		{
*                		//Otherwise we are free to run user application code.
*                		UserApplication();
*            		}
*        		}
*    		}
*
* Parameters:
*	   None
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*
******************************************************************************/
TXRX_ERRORS TxRx_PeriodTasks();



 #if defined WISDOM_STONE

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_SendData(BYTE *samples_block, WORD dataLen)
*
* Description:
*      This function sending a data packet to the other device. This function is
*	   the previously TXBlock() function. It uses that TxRx_SendPacketWithConfimation()
*	   function to accomplish this aim.
*
* Parameters:
*	   samples_block - The data we want to send - may be data or command.
*	   dataLen - Length of the data
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*	   
*
******************************************************************************/
TXRX_ERRORS TxRx_SendData(BYTE* samples_block, WORD TX_message_length);

/******************************************************************************
* Function:
*		TXRX_ERRORS m_TxRx_write(BYTE *str)
*
* Description:
*      The next function is responsible for transmitting "regular" responses.
*      In other words, this function is the equivalent of m_write for USB.
*	   This function is used in the stone.
*
* Parameters:
*	   str - The string you want to send.
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*	   
*
******************************************************************************/
TXRX_ERRORS m_TxRx_write(BYTE *str);


#elif defined (COMMUNICATION_PLUG)

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_SendCommand(BYTE *command)
*
* Description:
*      This function sending a command packet to the other device. It uses the
*	   TxRx_SendPacketWithConfimation() function to accomplish this aim.
*	   This functions is used in the plug.
*	   
* Example:
*		if(ProccessIO)
*		{
*			strcpy(msg, cmd_Current);	
*			TXRX_ERRORS status = TxRx_SendCommand(msg);
*			if(status != TXRX_NO_ERROR)
*			{
*				// Error handling
*			}
*		}
*		<code>
*
*
* Parameters:
*	   command - The command you want to send.
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*	   
*
******************************************************************************/
TXRX_ERRORS TxRx_SendCommand(BYTE *command);

#endif //COMMUNICATION_PLUG

/******************************************************************************
YS 22.12
This function resets the ACK sequencers.

******************************************************************************/
void TxRx_Reset_ACK_Sequencers();

// YL 20.8 ...
// was: void TxRx_printError(TXRX_ERRORS error);
/******************************************************************************
* Function:
*		int TxRx_PrintError(TXRX_ERRORS error) 
******************************************************************************/	
int TxRx_PrintError(TXRX_ERRORS error);	
// ... YL 20.8

// YL 4.8 moved runPlugCommand here from app file and renamed it to TxRx_Reconnect...
#if defined (COMMUNICATION_PLUG)

/******************************************************************************
* Function:
*		void TxRx_Reconnect(void) 
*******************************************************************************/
void TxRx_Reconnect(void);

//BACKUP:
/******************************************************************************
// TxRx_Reconnect()
// if it is a plug command, run it, otherwise don't.
// returns a boolean to indicate if it was a valid plug command.
*******************************************************************************/
//BOOL TxRx_Reconnect(void);
// ... YL 4.8

// YL 4.8 ...
/******************************************************************************
* Function:
*		BOOL TxRx_ExecuteIfPlugCommand(void) 
*******************************************************************************/	
BOOL TxRx_ExecuteIfPlugCommand(void);

// YL 24.9 ...
/******************************************************************************
* Function:
*		void TxRx_PrintNetworkTopology(void) 
*******************************************************************************/
void TxRx_PrintNetworkTopology(void);
// ... YL 24.9

#endif // #ifdef COMMUNICATION_PLUG
// ... YL 4.8

// YL 18.8 ...
// <> #if defined DEBUG_PRINT
/******************************************************************************
* Function:
*		void TxRx_PrintConnectionTable(void) 
*******************************************************************************/	
void TxRx_PrintConnectionTable(void);
// #endif
// ... YL 18.8

#endif //__TX_RX_H_
