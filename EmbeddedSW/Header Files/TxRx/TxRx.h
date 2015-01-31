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

# include "wistone_main.h"
/************************ VARIABLES ********************************/
#define SCAN_SPEED 10
#define TXRX_HEADER_SIZE 3
#define TXRX_TRAILER_SIZE 4
#define RETRY_TRANSMISSION_TIMES 5
#define RESYNC_THRESHOLD 3 //YS 5.10 number of failed TXRX actions before a resync/reset occurs

// If block ack is pererable, define the following:
#define ENABLE_BLOCK_ACK

// Some debug print.. Un define it in real use..
//#define TXRX_DEBUG_PRINT

// Next are defines of times until timeout.. To change here the timing, confused between timing of message and packet..
#define TIMEOUT_RECEIVING_MESSAGE			250*ONE_MILI_SECOND //YS 25.1
#define TIMEOUT_RETRYING_RECEIVING_PACKET	2*ONE_SECOND //YS 25.1
#define TIMEOUT_SENDING_MESSAGE				250*ONE_MILI_SECOND //YS 25.1
#define TIMEOUT_RESENDING_PACKET			2*ONE_SECOND //YS 25.1

extern BYTE blockTryTxCounter;

/************************ VARIABLES ********************************/

// Next is the possible TxRx status
typedef enum
{
   	TXRX_NO_ERROR = 0,                  // No errors
    TXRX_NO_CONNECTIONS_FOUND,          // The requested device is not present
	TXRX_CANNOT_JOIN_TO_NETWORK,
    TXRX_CANNOT_INITIALIZE,             // Cannot initialize MRF
	TXRX_NO_PACKET_RECEIVED,
	TXRX_PARTIAL_PACKET_RECEIVED,
	TXRX_UNABLE_SEND_PACKET,
	TXRX_WRONG_ACK_SEQ,
	TXRX_WRONG_DATA_SEQ,
	TXRX_WRONG_PACKET_LENGTH,
	TXRX_WRONG_BLOCK_TYPE,
	TXRX_RECEIVED_UNKNOWN_PACKET,
	TXRX_RECEIVED_INVALID_PACKET,
	TXRX_RECEIVED_INVALID_TRAILER
	
}TXRX_ERRORS;



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

* Parameters:
*	   None
*
* Return value: 
*	   None
*
******************************************************************************/
void TxRx_Init(BOOL justResetNetwork);//YS 17.11


/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_PeriodTasks()
*
* Description:
*      This is the primary user interface function to perform period tasks.
*	   This funcion checks whether there is availiable message, and if so,
*	   it tries to get the whole packet, and perform the appropriate action,
*	   depends on if ir runs at the stone or at the plug. 
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
*      The next function is responsible for transmiting "regular" responses.
*      In other words, this function is the coevelent of m_write for USB.
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
*	   This functions is used in he plug.
*	   
* Example:
*		if(ProccessIO)
*		{
*			strcpy(g_in_msg,cmd_Current);	
*			TXRX_ERRORS status = TxRx_SendCommand(g_in_msg);
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

#endif



/******************************************************************************
YS 22.12
This function resets the ACK sequencers.

******************************************************************************/
void TxRx_Reset_ACK_Sequencers();

void TxRx_printError(TXRX_ERRORS error);



#endif
