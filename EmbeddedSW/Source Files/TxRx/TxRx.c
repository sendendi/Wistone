/******************************************************************************
// Summary of TxRx.c:
// The TxRx.c file contains 3 types of functions:
// 1. Function that are common to the stone and to the plug
// 2. Function that are unique to the stone
// 3. Functions that are unique to the PLUG
// The functions in this file are orderd as follow: In the begginig that are the common
// function and so on..
******************************************************************************/

/************************ HEADERS **********************************/
#include "wistone_main.h"
#include "WirelessProtocols/P2P/P2P.h"
//#include "WirelessProtocols/SymbolTime.h"
#include "Transceivers/Transceivers.h"
#include "WirelessProtocols/MCHP_API.h"
//#include "WirelessProtocols/NVM.h"
#include "TxRx.h"
#include "Accelerometer.h"
#include "HardwareProfileTxRx.h"
#include "ConfigApp.h"
#include "misc_c.h"			//YL 10.8
#include "command.h"		//YL 12.8

/************************** MACROS *********************************/

static char *TxRx_err_messages[] = {
	"TxRx -  No Error",
    "TxRx - No connection found",           // The requested device is not present
	"TxRx - Can not join to a network",
	"TxRx - Can Not Intialize device",         // Cannot initialize device
	"TxRx - Did not receive packet",
	"TxRx - Got only partial packet",
	"TxRx - Unable to send a packet",
	"TxRx - Received wrong ack sequence number",
	"TxRx - Received wrong data sequence number",
	"TxRx - Received wrong packet length",
	"TxRx - Received wrong block type",
	"TxRx - Received unknown packet",
	"TxRx - Received invalid packet",
	"TxRx - Received invalid packet trailer",
	""
};

// The trailer is constant, and is used for syncronization after each block.
static BYTE TxRx_Trailer[TXRX_TRAILER_SIZE] = {
	0x25, 0x83, 0x33, 0x57
};

// There are 3 command types - the regular one, which consists of command and their response. 
//The second type is data block, which consists of block in size of 512. And the third and last
// is ack packet.

typedef enum{
	TXRX_TYPE_COMMAND 	= 0,
	TXRX_TYPE_DATA 	 	= 1,
	TXRX_TYPE_ACK		= 2
}BLOCK_TYPE;


// Next is the struct of transmission block. It consists block header, buffer and trailer, and current position
// in the block.
// The header consists of block type, ackSeq and block length (without header and trailer).
typedef struct {

	struct{
		BYTE blockType;
		BYTE ackSeq;
		WORD blockLen;
	}blockHeader;
	
	BYTE blockBuffer[MAX_BLOCK_SIZE+10];
	BYTE blockTrailer[TXRX_TRAILER_SIZE];

	WORD blockPos;
	
	
}TX_BLOCK_BUFFER;


// Next is the struct of receiving block. It consists block header, buffer and trailer, and some
// other information of the block for handling it(The last is not part of the block itself).
// The header consists of block type, ackSeq and block length (without header and trailer).
typedef struct {

	struct{
		BYTE blockType;		
		WORD blockLen;
	}blockHeader;

	BYTE blockBuffer[MAX_BLOCK_SIZE+10];
	BYTE blockTrailer[TXRX_TRAILER_SIZE];

	struct{
		WORD blockPos;
		BOOL isHeader;	
		BOOL isTrailer;
	}handlingParam;


}RX_BLOCK_BUFFER;


// Next struct consists information of the acknolegmaents. There are acknolegment sequences for transmiting data,
// and acknowlogment for receiving data. The ..LastSeq descrives the sequence of the last successful transminit/receiving,
// and the ..ExpectedSeq describes the ack that we are expect to get.
typedef struct{

	BYTE txLastSeq;
	BYTE txExpectedSeq;
	BYTE rxLastSeq;
	BYTE rxExpectedSeq;

}BLOCK_ACK_INFO;



/************************ VARIABLES ********************************/
TX_BLOCK_BUFFER txBlock;
RX_BLOCK_BUFFER rxBlock;
BLOCK_ACK_INFO  blockAckInfo;

// Next global variables are only for receiving commands during transsmission of blocks.
// Notice that when we use acks, and the stone is waiting for an ack, it can receive
// in this time a command (for instance "app stop", therefore we should seperate between
// these by the next 2 variables:
 
#if defined (WISDOM_STONE)
	//extern BYTE g_is_cmd_received;	// indicates that command was received during a transsmision. 
#endif

#if defined (COMMUNICATION_PLUG)
// indicates for the plug that "app stop" was s-e-n-t to the stone, and therefore the next block that will be received, will not be printed..
	extern BYTE isAppStop;	
#endif

BYTE blockTryTxCounter = 0;

extern BYTE messageRetryCounter;

/********************************************************************/
// Function Declarations:
/********************************************************************/

// Overall functions
TXRX_ERRORS TxRx_WistoneHandler();
TXRX_ERRORS TxRx_PlugHandler();

// Tx Function - To change the name of th function to more understnable...
TXRX_ERRORS TxRx_TransmitBuffer();
TXRX_ERRORS TxRx_SendPacket(BYTE *data, WORD dataLen, BLOCK_TYPE bType);

#if defined (WISDOM_STONE)
	TXRX_ERRORS TxRx_SendData(BYTE* samples_block, WORD TX_message_length);	
	TXRX_ERRORS m_TxRx_write(BYTE *str);

#elif defined (COMMUNICATION_PLUG)
	TXRX_ERRORS TxRx_SendCommand(BYTE* command);
#endif


TXRX_ERRORS TxRx_SendAck();

BYTE TxRx_noOverflowADD(BYTE toAdd, BYTE addingTo);


// Rx Function
TXRX_ERRORS TxRx_ReceiveHeaderPacket();
void TxRx_ReceiveBuffer();
TXRX_ERRORS TxRx_ReceiveMessage();
TXRX_ERRORS TxRx_ReceivePacket();


#if defined (COMMUNICATION_PLUG)
	TXRX_ERRORS TxRx_Connect();
#endif



/******************************************************************************
* Function:
*		void MRFInit()
*
* Description:
*      This is the primary user interface function to initialize the MRF.
*	   This fnction intializes the MRF ports, the SPI interface used by
*	   the MRFs, and enabling the interrupt to interrupt1.
*
* Example:
*      <code>
*	   Hardware_Init();
*      MRFInit();
*      </code>
*
* Parameters:
*	   None
*
* Return value: 
*	   None
*
******************************************************************************/


void MRFInit(){


#if defined(MRF24J40) || defined(MRF49XA)
    PHY_CS_TRIS = 0;
    PHY_CS = 1;
    //PHY_RESETn_TRIS = 0;
    //PHY_RESETn = 1;
#else
	#error "Invalid MRF"
#endif
	
	RF_INT_TRIS = 1;
	
	SDI_TRIS = 1;
	SDO_TRIS = 0;
	SCK_TRIS = 0;
	SPI_SDO = 0;        
	SPI_SCK = 0;             
	
#if defined(MRF49XA)
    nFSEL_TRIS = 0;
    FINT_TRIS = 1;	    
    nFSEL = 1; 
   
#elif defined(MRF24J40)
    PHY_WAKE_TRIS = 0;
    PHY_WAKE = 1;	
#endif
	
#if defined(HARDWARE_SPI)
     SPI2CON1 = 0b0000000100111010;
     SPI2STAT = 0x8000;	
#else
	#error "Not defined Hardware SPI"
#endif
	
	INTCON2bits.INT1EP = 1;
	
#if defined(ENABLE_NVM)
    EE_nCS_TRIS = 0;
    EE_nCS = 1;
#endif
	
	RFIF = 0;
	if( RF_INT_PIN == 0 )
	{
	    RFIF = 1;
	}
        
}


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
*	   BOOL justResetNetwork- to indicate if we only want to reset the networking stack, and not data counters (for network faults)
*
* Return value: 
*	   None
*
******************************************************************************/
void TxRx_Init(BOOL justResetNetwork){//YS 17.11
	long	currentChannel_long;
	char*	currentChannel_str;

	// Initialize the MRF ports.
	MRFInit();
	

	/*********************************************************************/
    // Function MiApp_ProtocolInit intialize the protocol stack.
    // The return value is a boolean to indicate the status of the
    //      operation.
    // The only parameter indicates if Network Freezer should be invoked.
    //      When Network Freezer feature is invoked, all previous network
    //      configurations will be restored to the states before the
    //      reset or power cycle
    /*********************************************************************/
    MiApp_ProtocolInit(FALSE);

	if(!justResetNetwork){
		// Next are init required for the acks.
		TxRx_Reset_ACK_Sequencers();//YS 22.12
	

		// Next are the init of the trailer that is conastant and used for syncronization
		// of the block
		WORD i = 0;
	
		for(i = 0; i < TXRX_TRAILER_SIZE; i++)
		{
			txBlock.blockTrailer[i] = TxRx_Trailer[i];
		}
	}
	/*******************************************************************/
    // Function MiApp_ConnectionMode sets the connection mode for the 
    // protocol stack. Possible connection modes are:
    //  - ENABLE_ALL_CONN       accept all connection request
    //  - ENABLE_PREV_CONN      accept only known device to connect
    //  - ENABL_ACTIVE_SCAN_RSP do not accept connection request, but 
    //                          allow response to active scan
    //  - DISABLE_ALL_CONN      disable all connection request, including
    //                          active scan request
    /*******************************************************************/
	MiApp_ConnectionMode(ENABLE_ALL_CONN);

#if defined (COMMUNICATION_PLUG)
	while(TxRx_Connect()!= TXRX_NO_ERROR);
	DumpConnection(0xFF);                    
	print_string(0, 0, "Start Conn Chan#");
	currentChannel_long = currentChannel & 0x0000FFFF;
	currentChannel_str = long_to_str(currentChannel_long); //YL 22.8 long_to_str (instead of num_to_str)
	print_string(0, 1, currentChannel_str);

#elif defined (WISDOM_STONE)

        {
            /*******************************************************************/
            // Function MiApp_StartConnection tries to start a new network
            //
            // The first parameter is the mode of start connection. There are 
            // two valid connection modes:
            //   - START_CONN_DIRECT        start the connection on current 
            //                              channel
            //   - START_CONN_ENERGY_SCN    perform an energy scan first, 
            //                              before starting the connection on 
            //                              the channel with least noise
            //   - START_CONN_CS_SCN        perform a carrier sense scan 
            //                              first, before starting the 
            //                              connection on the channel with 
            //                              least carrier sense noise. Not
            //                              supported for current radios
            //
            // The second parameter is the scan duration, which has the same 
            //     definition in Energy Scan. 10 is roughly 1 second. 9 is a 
            //     half second and 11 is 2 seconds. Maximum scan duration is 
            //     14, or roughly 16 seconds.
            //
            // The third parameter is the channel map. Bit 0 of the 
            //     double word parameter represents channel 0. For the 2.4GHz 
            //     frequency band, all possible channels are channel 11 to 
            //     channel 26. As the result, the bit map is 0x07FFF800. Stack 
            //     will filter out all invalid channels, so the application 
            //     only needs to pay attention to the channels that are not 
            //     preferred.
            /*******************************************************************/
			print_string(0, 0, "Energy Scanning");
			MiApp_StartConnection(START_CONN_ENERGY_SCN, 8, 0xFFFFFFFF);
			//TXSleep();
			DumpConnection(0xFF);                    
 			print_string(0, 0, "Start Conn Chan#");
			currentChannel_long = currentChannel & 0x0000FFFF;
			currentChannel_str = long_to_str(currentChannel_long);  //YL 22.8 long_to_str (instead of num_to_str)
			print_string(0, 1, currentChannel_str);
        }
#else
 			print_string(0, 0, "define either:");
 			print_string(0, 1, "STONE or PLUG");
#endif

	if(justResetNetwork){//YS 17.11
		while(!ConnectionTable[0].status.bits.isValid);//make sure the other side reconnected
	}
}

#if defined (COMMUNICATION_PLUG)


/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_Connect()
*
* Description:
*      This function should be called to connect to available connection.
*	   Notice that that the plug is connecting to the stone, therefore the stone
*	   is builiding the network, and the plug is connecting to the network. 
*
* Parameters:
*	   None
*
* Return value: 
*	   TXRX_NO_CONNECTIONS_FOUND
*	   TXRX_CANNOT_JOIN_TO_NETWORK
*
******************************************************************************/


TXRX_ERRORS TxRx_Connect(){

	BYTE i = 0,j;
        
    WORD myChannel = 0xFF;
    m_write((char *)"\r\nStarting Active Scan...");
    
	print_string(0, 0, "Active Scanning");

    /*******************************************************************/
    // Function MiApp_SearchConnection will return the number of 
    // existing connections in all channels. It will help to decide 
    // which channel to operate on and which connection to add.
    // The return value is the number of connections. The connection 
    //     data are stored in global variable ActiveScanResults. 
    //     Maximum active scan result is defined as 
    //     ACTIVE_SCAN_RESULT_SIZE
    // The first parameter is the scan duration, which has the same 
    //     definition in Energy Scan. 10 is roughly 1 second. 9 is a 
    //     half second and 11 is 2 seconds. Maximum scan duration is 14, 
    //     or roughly 16 seconds.
    // The second parameter is the channel map. Bit 0 of the 
    //     double word parameter represents channel 0. For the 2.4GHz 
    //     frequency band, all possible channels are channel 11 to 
    //     channel 26. As the result, the bit map is 0x07FFF800. Stack 
    //     will filter out all invalid channels, so the application 
    //     only needs to pay attention to the channels that are not 
    //     preferred.
    /*******************************************************************/
    i = MiApp_SearchConnection(8, 0xFFFFFFFF);
    
	if (i==0){
		return TXRX_NO_CONNECTIONS_FOUND;
	}

    if( i > 0 )
    {
        // now print out the scan result.
        m_write("\r\nActive Scan Results: \r\n");
        for(j = 0; j < i; j++)
        {
            m_write("Channel: ");
            PrintDec(ActiveScanResults[j].Channel );
            m_write("   RSSI: ");
            PrintChar(ActiveScanResults[j].RSSIValue);
            m_write("\r\n");
            myChannel = ActiveScanResults[j].Channel;
        }

		/*******************************************************************/
		// Function MiApp_EstablishConnection try to establish a new 
		// connection with peer device. 
		// The first parameter is the index to the active scan result, which 
		//      is acquired by discovery process (active scan). If the value
		//      of the index is 0xFF, try to establish a connection with any 
		//      peer.
		// The second parameter is the mode to establish connection, either 
		//      direct or indirect. Direct mode means connection within the 
		//      radio range; Indirect mode means connection may or may not 
		//      in the radio range. 
		/*******************************************************************/
		if( MiApp_EstablishConnection(0, CONN_MODE_DIRECT) == 0xFF )
		{
			m_write("\r\nJoin Fail");
			return TXRX_CANNOT_JOIN_TO_NETWORK;
		}
    }
   
	TxRx_Reset_ACK_Sequencers();//YS 22.12
	return TXRX_NO_ERROR;

}

#endif



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

TXRX_ERRORS TxRx_PeriodTasks(){

	
	TXRX_ERRORS status;
	MIWI_TICK t1,t2;//YS 25.1
	// Check if there is available message..
	if( MiApp_MessageAvailable())
	{	
		t1 = MiWi_TickGet();
		while(1){
			status = TxRx_ReceivePacket();		
			if(status == TXRX_NO_ERROR){
				break;
			}
			#if defined (TXRX_DEBUG_PRINT)
				TxRx_printError(status);
			#endif

			t2 = MiWi_TickGet();//YS 25.1
			if(MiWi_TickGetDiff(t2, t1) > TIMEOUT_RETRYING_RECEIVING_PACKET){
				return TXRX_NO_ERROR;
			}
		}
		
		#if defined (WISDOM_STONE)
			TxRx_WistoneHandler();	//YS 25.1

		#elif defined (COMMUNICATION_PLUG)
			TxRx_PlugHandler();//YS 25.1

		#endif
		//return status; YS 25.1
	}

	return TXRX_NO_ERROR;


}

/******************************************************************************
******************************************************************************/

#if defined (WISDOM_STONE)

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_WistoneHandler()
*
* Description:
*      This is an interface function to perform period tasks in the
*	   stone.
*	   This function is called in the TxRx_PeriodTasks. The function checks
*	   what is the type of the received packet, and proccesing it.
*
* Parameters:
*	   None
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*
******************************************************************************/

TXRX_ERRORS TxRx_WistoneHandler(){	

	
	if(rxBlock.blockHeader.blockType == TXRX_TYPE_DATA){ // It is impossible that the block type here is data, since this code is running in the stone..			
		return TXRX_RECEIVED_INVALID_PACKET;
	}
	
	#if defined (ENABLE_BLOCK_ACK)

		if(rxBlock.blockHeader.blockType == TXRX_TYPE_ACK){	// meaning we received an ack, nothing to do..
			return TXRX_NO_ERROR;
		}
		
		else{	// meaning it is a command, then we need to send ACK..
			TXRX_ERRORS status = TxRx_SendAck();

			if(status != TXRX_NO_ERROR){
				return status;
			}
		}
	#endif
	
	// Copy the input command to g_in_msg array
	strcpy(g_in_msg, rxBlock.blockBuffer);	// to check that RX_block_buffer is not bigger than 100


	return TXRX_NO_ERROR;
	
}

#endif


#if defined (COMMUNICATION_PLUG)

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_PlugHandler()
*
* Description:
*      This is an interface function to perform period tasks in the
*	   plug.
*	   This function is called in the TxRx_PeriodTasks. The function checks
*	   what is the type of the received packet, and proccesing it.
*
* Parameters:
*	   None
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*
******************************************************************************/
TXRX_ERRORS TxRx_PlugHandler(){

	
	BYTE isBlockNeedToBePrinted = 1;

	#if defined (ENABLE_BLOCK_ACK)
		TXRX_ERRORS status;
		if((rxBlock.blockHeader.blockType == TXRX_TYPE_COMMAND) || (rxBlock.blockHeader.blockType == TXRX_TYPE_DATA)){	// Need to send ack for the command/data
			if(blockAckInfo.rxLastSeq == blockAckInfo.rxExpectedSeq) {	// meaning we received again a block that we handled before, since the ack was unsuccessuful.
				isBlockNeedToBePrinted = 0;
			}
			status = TxRx_SendAck();
			if(status != TXRX_NO_ERROR){
				return status;
			}
		}

		else{ // meaning it is ack..
			return TXRX_NO_ERROR;
		}
	#endif
	
	WORD i = 0;
	//RFIE = 0;

	if(isBlockNeedToBePrinted == 0){	// Again, if we received a block that handled before and the ack was bot received by the stone.
		return TXRX_NO_ERROR;
	}


	if(rxBlock.blockHeader.blockType == TXRX_TYPE_DATA){	// if we received data, print it by b_write..
		if(isAppStop == 0){	// Do not print the last block that was received..
			b_write(rxBlock.blockBuffer, MAX_BLOCK_SIZE);
 		}
		
	}
			
	else if(rxBlock.blockHeader.blockType == TXRX_TYPE_COMMAND){ // if we received command, print it by m_write..
		m_write(rxBlock.blockBuffer);
	}
	
	//RFIE = 1;


	return TXRX_NO_ERROR;
}


#endif


/******************************************************************************
******************************************************************************/
/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_TransmitBuffer()
*
* Description:
*      This function performs the tranmission to the other device. It transmits
*	   the header, the bufffer itself and the trailer. All this data should be
*	   filled in the txBlock , and it is done in TxRx_SendPacket function.
*
* Parameters:
*	   None
*
* Return value: 
*	   TXRX_NO_ERROR
*	   TXRX_UNABLE_SEND_PACKET
*
******************************************************************************/
TXRX_ERRORS TxRx_TransmitBuffer(){
	
	MIWI_TICK t1;
	
	WORD trailerPos = 0;
	txBlock.blockPos = 0;

	MiApp_FlushTx(); 

	BYTE blockInf = (txBlock.blockHeader.blockType) & 0x03;
	blockInf |= ((txBlock.blockHeader.ackSeq <<2)&0xFC);
	MiApp_WriteData(blockInf);

	if(txBlock.blockHeader.blockLen > 512){
		txBlock.blockHeader.blockLen = 512;
	}

	BYTE blockLen = (BYTE)((txBlock.blockHeader.blockLen>>8)&(0x00FF));
	MiApp_WriteData(blockLen);
	blockLen = (BYTE)((txBlock.blockHeader.blockLen)&(0x00FF));
	MiApp_WriteData(blockLen);	
	
	WORD message_counter = 3;

	while(txBlock.blockPos < (txBlock.blockHeader.blockLen + TXRX_TRAILER_SIZE)){
		
		if(message_counter == 0){
			MiApp_FlushTx();
		}

		if(txBlock.blockPos < txBlock.blockHeader.blockLen){	// meaning we are writing the buffer itself..
			MiApp_WriteData(txBlock.blockBuffer[txBlock.blockPos++]);
		}
		else{	// meaning we writng the trailer..
			MiApp_WriteData(txBlock.blockTrailer[trailerPos++]);
			txBlock.blockPos++;
		}

		message_counter++;

		if(message_counter == TX_BUFFER_SIZE){			
			message_counter = 0;	

			t1 = MiWi_TickGet();
			while(MiApp_UnicastConnection(0, FALSE)==FALSE){
//YS 30.11 start	
				blockTryTxCounter = TxRx_noOverflowADD(5, blockTryTxCounter);	// 5 is RETRANSSMISION_TIMES
				//YS 30.11
				//t2 = MiWi_TickGet();
				//if(MiWi_TickGetDiff(t2, t1) > TIMEOUT_SENDING_MESSAGE){	// Waits 1 second to get the whole command
				//	return TXRX_UNABLE_SEND_PACKET;
				//}
			}
			blockTryTxCounter = TxRx_noOverflowADD(messageRetryCounter, blockTryTxCounter);	// add the number of transmission needed in the lower level.
		}
//YS 30.11 end
	}

	if(message_counter > 0){
		
		t1 = MiWi_TickGet();	
		while(MiApp_UnicastConnection(0, FALSE)==FALSE){
//YS 30.11 start
			blockTryTxCounter = TxRx_noOverflowADD(5, blockTryTxCounter);	// 5 is RETRANSSMISION_TIMES
//			t2 = MiWi_TickGet();
//			if(MiWi_TickGetDiff(t2, t1) > TIMEOUT_SENDING_MESSAGE){	// Waits 1 second to get the whole command
//				return TXRX_UNABLE_SEND_PACKET;
//			}
		}
		blockTryTxCounter = TxRx_noOverflowADD(messageRetryCounter, blockTryTxCounter);// add the number of transmission needed in the lower level.
	}
//YS 30.11 end
	return TXRX_NO_ERROR;
}

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_SendPacket(BYTE *data, WORD dataLen, BLOCK_TYPE bType)
*
* Description:
*      This function sending a packet to the other device. This packet can be 
*	   either command or data or ack. The function fills the txBlock with the 
*	   data. Then it used the TxRx_TransmitBuffer() function to send this
*	   data. Finally, if the packet is command or data, it waits for ack for 
*	   the packet.
*
* Parameters:
*	   data - The data we want to send - may be data or command.
*	   dataLen - Length of the data
*	   bType - The type of data we want to send - may be either command or data or ack.
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*	   
*
******************************************************************************/

//We assume that this function may be interruptible at any stage.

TXRX_ERRORS TxRx_SendPacket(BYTE *data, WORD dataLen, BLOCK_TYPE bType){

	TXRX_ERRORS status;
	txBlock.blockHeader.blockType = bType;	// getting the block type we want to send

	#if defined (ENABLE_BLOCK_ACK)

		if(bType == TXRX_TYPE_ACK){	// if it is ack
			txBlock.blockHeader.ackSeq = blockAckInfo.rxExpectedSeq;
			txBlock.blockHeader.blockLen = 0;
			status = TxRx_TransmitBuffer();
			return status;
		}
		//else - meaning that it is command or data
	
		blockAckInfo.txExpectedSeq = (blockAckInfo.txLastSeq + 1) % 60; 
		txBlock.blockHeader.ackSeq =  blockAckInfo.txExpectedSeq;

	#endif

	txBlock.blockHeader.blockLen = dataLen;

	if(txBlock.blockHeader.blockLen > 512){
		txBlock.blockHeader.blockLen = 512;
	}

	WORD i;
	for(i=0; i<txBlock.blockHeader.blockLen; i++){
		txBlock.blockBuffer[i] = data[i];
	}

//-	Initiate transmission (to USB/Wireless)

	status = TxRx_TransmitBuffer();
	
	#if defined (ENABLE_BLOCK_ACK)

		if (status != TXRX_NO_ERROR){
			return status;
	 	}
	
		status = TxRx_ReceivePacket();	// Wating for ack to arrive about the packet, never mind if it is ack of command or data
		if(status != TXRX_NO_ERROR){
			return status;
		}
		
		if(rxBlock.blockHeader.blockType == TXRX_TYPE_ACK){ // meaning it is indeed ack
			return TXRX_NO_ERROR;			
		}

		#if defined (WISDOM_STONE)

			else if(rxBlock.blockHeader.blockType == TXRX_TYPE_COMMAND){	// if we received app stop in the middle of transsmission..
				strcpy(g_in_msg, rxBlock.blockBuffer);	// to check that RX_block_buffer is not bigger than 100
				TXRX_ERRORS status = TxRx_SendAck();	// send ack to the command

				if(status != TXRX_NO_ERROR){
					return status;
				}
				g_is_cmd_received = 1;	// indicates that a command was received during this period - inform the upper level by setting this variable.

				status = TxRx_ReceivePacket();	// Wating for ack to arrive about the original packet, never mind if it is ack of command or data
				if(status != TXRX_NO_ERROR){
					return status;
				}
	
				if(rxBlock.blockHeader.blockType == TXRX_TYPE_ACK){ // meaning it is indeed ack
					return TXRX_NO_ERROR;			
				}	
			}
	

	
		#endif

		return TXRX_RECEIVED_UNKNOWN_PACKET;

	#else
		return status;		
	#endif
		
}

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_SendPacketWithConfirmation(BYTE *data, WORD dataLen, BLOCK_TYPE bType)
*
* Description:
*      This function sending a packet to the other device, and resending it if needed.
*	   It uses the TxRx_SendPacket() function. If the last fails, it continue resending it 
*	   until it succeed or until TIMEOUT_RESENDING_PACKET timeout.
*
* Parameters:
*	   data - The data we want to send - may be data or command.
*	   dataLen - Length of the data
*	   bType - The type of data we want to send - may be either command or data or ack.
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*	   
*
******************************************************************************/
TXRX_ERRORS TxRx_SendPacketWithConfirmation(BYTE *data, WORD dataLen, BLOCK_TYPE bType){

	TXRX_ERRORS status;
	MIWI_TICK t1, t2;

	t1 = MiWi_TickGet();
	while(1){
		status = TxRx_SendPacket(data, dataLen, bType);
		if(status == TXRX_NO_ERROR){
			break;
		}
		t2 = MiWi_TickGet();
		if(MiWi_TickGetDiff(t2, t1) > TIMEOUT_RESENDING_PACKET){	// Waits 3 seconds to get the whole command
			break;
		}
	}
	
	return status;
}

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


TXRX_ERRORS TxRx_SendData(BYTE* samples_block, WORD TX_message_length){

	TXRX_ERRORS status = TxRx_SendPacketWithConfirmation(samples_block, TX_message_length, TXRX_TYPE_DATA);

	if(status != TXRX_NO_ERROR){
		TxRx_printError(status);
	}
	return status;
	
}



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

TXRX_ERRORS m_TxRx_write(BYTE *str){
	
	WORD commandLen = strlen((char*)str);
	TXRX_ERRORS status = TxRx_SendPacketWithConfirmation(str, commandLen, TXRX_TYPE_COMMAND);
	
	if(status != TXRX_NO_ERROR){
		TxRx_printError(status);
	}	
	return status;
}

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

TXRX_ERRORS TxRx_SendCommand(BYTE* command){

	WORD commandLen = strlen((char*)command);
	TXRX_ERRORS status = TxRx_SendPacketWithConfirmation(command, commandLen, TXRX_TYPE_COMMAND);

	if(status != TXRX_NO_ERROR){
		TxRx_printError(status);
	}
	return status;

}



/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_SendAck()
*
* Description:
*      This function sending ack in response to command or data. This function is
*	   It uses that TxRx_SendPacketWithConfimation() function to accomplish this aim.
*	   This functions is used in both the stone and plug.
*	   
*
* Parameters:
*	   None.
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*	   
*
******************************************************************************/


TXRX_ERRORS TxRx_SendAck(){

	TXRX_ERRORS status = TxRx_SendPacket(NULL, 0, TXRX_TYPE_ACK);
	if(status == TXRX_NO_ERROR){
		blockAckInfo.rxLastSeq = blockAckInfo.rxExpectedSeq;
	}
	else{
		TxRx_printError(status);	
	}
	return status;
	
}


/******************************************************************************
******************************************************************************/

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_ReceivePacketHeader()
*
* Description:
*      This function is responsibe for receving the packet header, and getting information
*	   about the packet. 
*
* Parameters:
*	   None.
*
* Return value: 
*	   TXRX_NO_ERROR
*	   TXRX_WRONG_BLOCK_TYPE.
*	   TXRX_WRONG_DATA_SEQ
*	   TXRX_WRONG_ACK_SEQ
*	   TXRX_WRONG_PACKET_LENGTH  
*	   
*
******************************************************************************/

TXRX_ERRORS TxRx_ReceivePacketHeader(){

	WORD i = 0;

	rxBlock.blockHeader.blockType = ((rxMessage.Payload[0])& (0x03));
	if(rxBlock.blockHeader.blockType > 2){
		return TXRX_WRONG_BLOCK_TYPE;
	}


	#if defined (ENABLE_BLOCK_ACK)

		if((rxBlock.blockHeader.blockType == TXRX_TYPE_COMMAND) || (rxBlock.blockHeader.blockType == TXRX_TYPE_DATA)){	// meaning the block is either command or data
				
			BYTE receivedDataSeq = (((rxMessage.Payload[0])>> 2)&0x3F);		
			
			if(receivedDataSeq == ((blockAckInfo.rxLastSeq + 1)%60)){	// if the recived seqence is +1 more than the last ack
				blockAckInfo.rxExpectedSeq = receivedDataSeq;
			}

			else if(receivedDataSeq == blockAckInfo.rxLastSeq ){	//meaning the previous ack was not received.
				blockAckInfo.rxExpectedSeq = receivedDataSeq;
				
			}

			else{	
				#if defined (COMMUNICATION_PLUG) && defined (TXRX_DEBUG_PRINT)
					sprintf((char *)temp_string, "\r\nreceivedData:%d\r\n", receivedDataSeq); 
					m_write(temp_string);
					sprintf((char *)temp_string, "lastReceived:%d\r\n", blockAckInfo.rxLastSeq); 
					m_write(temp_string);
				#endif			
				return TXRX_WRONG_DATA_SEQ;
			}
				
		}
		
	
		if(rxBlock.blockHeader.blockType == TXRX_TYPE_ACK){ // meaning it is ack from the plug to the stone
			
			BYTE receivedAckSeq = (((rxMessage.Payload[0])>> 2)&0x3F);
	
			if(receivedAckSeq == blockAckInfo.txExpectedSeq){	// if the received ack is the same as the last data seq that we sent
				blockAckInfo.txLastSeq = blockAckInfo.txExpectedSeq; 
				return TXRX_NO_ERROR;
			}
			else{
				return TXRX_WRONG_ACK_SEQ; 
			}
		}

	#endif

	rxBlock.blockHeader.blockLen = (WORD)rxMessage.Payload[1];
	rxBlock.blockHeader.blockLen <<= 8;
	
	rxBlock.blockHeader.blockLen &= 0xFF00;
	rxBlock.blockHeader.blockLen += (WORD)((rxMessage.Payload[2])&(0x00FF));

	if(rxBlock.blockHeader.blockLen > 512)
	{
		rxBlock.blockHeader.blockLen = 512;
		return TXRX_WRONG_PACKET_LENGTH;
	}

	rxBlock.handlingParam.blockPos = 0;
	rxBlock.handlingParam.isHeader = FALSE;
	
	
	for(i = 3; i < rxMessage.PayloadSize; i++)
    {
   		rxBlock.blockBuffer[rxBlock.handlingParam.blockPos++] =  rxMessage.Payload[i];
    }
	
	
	return TXRX_NO_ERROR;	
}



/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_ReceiveBuffer()
*
* Description:
*      This function is responsibe for transfering the message in rxMessage struct
*	   to our rxBlock struct. Notice that this function transfers only data in
*	   the buffer of the block, and not the packet header and trailer, which is
*	   done in TxRx_ReceivePacketHeader() and TxRx_ReceivePacketTrailer() functions.
*
* Parameters:
*	   None.
*
* Return value: 
*	   None.
*	   
*
******************************************************************************/

void TxRx_ReceiveBuffer(){
	
	WORD i = 0;
	
    for(i = 0; i < rxMessage.PayloadSize; i++)
    {
   		rxBlock.blockBuffer[rxBlock.handlingParam.blockPos++] =  rxMessage.Payload[i];
    }
}

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_ReceivePacketTrailer()
*
* Description:
*       This function is responsibe for receving the packet trailer, and
*		initilization for receiving the next packet.
*
* Parameters:
*	   None.
*
* Return value: 
*	   TXRX_NO_ERROR
*	   TXRX_RECEIVED_INVALID_TRAILER
*	   
*
******************************************************************************/

TXRX_ERRORS TxRx_ReceivePacketTrailer(){

	TXRX_ERRORS status = TXRX_NO_ERROR;

	WORD i =0;

	for(i = 0; i < TXRX_TRAILER_SIZE; i++)
	{
		rxBlock.blockTrailer[i] = rxBlock.blockBuffer[rxBlock.blockHeader.blockLen+i];	
		if(rxBlock.blockTrailer[i] != TxRx_Trailer[i])
		{
			status = TXRX_RECEIVED_INVALID_TRAILER;
			break;
		}
	}
	
	rxBlock.handlingParam.isHeader = TRUE;	// meaning we received all the packet, and next we are waiting for next block=>waiting for header... 
	rxBlock.blockBuffer[rxBlock.blockHeader.blockLen] = '\0';

	rxBlock.handlingParam.blockPos = 0;
	rxBlock.blockHeader.blockLen = 0;

	return status;

}


/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_ReceiveMessage()
*
* Description:
*      This function is responsibe for receving a message. As mentioned, a packet
*	   consists of several messages, therfore this function is used by 
*	   TxRx_ReceivePacket() to get a message at a time, until the whole packet
*	   is received.
*
* Parameters:
*	   None.
*
* Return value: 
*	   TXRX_NO_ERROR
*	   TXRX_WRONG_BLOCK_TYPE.
*	   TXRX_WRONG_DATA_SEQ
*	   TXRX_WRONG_ACK_SEQ
*	   TXRX_WRONG_PACKET_LENGTH 
*	   TXRX_RECEIVED_INVALID_TRAILER
*	   TXRX_NO_PACKET_RECEIVED
*	   
*
******************************************************************************/

TXRX_ERRORS TxRx_ReceiveMessage(){
	

	/*******************************************************************/
	// Function MiApp_MessageAvailable will return a boolean to indicate 
	// if a message for application layer has been received by the 
	// transceiver. If a message has been received, all information will 
	// be stored in the rxMessage, structure of RECEIVED_MESSAGE.
	/*******************************************************************/
	if( MiApp_MessageAvailable() )
	{		         
		TXRX_ERRORS status;
		if(rxBlock.handlingParam.isHeader == TRUE){	// Meaning it is the beggining of the block
			 status = TxRx_ReceivePacketHeader();
			if(status != TXRX_NO_ERROR){
				MiApp_DiscardMessage();
				return status;
			}
		}

		else{
			TxRx_ReceiveBuffer();
		} 

		if((rxBlock.handlingParam.blockPos) >= (rxBlock.blockHeader.blockLen+TXRX_TRAILER_SIZE)){	// Meaning we got the whole of the block
			status = TxRx_ReceivePacketTrailer();
			if(status != TXRX_NO_ERROR){
				MiApp_DiscardMessage();
				return status;	
			}
		}

		MiApp_DiscardMessage();
		
		
		return TXRX_NO_ERROR; // message received
	}
		
	return TXRX_NO_PACKET_RECEIVED; 
   			     
}


/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_ReceivePacket()
*
* Description:
*      This function is the primary function for receiving a whole packet. The 
*	   function uses TxRx_ReceiveMessage() to get messages, and union all these
*	   messages to get a whole packet.
*
* Parameters:
*	   None.
*
* Return value: 
*	   TXRX_NO_ERROR
*	   TXRX_WRONG_BLOCK_TYPE.
*	   TXRX_WRONG_DATA_SEQ
*	   TXRX_WRONG_ACK_SEQ
*	   TXRX_WRONG_PACKET_LENGTH 
*	   TXRX_RECEIVED_INVALID_TRAILER
*	   TXRX_NO_PACKET_RECEIVED
*	   TXRX_PARTIAL_PACKET_RECEIVED
*	   
*
******************************************************************************/

TXRX_ERRORS TxRx_ReceivePacket(){

	WORD i = 0;
	TXRX_ERRORS status = TXRX_NO_ERROR;

	for(i=0; i<(MAX_BLOCK_SIZE); i++){
		rxBlock.blockBuffer[i] = '\0';
	}

	rxBlock.blockHeader.blockType = 0;

	rxBlock.handlingParam.isHeader = TRUE;
	rxBlock.handlingParam.isTrailer = FALSE;
	rxBlock.handlingParam.blockPos = 0;
	rxBlock.blockHeader.blockLen = 0;

	
	MIWI_TICK t1, t2;

	t1 = MiWi_TickGet();
	while(1){
		 status = TxRx_ReceiveMessage();	// Recive the command packet
		
		if(status != TXRX_NO_PACKET_RECEIVED){
			break;
		}
		t2 = MiWi_TickGet();
		
		if((MiWi_TickGetDiff(t2, t1)) > TIMEOUT_RECEIVING_MESSAGE){	// Waits 5 seconds to get the whole command
			status = TXRX_NO_PACKET_RECEIVED;
			break;
		}
	}
	
	if(status == TXRX_NO_ERROR){ // meaning we received the header of the packet at least
		t1 = MiWi_TickGet();
		while(rxBlock.handlingParam.isHeader == FALSE){	// While we did not get the whole command..
			status = TxRx_ReceiveMessage();	// Continue to try getting the whole command
			if((status != TXRX_NO_ERROR)&&(status != TXRX_NO_PACKET_RECEIVED)){
				break;
			}
			t2 = MiWi_TickGet();
			
			if((MiWi_TickGetDiff(t2, t1)) > TIMEOUT_RECEIVING_MESSAGE){	// Waits 5 seconds to get the whole command
				status = TXRX_PARTIAL_PACKET_RECEIVED;
				break;
			}
		}
	}
	
	return status;
}



/******************************************************************************
YS 30.11
This function was written in order to prevent overflows which cause an
arithmetic trap to occur.

******************************************************************************/
BYTE TxRx_noOverflowADD(BYTE toAdd, BYTE addingTo){
	if(toAdd>=255-addingTo){
		return 255;
	}
	return addingTo+toAdd;
}

/******************************************************************************
YS 22.12
This function resets the ACK sequencers.

******************************************************************************/
void TxRx_Reset_ACK_Sequencers(){//YS 22.12
	txBlock.blockHeader.ackSeq = 1;
	blockAckInfo.txLastSeq =0;
	blockAckInfo.txExpectedSeq = 1;
	blockAckInfo.rxLastSeq = 0;
	blockAckInfo.rxExpectedSeq = 1;
}

/******************************************************************************
******************************************************************************/
void TxRx_printError(TXRX_ERRORS error){

	#if defined (COMMUNICATION_PLUG)
		m_write(TxRx_err_messages[error]);
		m_write("\r\n");
	#endif

	print_string(0, 0, TxRx_err_messages[error]);
}

