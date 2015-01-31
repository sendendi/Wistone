/******************************************************************************
// Summary of TxRx.c:
// The TxRx.c file contains 3 types of functions:
// 1. Function that are common to the stone and to the plug
// 2. Function that are unique to the stone
// 3. Functions that are unique to the PLUG

******************************************************************************/

/************************ INCLUDE **********************************/

#include "wistone_main.h"
#include "WirelessProtocols/MiWi/MiWi.h"
#include "Transceivers/Transceivers.h"
#include "WirelessProtocols/MCHP_API.h"
#include "TxRx.h"
#include "Accelerometer.h"
#include "HardwareProfileTxRx.h"
#include "ConfigApp.h"
#include "misc_c.h"			
#include "command.h"		
#include "eeprom.h" 		
#include "parser.h"			
#include "TimeDelay.h"		
#include "led_buzzer.h"		// YL 31.10 for broadcast and phase counters 

/************************ DEFINE ************************************/

static char *TxRx_err_messages[] = {
	"TxRx - No Error",
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
	"TxRx - NWK Unknown Destination",			// YL 4.8 invalid network address of the destination
	"TxRx - NWK Not Me",						// YL 23.7 
	""
};

// The trailer is constant, and is used for synchronization after each block.
static BYTE TxRx_Trailer[TXRX_TRAILER_SIZE] = {
	0x25, 0x83, 0x33, 0x57
};

// Next is the struct of transmission block. 
// It consists of block header, buffer, trailer and current position in the block.
// The header consists of block type, ackSeq and block length (without header and trailer).
typedef struct {

	struct {
		BYTE blockType;
		BYTE ackSeq;
		BYTE sourceNwkAddress[MY_ADDRESS_LENGTH]; 			// for: MY_ADDRESS_LENGTH = 2:
															//		sourceNwkAddress[0] is EUI_0 from the EEPROM
															//		sourceNwkAddress[1] is EUI_1 (constant)												
		BYTE destinationNwkAddress[MY_ADDRESS_LENGTH];		// for: MY_ADDRESS_LENGTH = 2:
															//		destinationNwkAddress[0] is a parameter EUI_0 
															//		destinationNwkAddress[1] is a parameter EUI_1 
		WORD blockLen;
	} blockHeader;
	
	BYTE blockBuffer[MAX_BLOCK_SIZE + 10]; 
	BYTE blockTrailer[TXRX_TRAILER_SIZE];
	WORD blockPos;
} TX_BLOCK_BUFFER;

// Next is the struct of receiving block.
// It consists of block header, buffer, trailer and some other information 
// of the block for handling it (the last one is not a part of the block itself).
// The header consists of block type, ackSeq and block length (without header and trailer).
typedef struct {

	struct {
		BYTE blockType;	
		WORD blockLen;
	} blockHeader;

	BYTE blockBuffer[MAX_BLOCK_SIZE + 10];
	BYTE blockTrailer[TXRX_TRAILER_SIZE];

	struct {
		WORD blockPos;
		BOOL isHeader;	
		BOOL isTrailer;
	} handlingParam;
} RX_BLOCK_BUFFER;

// Next struct consists of acknowledgements information. 
// There are acknowledgement sequences for transmitting data and for receiving data.
// The ..LastSeq describes the sequence of the last successful transmitting/receiving,
// and the ..ExpectedSeq describes the ack that we expect to get.
typedef struct {
	BYTE txLastSeq;
	BYTE txExpectedSeq;
	BYTE rxLastSeq;
	BYTE rxExpectedSeq;
} BLOCK_ACK_INFO;

/************************ VARIABLES ********************************/
TX_BLOCK_BUFFER txBlock;
RX_BLOCK_BUFFER rxBlock;
#if defined WISDOM_STONE
	BLOCK_ACK_INFO  blockAckInfo;
#elif defined COMMUNICATION_PLUG
	BLOCK_ACK_INFO  blockAckInfo[MAX_NWK_SIZE];  			// the indices in the array match EUI[0] of the stone: 
															// e.g - ack info of the stone with EUI[0] = 2 is in blockAckInfo[2]
#endif // WISDOM_STONE

// Next global variables are only for receiving commands during transmission of blocks.
// Notice that when we use acks, and the stone is waiting for an ack, it can receive
// in this time a command (for instance "app stop", therefore we should separate between
// these by the next 2 variables:
 
#if defined WISDOM_STONE
	//extern BYTE g_is_cmd_received;			// indicates that command was received during a transmission. 
#endif

#if defined COMMUNICATION_PLUG
	// indicates for the plug that "app stop" was sent to the stone, and therefore the next block that will be received, will not be printed.
	BOOL isAppStop;							// YL indicates that the plug received "app stop" command, 
											// and therefore it should stop receiving data blocks from the stone
	BOOL isStopped[MAX_NWK_SIZE];			// the indices in the array match EUI[0] of the stone: 
											// e.g - ack info of the stone with EUI[0] = 2 is in blockAckInfo[2]
#endif // COMMUNICATION_PLUG

#if defined ENABLE_RETRANSMISSION
	BYTE blockTryTxCounter = 0;
	extern BYTE messageRetryCounter;
#endif

BYTE finalDestinationNwkAddress[MY_ADDRESS_LENGTH]; 		// currently max possible MAX_NWK_SIZE is 8 components in the network, so 1 byte is enough for the network address of the destination
BOOL isBroadcast;											// indicates that we received broadcast command

#if defined COMMUNICATION_PLUG && defined ENABLE_TXRX_ACK 	// for indices of blockAckInfo array
	BYTE txToEUI0;			// EUI_0 of the destination when the plug is the transmitter (= finalDestinationNwkAddress[0])
	BYTE rxFromEUI0;		// EUI_0 of the source when the plug is the receiver (= finalDestinationNwkAddress[0])
#endif // COMMUNICATION_PLUG, ENABLE_TXRX_ACK

#if defined COMMUNICATION_PLUG
	BOOL isNetworkMember[MAX_NWK_SIZE]; 	// array of indications whether the stone is or isn't network member; the "0" index belongs to the plug, all other indices reflect the EUI_0 of the stone  
	BOOL isCoordinator[MAX_NWK_SIZE];		// array of indications whether the stone is or isn't network member; the "0" index belongs to the plug, all other indices reflect the EUI_0 of the stone
	BYTE parentDeviceEUI0[MAX_NWK_SIZE];	// array of EUI_0 addresses of stone-network-members' parents; the "0" index belongs to the plug, all other indices reflect the EUI_0 of the stone
	BYTE currentNwkSize;	
#elif defined WISDOM_STONE
	BOOL isCoordinator;
	BYTE parentDeviceEUI0;
#endif

#define JOIN_SEND				0b01010101					// 0b0101,0101 for join-info correspondence (join-info messages are sent only at init stage, therefore no ambiguity is expected)
#define JOIN_RECEIVE			(JOIN_SEND << 1)			// 0b1010,1010 for join-info correspondence
#define JOIN_COORDINATOR_MASK	0b10000000					// MSB in join-info-byte indicates whether the device is a coordinator
#define JOIN_PARENT_MASK		~JOIN_COORDINATOR_MASK		// info-byte, excluding the MSB, keeps parent-device EUI_0

// YL 31.10 ...
BOOL isTxRxTypeCommand;	// for g_phase_counter
// ... YL 31.10

// YL 11.1 ...
WORD_VAL g_phase_counter_start;
WORD_VAL g_phase_counter_stop;
WORD_VAL g_broadcast_counter_start;
WORD_VAL g_broadcast_counter_stop;
// ... YL 11.1

/***************** FUNCTION DECLARATIONS ****************************/

// Overall functions:
TXRX_ERRORS TxRx_WistoneHandler();
TXRX_ERRORS TxRx_PlugHandler();

// Tx Functions:
TXRX_ERRORS TxRx_TransmitBuffer();
TXRX_ERRORS TxRx_SendPacket(BYTE *data, WORD dataLen, BLOCK_TYPE bType);

#if defined WISDOM_STONE
	TXRX_ERRORS TxRx_SendData(BYTE* samples_block, WORD TX_message_length);	
	TXRX_ERRORS m_TxRx_write(BYTE *str);
#elif defined COMMUNICATION_PLUG
	TXRX_ERRORS TxRx_SendCommand(BYTE* command);
#endif //WISDOM_STONE

#if defined ENABLE_TXRX_ACK // YL 25.12 added #ifdef
TXRX_ERRORS TxRx_SendAck();
#endif
BYTE TxRx_ByteAdd(BYTE toAdd, BYTE addingTo); // YL 12.1 was: TxRx_noOverflowADD; renamed to TxRx_ByteAdd

// Rx Functions:
TXRX_ERRORS TxRx_ReceiveHeaderPacket();
void TxRx_ReceiveBuffer();
TXRX_ERRORS TxRx_ReceiveMessage();
TXRX_ERRORS TxRx_ReceivePacket();

#if defined COMMUNICATION_PLUG
void TxRx_ReceiveJoinInfo(void);
#elif defined WISDOM_STONE
void TxRx_SendJoinInfo(void);
#endif

/******************************************************************************
* Function:
*		void MRFInit()
*
* Description:
*      This is the primary user interface function to initialize the MRF.
*	   This function initializes the MRF ports, the SPI interface used by
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
void MRFInit() {

#if defined(MRF24J40) || defined(MRF49XA)
    PHY_CS_TRIS = 0;
    PHY_CS = 1;
    //PHY_RESETn_TRIS = 0;
    //PHY_RESETn = 1;
#else
	#error "Invalid MRF"
#endif //MRF24J40
	
	RF_INT_TRIS = 1;
	
	SDI_TRIS = 1;
	SDO_TRIS = 0;
	SCK_TRIS = 0;
	SPI_SDO  = 0;        
	SPI_SCK  = 0;             
	
#if defined(MRF49XA)
    nFSEL_TRIS = 0;
    FINT_TRIS  = 1;	    
    nFSEL      = 1; 
   
#elif defined(MRF24J40)
    PHY_WAKE_TRIS = 0;
    PHY_WAKE = 1;	
#endif //MRF49XA
	
#if defined(HARDWARE_SPI)
     SPI2CON1 = 0b0000000100111010;
     SPI2STAT = 0x8000;	
#else
	#error "Not defined Hardware SPI"
#endif //HARDWARE_SPI
	
	INTCON2bits.INT1EP = 1;
	
#if defined(ENABLE_NVM)
    EE_nCS_TRIS = 0;
    EE_nCS = 1;
#endif //ENABLE_NVM
	
	RFIF = 0;
	if (RF_INT_PIN == 0) {
	    RFIF = 1;
	}       
    RFIE = 1; // YL 11.5(BM) - added to be like in the RFD example
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
*	   BOOL justResetNetwork - to indicate if we only want to reset the networking stack, and not data counters (for network faults)
*
* Return value: 
*	   None
*
******************************************************************************/ 
void TxRx_Init(BOOL justResetNetwork) {

	MIWI_TICK 	t1, t2;	// to limit TxRx_Init time for the plug\stone
	BYTE		i;

	// initialize the MRF ports
	MRFInit();
	
	// read from EEPROM the byte that is used as the LSByte of the EUI:		
	myLongAddress[0] = (BYTE)(0xFF & eeprom_read_byte(EUI_0_ADDRESS)); 

	MiApp_ProtocolInit(FALSE);  
	
	if (!justResetNetwork) {
		#if defined ENABLE_TXRX_ACK // YL 25.12 added #ifdef
		TxRx_Reset_ACK_Sequencers(); //YS 22.12 init required for the acks	// YL 29.7 AY called TxRx_Reset_ACK_Sequencers in both - plug and stone if(!justResetNetwork), and in addition - at the end of TxRx_Connect, in plug only, and without any condition (whereas the stone started the network); do we need that additional call? 
		#endif // ENABLE_TXRX_ACK
		
		for (i = 0; i < TXRX_TRAILER_SIZE; i++) {
			txBlock.blockTrailer[i] = TxRx_Trailer[i];	// init of constant trailer that is used for synchronization of the block
		}
		
		#if defined COMMUNICATION_PLUG
			for (i = 0; i < MAX_NWK_SIZE; i++) {			
				isStopped[i] = FALSE;
				isNetworkMember[i] = FALSE;
				isCoordinator[i] = FALSE;
				parentDeviceEUI0[i] = 0xFF;
				currentNwkSize = INIT_NWK_SIZE;				
			}
		#elif defined WISDOM_STONE
			isCoordinator = FALSE;
			parentDeviceEUI0 = 0xFF;
		#endif
	}
	
	MiApp_ConnectionMode(ENABLE_ALL_CONN);					                                              
	
	if (myLongAddress[0] == NWK_STARTER_ADDR_EUI0) {
		// the stone with NWK_STARTER_ADDR_EUI0 is the only PAN coordinator.
		// it is the only one that starts the network, and then accepts others that join it
		
		t1 = MiWi_TickGet();
		while (1) {
			MiApp_StartConnection(START_CONN_ENERGY_SCN, 10, 0xFFFFFFFF); 		// YL 25.5 to select the most quiet channel: START_CONN_ENERGY_SCN instead of START_CONN_DIRECT
			t2 = MiWi_TickGet();
			if (MiWi_TickGetDiff(t2, t1) > TIMEOUT_NWK_STARTING) {		
				break;
			}
		}	
	}
	else {
		// the stone is a regular (non-PAN) coordinator.
		// it searches for networks and establish connection to one of them
		
		BOOL	joinedNetwork = FALSE;
		BYTE	totalActiveScanResponses;
		
		t1 = MiWi_TickGet();		
		while (!joinedNetwork) { 
			totalActiveScanResponses = MiApp_SearchConnection(10, 0xFFFFFFFF);
			i = 0;
			while ((!joinedNetwork) && (i < totalActiveScanResponses)) {			
				if (MiApp_EstablishConnection(i, CONN_MODE_DIRECT) != 0xFF) { 
					joinedNetwork = TRUE;  										// a connection has been established - WISDOM_STONE completed TxRx_Init successfully
					#if defined COMMUNICATION_PLUG
						// update plug parameters:
						isNetworkMember[0] = TRUE;
						#if defined NWK_ROLE_COORDINATOR
							isCoordinator[0] = TRUE;	
						#endif							
						parentDeviceEUI0[0] = ConnectionTable[myParent].Address[0];			
						// update constant network-starter parameters:
						isNetworkMember[NWK_STARTER_ADDR_EUI0] = TRUE;	
						isCoordinator[NWK_STARTER_ADDR_EUI0] = TRUE;
						parentDeviceEUI0[NWK_STARTER_ADDR_EUI0] = 0xFF;	// the parent of the network-starter is always 0xFF											
					#elif defined WISDOM_STONE
						// update stone parameters:
						#if defined NWK_ROLE_COORDINATOR
							isCoordinator = TRUE;
						#endif
						parentDeviceEUI0 = ConnectionTable[myParent].Address[0];
						// 11.2 TxRx_SendJoinInfo();
					#endif
					break;
				}
				i++;  															// try to establish connection with next active scan response
			}
				t2 = MiWi_TickGet();  									
				if (MiWi_TickGetDiff(t2, t1) > TIMEOUT_NWK_JOINING) {	
					break;
				}
		}
		
		if (!joinedNetwork) {  				// no connections has been found 
			g_sleep_request = TRUE;			// go to sleep till next wakeup time
		}
	}	

	play_buzzer(1);	// the stone/plug finished TxRx_Init
	
	// YL 25.5 added AY...
	if (justResetNetwork) { // YS 17.11
		while (!ConnectionTable[0].status.bits.isValid); 	// make sure the other side reconnected
	}
	// ...YL 25.5	 
}

#if defined COMMUNICATION_PLUG
/******************************************************************************
* Function:
*		void TxRx_ReceiveJoinInfo(void)
* the plug confirms the receiving of join-info with an ack message of 5 bytes in total:
* - 0: JOIN_RECEIVE (id)
* - 1, 2: long address of the source
* - 3, 4: long address of the destination
*******************************************************************************/
void TxRx_ReceiveJoinInfo(void) {
	
	MIWI_TICK t1, t2;
	BYTE i;
	BYTE sourceNwkAddress[MY_ADDRESS_LENGTH] = {0};
	BYTE sourceParent = 0;
	BOOL sourceIsCoordinator = FALSE;
	
	if (MiApp_MessageAvailable() == FALSE ||		// nothing to do
		rxMessage.Payload[0] != JOIN_SEND ||		// unexpected message type
		rxMessage.Payload[3] != myLongAddress[0]) {	// unexpected destination eui0 (should not happen because MiWi layer discards messages with inappropriate destination addresses)		
		return;
	} 
	// read the received join-info:
	sourceNwkAddress[0] = rxMessage.Payload[1];
	sourceNwkAddress[1] = rxMessage.Payload[2];	
	sourceParent = (rxMessage.Payload[5] == 0xFF) ? 0xFF : (rxMessage.Payload[5] & JOIN_PARENT_MASK); // the only device with parentEUI0 = 0xFF is the PAN COORDINATOR; for the rest devices parentEUI0 isn't bigger than MAX_NWK_SIZE ( << 100)
	sourceIsCoordinator = ((rxMessage.Payload[5] & JOIN_COORDINATOR_MASK) ? TRUE : FALSE);	
	// update the network data structures:
	parentDeviceEUI0[(sourceNwkAddress[0])] = sourceParent;
	isCoordinator[(sourceNwkAddress[0])] = sourceIsCoordinator;
	isNetworkMember[(sourceNwkAddress[0])] = TRUE;
	// send an acknowledgement:
	MiApp_FlushTx();		
	MiApp_WriteData(JOIN_RECEIVE);
	// the source:	
	for (i = 0; i < MY_ADDRESS_LENGTH; i++) {
		MiApp_WriteData(myLongAddress[i]);		
	}
	// the destination:	
	for (i = 0; i < MY_ADDRESS_LENGTH; i++) {
		MiApp_WriteData(sourceNwkAddress[i]);		// the source of the received join-info becomes the destination of an acknowledgement
	}
	t1 = MiWi_TickGet();
	while (MiApp_UnicastAddress(sourceNwkAddress, TRUE, FALSE) == FALSE) {
		t2 = MiWi_TickGet();
		if (MiWi_TickGetDiff(t2, t1) > TIMEOUT_RESENDING_PACKET) {				
			// YL 7.11 TxRx_PrintError(TXRX_UNABLE_SEND_PACKET);
			return;
		}	
	}
	// YL 24.10 ...
	//YL 3.11 currentNwkSize++; // TODO - check the assumption - each stone tries joining only once
	// ... YL 24.10	
}

#elif defined WISDOM_STONE
/******************************************************************************
* Function:
*		void TxRx_SendJoinInfo(void)
* the stone sends join-info to the plug - a message of 6 bytes in total:
* - 0: JOIN_SEND (id)
* - 1, 2: long address of the source
* - 3, 4: long address of the destination
* - 5: join-info (coordinator + parent)
*******************************************************************************/
void TxRx_SendJoinInfo(void) {

	MIWI_TICK t1, t2;
	BYTE i;
	BYTE ack = 0, info = 0;
	BYTE destinationNwkAddress[MY_ADDRESS_LENGTH];

	MIWI_TICK t3, t4;
	t3 = MiWi_TickGet();						
	while(1) {
		MiWiTasks();				
		MiApp_FlushTx();	
		MiApp_WriteData(JOIN_SEND);
		// the source:	
		for (i = 0; i < MY_ADDRESS_LENGTH; i++) {
			MiApp_WriteData(myLongAddress[i]);		
		}
		// the destination:	
		destinationNwkAddress[0] = PLUG_NWK_ADDR_EUI0;
		destinationNwkAddress[1] = PLUG_NWK_ADDR_EUI1;	
		for (i = 0; i < MY_ADDRESS_LENGTH; i++) {
			MiApp_WriteData(destinationNwkAddress[i]);		
		}
		// join-info:  	
		info = parentDeviceEUI0; 
		if (isCoordinator == TRUE) {
			info |= JOIN_COORDINATOR_MASK; 
		}
		MiApp_WriteData(info);
		// send join-info:
		t1 = MiWi_TickGet();
		while (MiApp_UnicastAddress(destinationNwkAddress, TRUE, FALSE) == FALSE) {
			t2 = MiWi_TickGet();
			if (MiWi_TickGetDiff(t2, t1) > TIMEOUT_RESENDING_PACKET) {	
				TxRx_PrintError(TXRX_UNABLE_SEND_PACKET);
				goto tick;
			}
		}
		while (1) {
			if (MiApp_MessageAvailable() == TRUE) {
				ack = rxMessage.Payload[0];
				if ((ack == JOIN_RECEIVE) && (myLongAddress[0] == rxMessage.Payload[3])) { // proper message type and destination eui0 (extra check because MiWi layer should discard messages with inappropriate destination addresses)			
					return;
				}	
			}
			t2 = MiWi_TickGet();
			if (MiWi_TickGetDiff(t2, t1) > TIMEOUT_RESENDING_PACKET) {	
				TxRx_PrintError(TXRX_UNABLE_SEND_PACKET);
				break;
			}			
		}
		
tick:	t4 = MiWi_TickGet();
		if (MiWi_TickGetDiff(t4, t3) > TIMEOUT_NWK_JOINING) {	
			return;
		}	
	}		
}
#endif

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
TXRX_ERRORS TxRx_PeriodTasks() {

	TXRX_ERRORS status;
	MIWI_TICK t1, t2; 
	// check if there is available message
	if (MiApp_MessageAvailable()) {	
		t1 = MiWi_TickGet();
		while (1) {
			status = TxRx_ReceivePacket();
			if (status == TXRX_NO_ERROR) {		
				break;
			}
			t2 = MiWi_TickGet(); 
			if (MiWi_TickGetDiff(t2, t1) > TIMEOUT_RETRYING_RECEIVING_PACKET) {
				status = TXRX_NO_PACKET_RECEIVED;
			}	
		}

		if (status != TXRX_NO_ERROR) {
			return status;	
		}

		#if defined WISDOM_STONE
			status = TxRx_WistoneHandler();
		#elif defined COMMUNICATION_PLUG
			status = TxRx_PlugHandler();
		#endif //WISDOM_STONE
	}

	return status;
}

#if defined WISDOM_STONE

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_WistoneHandler()
*
* Description:
*      This is an interface function to perform period tasks in the
*	   stone.
*	   This function is called in the TxRx_PeriodTasks. The function checks
*	   what is the type of the received packet, and processes it.
*
* Parameters:
*	   None
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*
******************************************************************************/
TXRX_ERRORS TxRx_WistoneHandler(){	
	
	if (rxBlock.blockHeader.blockType == TXRX_TYPE_DATA) {   	// it is impossible that the block type here is data, since this code is running in the stone		
		return TXRX_RECEIVED_INVALID_PACKET;
	}
	
	#if defined ENABLE_TXRX_ACK
		if (rxBlock.blockHeader.blockType == TXRX_TYPE_ACK) { 	// we received an ack, nothing to do	
			return TXRX_NO_ERROR;
		}
		else {													// it is a command, so we need to send ACK			
			TXRX_ERRORS status = TxRx_SendAck();
			if (status != TXRX_NO_ERROR) {				
				return status;
			}
		}
	#endif //ENABLE_TXRX_ACK
	
	// copy the input command to g_in_msg array to check that RX_block_buffer is not bigger than 100
	strcpy(g_in_msg, (char*)rxBlock.blockBuffer);	
	return TXRX_NO_ERROR;
}
#endif //WISDOM_STONE

#if defined COMMUNICATION_PLUG

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_PlugHandler()
*
* Description:
*      This is an interface function to perform period tasks in the
*	   plug.
*	   This function is called in the TxRx_PeriodTasks. The function checks
*	   what is the type of the received packet, and processing it.
*
* Parameters:
*	   None
*
* Return value: 
*	   Any parameter at the TXRX_ERRORS enum.
*
******************************************************************************/
TXRX_ERRORS TxRx_PlugHandler() {

	BYTE isBlockNeedToBePrinted = 1;
	
	#if defined ENABLE_TXRX_ACK
		TXRX_ERRORS status;
		if ((rxBlock.blockHeader.blockType == TXRX_TYPE_COMMAND) ||
			(rxBlock.blockHeader.blockType == TXRX_TYPE_DATA)) {				// need to send ack for the command/data <- can the plug receive a command?
			if (blockAckInfo[rxFromEUI0].rxLastSeq ==
				blockAckInfo[rxFromEUI0].rxExpectedSeq) {	// we received again a block that we handled before, since the ack was unsuccessful			
				isBlockNeedToBePrinted = 0;
			}
			status = TxRx_SendAck();
			if (status != TXRX_NO_ERROR) {			
				return status;
			}
		}
		else { // it is ack	
			return TXRX_NO_ERROR;
		}
	#endif // ENABLE_TXRX_ACK
	
	if (isBlockNeedToBePrinted == 0) {									// if we received a block that was handled before and the ack was not received by the stone		
		return TXRX_NO_ERROR;
	}
	if (rxBlock.blockHeader.blockType == TXRX_TYPE_DATA) {				// if we received data, print it using b_write
		if (isStopped[(finalDestinationNwkAddress[0])] == FALSE) {		// do not print the last block that was received
			b_write(rxBlock.blockBuffer, MAX_BLOCK_SIZE);
 		}	
	}			
	else if (rxBlock.blockHeader.blockType == TXRX_TYPE_COMMAND) {	 	// if we received command, print it using m_write	
		m_write((char*)rxBlock.blockBuffer);
	}	

	return TXRX_NO_ERROR;
}
#endif //COMMUNICATION_PLUG

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_TransmitBuffer()
*
* Description:
*      This function performs the transmission to the other device. It transmits
*	   the header, the buffer itself and the trailer. All this data should be
*	   filled in the txBlock , and it is done in TxRx_SendPacket function.
*	   YL - TxRx_TransmitBuffer calls MiApp_WriteData that copies the contents 
*	   of txBlock (blockHeader, blockBuffer, blockTrailer) into TxBuffer and unicasts it
*
* Parameters:
*	   None
*
* Return value: 
*	   TXRX_NO_ERROR
*	   TXRX_UNABLE_SEND_PACKET
*
******************************************************************************/
TXRX_ERRORS TxRx_TransmitBuffer() {
		
	MIWI_TICK t1;	
	WORD trailerPos = 0;
	txBlock.blockPos = 0;
	BYTE i = 0;								// to write "MY_ADDRESS_LENGTH" bytes of sourceNwkAddress 
	MIWI_TICK t2;							// to enable timeout on unicast trials
	TXRX_ERRORS status = TXRX_NO_ERROR;		// to inform on timeout
	
	// YL 31.10 ...
	isTxRxTypeCommand = FALSE;
	// ... YL 31.10
	
	MiApp_FlushTx(); 
	BYTE blockInf = (txBlock.blockHeader.blockType) & TXRX_TYPE_MASK;
	blockInf |= ((txBlock.blockHeader.ackSeq << 2) & TXRX_ACK_MASK); 			// YL blockInf = aaaa, aatt (6 ack bits + 2 type bits)
	MiApp_WriteData(blockInf);
			
	for (i = 0; i < MY_ADDRESS_LENGTH; i++) {
		MiApp_WriteData(txBlock.blockHeader.sourceNwkAddress[i]);				// for: MY_ADDRESS_LENGTH = 2:
																				//		sourceNwkAddress[0] is EUI_0 from the EEPROM
																				//		sourceNwkAddress[1] is EUI_1 (constant)
																				
	}
			
	for (i = 0; i < MY_ADDRESS_LENGTH; i++) {
		MiApp_WriteData(txBlock.blockHeader.destinationNwkAddress[i]);			// for: MY_ADDRESS_LENGTH = 2:
																				//		destinationNwkAddress[0] is a parameter EUI_0 
																				//		destinationNwkAddress[1] is a parameter EUI_1 																				
	}
	
	if (txBlock.blockHeader.blockLen > MAX_BLOCK_SIZE) {	
		txBlock.blockHeader.blockLen = MAX_BLOCK_SIZE;
	}
	
	BYTE blockLen = 0;
	for (i = MSG_LEN_LENGTH; i > 0; i--) {
		blockLen = (BYTE)(txBlock.blockHeader.blockLen >> (8 * (i - 1))) ;		// the upper byte is first
		MiApp_WriteData(blockLen);
	}
		
	WORD message_counter = MSG_INF_LENGTH + 2 * MY_ADDRESS_LENGTH + MSG_LEN_LENGTH;
	// YL 25.12 ... added phase
	if (txBlock.blockHeader.blockType == TXRX_TYPE_COMMAND) {
		isTxRxTypeCommand = TRUE;
		// YL 12.1 ...
		g_phase_counter_stop.Val = g_phase_counter.Val - g_phase_counter_start.Val;
		if (g_phase_counter_stop.Val < 0) {
			g_phase_counter_stop.Val += 0xFFFF;	// overflow
		}
		#if defined DEBUG_PRINT
			m_write_debug("PHStop: ");
			m_write_debug(int_to_str(g_phase_counter_stop.Val));
			m_write_debug("\r\n");
		#endif			
		#if defined COMMUNICATION_PLUG
			g_phase_counter_stop.Val += g_broadcast_counter_stop.Val; // TODO overflow?
			#if defined DEBUG_PRINT
				m_write_debug("PHStop + BCStop: ");
				m_write_debug(int_to_str(g_phase_counter_stop.Val));
				m_write_debug("\r\n");
			#endif		
		#endif // COMMUNICATION_PLUG
		MiApp_WriteData(g_phase_counter_stop.byte.HB);
		MiApp_WriteData(g_phase_counter_stop.byte.LB);
		// ... YL 12.1		
		message_counter += MSG_PHS_LENGTH;	
		// YL 11.1 ... added so only the delta would be added in TxPacket (make sure it works in application-level retransmissions too)
		g_phase_counter_start.Val = g_phase_counter.Val;
		// ... YL 11.1
		#if defined DEBUG_PRINT // YL 24.12 TODO add the following warning to the code
			if (txBlock.blockHeader.blockLen > TX_BUFFER_SIZE - (MIWI_HEADER_LEN + TXRX_TRAILER_SIZE)) {	// the length of TXRX_TYPE_COMMAND is given by strlen of the command string
				m_write_debug("\r\n the command is too long - erroneous phase-counter update!! \r\n");
			}
		#endif
	}
	// ... YL 25.12
	
	while (txBlock.blockPos < (txBlock.blockHeader.blockLen + TXRX_TRAILER_SIZE)) {	// YL blockPos counts the bytes of the data (blockBuffer) and of blockTrailer
		if (message_counter == 0) {
			MiApp_FlushTx();														// YL resets the pointer (TxData) to PAYLOAD_START - the 12th byte of TxBuffer
		}
		if (txBlock.blockPos < txBlock.blockHeader.blockLen) {						// meaning we are writing the buffer itself.
			MiApp_WriteData(txBlock.blockBuffer[txBlock.blockPos++]);
		}
		else {																		// meaning we are writing the trailer.
			MiApp_WriteData(txBlock.blockTrailer[trailerPos++]);
			txBlock.blockPos++;
		}
		message_counter++;
		// YL 14.12 ... the transceiver discards the message if it's length exceeds TX_BUFFER_SIZE
		// was: if (message_counter == TX_BUFFER_SIZE) {							 	
		if (message_counter == TX_BUFFER_SIZE - MIWI_HEADER_LEN) {
		// ... YL 14.12
			message_counter = 0;														
			t1 = MiWi_TickGet();
			while (MiApp_UnicastAddress(finalDestinationNwkAddress, TRUE, FALSE) == FALSE) {
				#if defined ENABLE_RETRANSMISSION
					blockTryTxCounter = TxRx_ByteAdd(RETRANSMISSION_TIMES, blockTryTxCounter);
				#endif
				t2 = MiWi_TickGet();
				if (MiWi_TickGetDiff(t2, t1) > TIMEOUT_RESENDING_PACKET) {	
					status = TXRX_UNABLE_SEND_PACKET;					
					break;
				}		
			}

			#if defined ENABLE_RETRANSMISSION
				blockTryTxCounter = TxRx_ByteAdd(messageRetryCounter, blockTryTxCounter);	// add the number of transmission needed in the lower level.
			#endif
		}  
		if (status == TXRX_UNABLE_SEND_PACKET) {	
			break;
		}
	}
	if (message_counter > 0) {		
		t1 = MiWi_TickGet();		
		while (MiApp_UnicastAddress(finalDestinationNwkAddress, TRUE, FALSE) == FALSE) {	
			#if defined ENABLE_RETRANSMISSION
				blockTryTxCounter = TxRx_ByteAdd(RETRANSMISSION_TIMES, blockTryTxCounter);	
			#endif
		
			t2 = MiWi_TickGet();
			if (MiWi_TickGetDiff(t2, t1) > TIMEOUT_RESENDING_PACKET) {	
				status = TXRX_UNABLE_SEND_PACKET;			
				break;
			}		
		}
		#if defined ENABLE_RETRANSMISSION
			blockTryTxCounter = TxRx_ByteAdd(messageRetryCounter, blockTryTxCounter);			// add the number of transmission needed in the lower level.
		#endif
	}   

	return status;	// to return TXRX_UNABLE_SEND_PACKET if needed	
}

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_SendPacket(BYTE *data, WORD dataLen, BLOCK_TYPE bType)
*
* Description:
*      This function sends a packet to the other device. This packet can be 
*	   either command or data or ack. The function fills the txBlock with the 
*	   data. Then it uses the TxRx_TransmitBuffer() function to send this
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

//We assume that this function may be interrupted at any stage. // YL TODO - reset

TXRX_ERRORS TxRx_SendPacket(BYTE *data, WORD dataLen, BLOCK_TYPE bType){
	
	TXRX_ERRORS status;
	txBlock.blockHeader.blockType = bType;										// getting the block type we want to send
	BYTE i = 0;	// to write "MY_ADDRESS_LENGTH" bytes into sourceNwkAddress  
		
	#if defined ENABLE_TXRX_ACK
		if (bType == TXRX_TYPE_ACK) {											// if it is ack
			// fill txBlock with ack info:
			#if defined WISDOM_STONE
				txBlock.blockHeader.ackSeq = blockAckInfo.rxExpectedSeq; 		 			
			#elif defined COMMUNICATION_PLUG
				txToEUI0 = finalDestinationNwkAddress[0];
				txBlock.blockHeader.ackSeq = blockAckInfo[txToEUI0].rxExpectedSeq;
			#endif // WISDOM_STONE
		
			for (i = 0; i < MY_ADDRESS_LENGTH; i++) {
				txBlock.blockHeader.sourceNwkAddress[i] = myLongAddress[i];		// read "MY_ADDRESS_LENGTH" bytes into sourceNwkAddress	
			}			
			for (i = 0; i < MY_ADDRESS_LENGTH; i++) {
				txBlock.blockHeader.destinationNwkAddress[i] = finalDestinationNwkAddress[i];	// read "MY_ADDRESS_LENGTH" bytes into destinationNwkAddress
			}		
			txBlock.blockHeader.blockLen = 0;			
			status = TxRx_TransmitBuffer();
			return status;
		}
		// else - it is command or data
		#if defined WISDOM_STONE
			blockAckInfo.txExpectedSeq = (blockAckInfo.txLastSeq + 1) % MAX_ACK_LENGTH;
			txBlock.blockHeader.ackSeq =  blockAckInfo.txExpectedSeq;			
		#elif defined COMMUNICATION_PLUG
			txToEUI0 = finalDestinationNwkAddress[0];
			blockAckInfo[txToEUI0].txExpectedSeq = (blockAckInfo[txToEUI0].txLastSeq + 1) % MAX_ACK_LENGTH;
			txBlock.blockHeader.ackSeq =  blockAckInfo[txToEUI0].txExpectedSeq; 								// YL ackSeq gets new (incremented) num
		#endif // WISDOM_STONE
	#endif // ENABLE_TXRX_ACK
	
	for (i = 0; i < MY_ADDRESS_LENGTH; i++) {
		txBlock.blockHeader.sourceNwkAddress[i] = myLongAddress[i];				// read "MY_ADDRESS_LENGTH" bytes into sourceNwkAddress
	}	
	for (i = 0; i < MY_ADDRESS_LENGTH; i++) {
		txBlock.blockHeader.destinationNwkAddress[i] = finalDestinationNwkAddress[i];	// read "MY_ADDRESS_LENGTH" bytes into destinationNwkAddress	
	}	
	txBlock.blockHeader.blockLen = dataLen;										// YL TxRx_SendPacket fills txBlock with the len of the cmd/data, and the data itself (MAX_BLOCK_SIZE = 512 bytes max in txBlock.blockBuffer)
	if (txBlock.blockHeader.blockLen > MAX_BLOCK_SIZE) { 						
		txBlock.blockHeader.blockLen = MAX_BLOCK_SIZE;
	}
	WORD j;
	for (j = 0; j < txBlock.blockHeader.blockLen; j++) {
		txBlock.blockBuffer[j] = data[j]; 										
	}
	
	// initiate transmission (to USB/Wireless)
	status = TxRx_TransmitBuffer();												// YL TxRx_TransmitBuffer sends the cmd/data
	
	#if defined ENABLE_TXRX_ACK
		if (status != TXRX_NO_ERROR) {			
			return status;
	 	}
		status = TxRx_ReceivePacket();											// waiting for ack of the packet to arrive (the ack is for command or for data)
		if (status != TXRX_NO_ERROR) {	
			return status;
		}	
		if (rxBlock.blockHeader.blockType == TXRX_TYPE_ACK) { 					// it is ack
			return TXRX_NO_ERROR;			
		}
		#if defined WISDOM_STONE
			else if (rxBlock.blockHeader.blockType == TXRX_TYPE_COMMAND) { 		// if we received app stop in the middle of transmission
				strcpy(g_in_msg, (char*)rxBlock.blockBuffer);					// to check that RX_block_buffer is not bigger than 100 // YL 14.4 added casting to avoid signedness warning
				TXRX_ERRORS status = TxRx_SendAck();							// send ack to the command
				if (status != TXRX_NO_ERROR) {			
					return status;
				}
				g_is_cmd_received = 1;											// indicates that a command was received during this period - inform the upper level by setting this variable				
				status = TxRx_ReceivePacket();									// waiting for ack of the original packet to arrive (the ack is for command or for data)
				if (status != TXRX_NO_ERROR) {				
					return status;
				}
				if (rxBlock.blockHeader.blockType == TXRX_TYPE_ACK) { 			// it is ack
					return TXRX_NO_ERROR;			
				}	
			}
		#endif //WISDOM_STONE
		return TXRX_RECEIVED_UNKNOWN_PACKET;
	#else
		return status;		
	#endif //ENABLE_TXRX_ACK	
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
TXRX_ERRORS TxRx_SendPacketWithConfirmation(BYTE *data, WORD dataLen, BLOCK_TYPE bType) {

	TXRX_ERRORS status;
	MIWI_TICK t1, t2;

	t1 = MiWi_TickGet();	
	while (1) {
		status = TxRx_SendPacket(data, dataLen, bType);
		if (status == TXRX_NO_ERROR) {
			break;
		}
		t2 = MiWi_TickGet();
		if (MiWi_TickGetDiff(t2, t1) > TIMEOUT_RESENDING_PACKET) {	// waits a few seconds to get the whole command
			status = TXRX_UNABLE_SEND_PACKET;		
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
*      This function sending a data packet to other device. This function is
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
TXRX_ERRORS TxRx_SendData(BYTE* samples_block, WORD TX_message_length) {
	
	TXRX_ERRORS status = TxRx_SendPacketWithConfirmation(samples_block, TX_message_length, TXRX_TYPE_DATA);

	if (status != TXRX_NO_ERROR) {
		TxRx_PrintError(status);
	}

	return 0;	// YL NOTE: as if there is no error so after the timeout the stone would stop searching for non-existing nwk address and continue normally
}

/******************************************************************************
* Function:
*		TXRX_ERRORS m_TxRx_write(BYTE *str)
*
* Description:
*      The next function is responsible for transmitting "regular" responses.
*      In other words, this function is the equivalent to m_write for USB.
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
TXRX_ERRORS m_TxRx_write(BYTE *str) {
	
	WORD commandLen = strlen((char*)str);
	TXRX_ERRORS status = TxRx_SendPacketWithConfirmation(str, commandLen, TXRX_TYPE_COMMAND);
	
	if (status != TXRX_NO_ERROR) {
		TxRx_PrintError(status);
	}	
	return status;
}

#if defined COMMUNICATION_PLUG
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
*			strcpy(msg,cmd_Current);	
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
TXRX_ERRORS TxRx_SendCommand(BYTE* command) {
		
	TXRX_ERRORS status;
	WORD commandLen = strlen((char*)command);
	
	if (isBroadcast == TRUE) {
		// We use g_broadcast_counter and g_phase_counter to measure the phase 
		// between the stones. Therefore we need these counters only when the 
		// plug broadcasts a message with a command to all the stones in the 
		// network. Since this is a case of application-broadcast - the lower
		// layers (MiWi and MAC) do not treat it as such, and therefore the 
		// counters are added to all the command-messages (i.e. - in case of
		// application-unicast too, but we use the counters only for application-broadcast) 	
		#if defined DEBUG_PRINT
			m_write_debug("\r\n");
			m_write_debug("************** B R O A D C A S T **************");
			m_write_debug("\r\n");
		#endif
		// YL 12.1 ...
		g_broadcast_counter_start.Val = g_broadcast_counter.Val;  // start to record the broadcast-counter
		#if defined DEBUG_PRINT
			m_write_debug("BCStart: ");
			m_write_debug(int_to_str(g_broadcast_counter_start.Val));
			m_write_debug("\r\n");
			DelayMs(100);
		#endif		
		// ... YL 12.1
		BYTE destinationEUI0;
		for (destinationEUI0 = NWK_STARTER_ADDR_EUI0; destinationEUI0 < MAX_NWK_SIZE; destinationEUI0++) { 	// index "0" (the plug) is irrelevant
			if (isNetworkMember[destinationEUI0] == TRUE) {		
				finalDestinationNwkAddress[0] = destinationEUI0;
				#if defined DEBUG_PRINT
					m_write_debug("\r\n");
					m_write_debug("******* T O: ");
					m_write_debug(byte_to_str(finalDestinationNwkAddress[0]));
					m_write_debug("\r\n");
				#endif
				// YL 12.1 ...
				g_broadcast_counter_stop.Val = g_broadcast_counter.Val - g_broadcast_counter_start.Val; 	// read the record of the broadcast-counter
				if (g_broadcast_counter_stop.Val < 0) {
					g_broadcast_counter_stop.Val += 0xFFFF;		 // overflow
				}
				#if defined DEBUG_PRINT
					m_write_debug("BCStop: ");
					m_write_debug(int_to_str(g_broadcast_counter_stop.Val));
					m_write_debug("\r\n");
				#endif				
				g_phase_counter_start.Val = g_phase_counter.Val; // start to record the phase-counter	
				#if defined DEBUG_PRINT
					m_write_debug("PHStart: ");
					m_write_debug(int_to_str(g_phase_counter_start.Val));
					m_write_debug("\r\n");
				#endif				
				// ... YL 12.1				
				status = TxRx_SendPacketWithConfirmation(command, commandLen, TXRX_TYPE_COMMAND);
				if (status != TXRX_NO_ERROR) {
					TxRx_PrintError(status);
				}
			}
			// YL 8.11 ... remove if isn't effective
			DelayMs(100);
			// ... YL 8.11			
		}
	}
	else {
		status = TxRx_SendPacketWithConfirmation(command, commandLen, TXRX_TYPE_COMMAND);
		if (status != TXRX_NO_ERROR) {
			TxRx_PrintError(status);
		}		
	}
	#if defined DEBUG_PRINT
		m_write_debug("done!");
	#endif	
	return 0;	// as if there is no error so after the timeout 
				// the plug would stop searching for non-existing nwk address
				// and continue normally (after appropriate message display)
}
#endif 

#if defined ENABLE_TXRX_ACK // YL 25.12 added #ifdef
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
TXRX_ERRORS TxRx_SendAck() {
		
	TXRX_ERRORS status = TxRx_SendPacket(NULL, 0, TXRX_TYPE_ACK);
	
	#if defined WISDOM_STONE
		if (status == TXRX_NO_ERROR) {
			blockAckInfo.rxLastSeq = blockAckInfo.rxExpectedSeq;	// TODO YL TxRx_SendPacket of ack succeeded, so the transmitter got the ack... 			
		}
	#elif defined COMMUNICATION_PLUG
		if (status == TXRX_NO_ERROR) {
			blockAckInfo[rxFromEUI0].rxLastSeq = blockAckInfo[rxFromEUI0].rxExpectedSeq;	// TODO YL TxRx_SendPacket of ack succeeded, so the transmitter got the ack... 			
		}
	#endif // WISDOM_STONE

	return status;
}
#endif // ENABLE_TXRX_ACK

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_ReceivePacketHeader()
*
* Description:
*      This function is responsible for receiving the packet header, and getting information
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
TXRX_ERRORS TxRx_ReceivePacketHeader() {
	
	WORD i = 0;
	BYTE j = 0;
	BYTE messageInformation[MSG_INF_LENGTH]; 	// to replace rxMessage.Payload[0] and to make the reading of rxMessage.Payload more clear
	BYTE receivedSourceNwkDestination[MY_ADDRESS_LENGTH];
	BYTE receivedDestinationNwkDestination[MY_ADDRESS_LENGTH];
	
	for (i = 0; i < MSG_INF_LENGTH; i++) {
		messageInformation[i] = rxMessage.Payload[i];
	}
	rxBlock.blockHeader.blockType = ((messageInformation[0]) & TXRX_TYPE_MASK);	
	if (rxBlock.blockHeader.blockType > (TXRX_TYPE_MAX - 1)) {
		return TXRX_WRONG_BLOCK_TYPE;
	}	
	for (i = MSG_INF_LENGTH, j = 0;
		i < (MSG_INF_LENGTH + MY_ADDRESS_LENGTH); i++, j++) {
		receivedSourceNwkDestination[j] = rxMessage.Payload[i];
	}
	for (i = MSG_INF_LENGTH + MY_ADDRESS_LENGTH, j = 0; 
		i < (MSG_INF_LENGTH + 2 * MY_ADDRESS_LENGTH); i++, j++) {
		receivedDestinationNwkDestination[j] = rxMessage.Payload[i];
	}
	// make sure the message is indeed accepted by the addressed receiver:
	for (j = 0; j < MY_ADDRESS_LENGTH; j++)	{
		if (receivedDestinationNwkDestination[j] != myLongAddress[j]) {
			return TXRX_NWK_NOT_ME; 
		}
	}
	// the received source network address becomes the destination network address for the reply:
	for (j = 0; j < MY_ADDRESS_LENGTH; j++)	{
		finalDestinationNwkAddress[j] = receivedSourceNwkDestination[j];
	}
	
	#if defined ENABLE_TXRX_ACK		
		if ((rxBlock.blockHeader.blockType == TXRX_TYPE_COMMAND) 
			|| (rxBlock.blockHeader.blockType == TXRX_TYPE_DATA)) {					// the block is either command or data
			BYTE receivedDataSeq = (((messageInformation[0]) >> 2) & TXRX_SEQ_MASK);		
			#if defined WISDOM_STONE
				if (receivedDataSeq == ((blockAckInfo.rxLastSeq + 1) % MAX_ACK_LENGTH)) { 	// if the received sequence is +1 more than the last ack //YL 9.8 replaced 60 with MAX_ACK_LENGTH
					blockAckInfo.rxExpectedSeq = receivedDataSeq;
				}
				else if (receivedDataSeq == blockAckInfo.rxLastSeq ) {				// the previous ack was not received
					blockAckInfo.rxExpectedSeq = receivedDataSeq;
				}
				else {				
					return TXRX_WRONG_DATA_SEQ;
				}
			#elif defined COMMUNICATION_PLUG
				rxFromEUI0 = finalDestinationNwkAddress[0]; 
				if (receivedDataSeq == ((blockAckInfo[rxFromEUI0].rxLastSeq + 1) % MAX_ACK_LENGTH)) { 	// if the received sequence is +1 more than the last ack //YL 9.8 replaced 60 with MAX_ACK_LENGTH
					blockAckInfo[rxFromEUI0].rxExpectedSeq = receivedDataSeq;
				}
				else if (receivedDataSeq == blockAckInfo[rxFromEUI0].rxLastSeq ) {	// the previous ack was not received
					blockAckInfo[rxFromEUI0].rxExpectedSeq = receivedDataSeq;
				}
				else {					
					return TXRX_WRONG_DATA_SEQ;
				}
			#endif // WISDOM_STONE
		}
		if (rxBlock.blockHeader.blockType == TXRX_TYPE_ACK) {		 				// the ack is from the plug to the stone // YL maybe "else if" instead of "if"?
			BYTE receivedAckSeq = (((messageInformation[0]) >> 2) & TXRX_SEQ_MASK);				
			
			#if defined WISDOM_STONE
				if (receivedAckSeq == blockAckInfo.txExpectedSeq) {					// if the received ack is the same as the last data seq that we sent
					blockAckInfo.txLastSeq = blockAckInfo.txExpectedSeq; 				
					return TXRX_NO_ERROR;
				}
				else {			
					return TXRX_WRONG_ACK_SEQ; 
				}
			#elif defined COMMUNICATION_PLUG
				rxFromEUI0 = finalDestinationNwkAddress[0];
				if (receivedAckSeq == blockAckInfo[rxFromEUI0].txExpectedSeq) {		// if the received ack is the same as the last data seq that we sent
					blockAckInfo[rxFromEUI0].txLastSeq = blockAckInfo[rxFromEUI0].txExpectedSeq; 				
					return TXRX_NO_ERROR;
				}
				else {			
					return TXRX_WRONG_ACK_SEQ; 
				}
			#endif // WISDOM_STONE
		}
	#endif // ENABLE_TXRX_ACK

	// now we read the length of received message: 
	// i = MSG_INF_LENGTH + 2 * MY_ADDRESS_LENGTH	
	rxBlock.blockHeader.blockLen = 0;
	for (j = 0; j < MSG_LEN_LENGTH; j++) {
		rxBlock.blockHeader.blockLen <<= 8;
		rxBlock.blockHeader.blockLen += (WORD)rxMessage.Payload[i++];		
	}
	if (rxBlock.blockHeader.blockLen > MAX_BLOCK_SIZE) { 							
		rxBlock.blockHeader.blockLen = MAX_BLOCK_SIZE;	
		return TXRX_WRONG_PACKET_LENGTH;
	}	
	// YL 12.1 ...
	// now we read the phase of the received message (only in case of command message): 
	// i = MSG_INF_LENGTH + 2 * MY_ADDRESS_LENGTH + MSG_LEN_LENGTH	 
	if (rxBlock.blockHeader.blockType == TXRX_TYPE_COMMAND) {
		WORD rxPhase = 0;
		for (j = 0; j < MSG_PHS_LENGTH; j++) {
			rxPhase <<= 8;
			rxPhase += (WORD)(rxMessage.Payload[i++]);	
		}
		#if defined DEBUG_PRINT
			/*
			m_write_debug("4. ");
			m_write_debug(int_to_str((int)rxPhase));
			*/
		#endif
		g_phase_counter_start.Val += rxPhase; // no overflow is expected since g_phase_counter_start is zeroed when the buffer is received
	}		
	// ... YL 12.1	

	rxBlock.handlingParam.blockPos = 0;
	rxBlock.handlingParam.isHeader = FALSE;											// YL rxBlock.handlingParam.isHeader <- FALSE to enable receiving "non header" message portions; rxBlock.handlingParam.isHeader turns TRUE again after we receive the "trailer" portion  
	// i = MSG_INF_LENGTH + 2 * MY_ADDRESS_LENGTH + MSG_LEN_LENGTH [+ MSG_PHS_LENGTH]
	while (i < rxMessage.PayloadSize) {												// YL the receiver reads the data into rxBlock.blockBuffer ("data" - meaning - Payload bytes except for 3 first bytes of the header; these "data" bytes may include the trailer too)
		rxBlock.blockBuffer[rxBlock.handlingParam.blockPos++] = rxMessage.Payload[i++];
    }
	return TXRX_NO_ERROR;	
}	

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_ReceiveBuffer()
*
* Description:
*      This function is responsible for transferring the message in rxMessage struct
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

	WORD i;
		
    for (i = 0; i < rxMessage.PayloadSize; i++) {
   		rxBlock.blockBuffer[rxBlock.handlingParam.blockPos++] = rxMessage.Payload[i];
    }	
}

/******************************************************************************
* Function:
*		TXRX_ERRORS TxRx_ReceivePacketTrailer()
*
* Description:
*       This function is responsible for receiving the packet trailer, and
*		initialization for receiving the next packet.
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
	WORD i;
	
	for (i = 0; i < TXRX_TRAILER_SIZE; i++) {
		rxBlock.blockTrailer[i] = rxBlock.blockBuffer[rxBlock.blockHeader.blockLen + i];		
		if (rxBlock.blockTrailer[i] != TxRx_Trailer[i]) {						// YL check last TXRX_TRAILER_SIZE = 4 bytes of blockBuffer; these bytes should be identical to constant trailer string
			status = TXRX_RECEIVED_INVALID_TRAILER;
			break;
		}
	}
	rxBlock.handlingParam.isHeader = TRUE;										// we received the whole packet; next we are waiting for the header of the next block; //YL reset rxBlock fields for next transmission
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
*      This function is responsible for receiving a message. As mentioned, a packet
*	   consists of several messages, therefore this function is used by 
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
TXRX_ERRORS TxRx_ReceiveMessage() {
	
	/*******************************************************************/
	// Function MiApp_MessageAvailable will return a boolean to indicate 
	// if a message for application layer has been received by the 
	// transceiver. If a message has been received, all information will 
	// be stored in the rxMessage, structure of RECEIVED_MESSAGE.
	// YL TxRx_ReceiveMessage uses:
	// - TxRx_ReceivePacketHeader
	// - TxRx_ReceiveBuffer
	// - TxRx_ReceivePacketTrailer
	// to copy rxMessage.Payload bytes into the fields of rxBlock struct
	/*******************************************************************/
		
	if (MiApp_MessageAvailable()) {		         
		TXRX_ERRORS status;
		if (rxBlock.handlingParam.isHeader == TRUE) {				// it is the beginning of the block
			status = TxRx_ReceivePacketHeader();
			if (status != TXRX_NO_ERROR) {				
				MiApp_DiscardMessage();
				return status;
			}
		}
		else {
			TxRx_ReceiveBuffer();
		} 
		if ((rxBlock.handlingParam.blockPos) >= 
			(rxBlock.blockHeader.blockLen + TXRX_TRAILER_SIZE)) {	// we got the whole block
			status = TxRx_ReceivePacketTrailer();
			if (status != TXRX_NO_ERROR) {				
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
TXRX_ERRORS TxRx_ReceivePacket() {
	
	WORD i = 0;
	TXRX_ERRORS status = TXRX_NO_ERROR;
	
	for (i = 0; i < (MAX_BLOCK_SIZE); i++) {							// YL TxRx_ReceivePacket resets all rxBlock fields before calling TxRx_ReceiveMessage that actually recieves the data according to it's type (header\buffer\trailer)
		rxBlock.blockBuffer[i] = '\0';
	}
	rxBlock.blockHeader.blockType = 0;									
	rxBlock.handlingParam.isHeader = TRUE;
	rxBlock.handlingParam.isTrailer = FALSE;
	rxBlock.handlingParam.blockPos = 0;
	rxBlock.blockHeader.blockLen = 0;
	MIWI_TICK t1, t2;
	t1 = MiWi_TickGet();
		
	while (1) {
		status = TxRx_ReceiveMessage();									// receive the command packet
		if (status != TXRX_NO_PACKET_RECEIVED) {
			break;
		}
		t2 = MiWi_TickGet();		
		if ((MiWi_TickGetDiff(t2, t1)) > TIMEOUT_RECEIVING_MESSAGE) {	// waits a few seconds to get the whole command		
			status = TXRX_NO_PACKET_RECEIVED;
			break;
		}
	}
	if (status == TXRX_NO_ERROR) {										// we received at least the header of the packet
		t1 = MiWi_TickGet();
		while (rxBlock.handlingParam.isHeader == FALSE) {				// until we get the whole command - continue try getting it
			status = TxRx_ReceiveMessage();	
			if ((status != TXRX_NO_ERROR) && (status != TXRX_NO_PACKET_RECEIVED)) {
				break;
			}
			t2 = MiWi_TickGet();
			if ((MiWi_TickGetDiff(t2, t1)) > TIMEOUT_RECEIVING_MESSAGE) { // waits a few seconds to get the whole command
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
BYTE TxRx_ByteAdd(BYTE toAdd, BYTE addingTo) {

	if (toAdd >= 0xFF - addingTo) {
		return 0xFF;	// YL toAdd is 0xFF max, and then addingTo is 0, so there is no need to split to different cases; does the code reflect the original intention?
	}
	return addingTo + toAdd;
}

#if defined ENABLE_TXRX_ACK // YL 25.12 added #ifdef
/******************************************************************************
YS 22.12
This function resets the ACK sequencers.
******************************************************************************/
void TxRx_Reset_ACK_Sequencers() { // YS 22.12
	
	txBlock.blockHeader.ackSeq = 1; 
	#if defined WISDOM_STONE
		blockAckInfo.txLastSeq = 0;
		blockAckInfo.txExpectedSeq = 1;
		blockAckInfo.rxLastSeq = 0;
		blockAckInfo.rxExpectedSeq = 1;
	#elif defined COMMUNICATION_PLUG
	BYTE i;
	for (i = 1; i < MAX_NWK_SIZE; i++) {  	// index "0" (the plug) is irrelevant // TODO - check
		blockAckInfo[i].txLastSeq = 0;
		blockAckInfo[i].txExpectedSeq = 1;
		blockAckInfo[i].rxLastSeq = 0;
		blockAckInfo[i].rxExpectedSeq = 1;
	}
	#endif
}
#endif // ENABLE_TXRX_ACK

/******************************************************************************
* Function:
* 		int TxRx_PrintError(TXRX_ERRORS error)
******************************************************************************/
int TxRx_PrintError(TXRX_ERRORS error) {
	
	#if defined COMMUNICATION_PLUG		
		m_write(TxRx_err_messages[error]);
	#endif // COMMUNICATION_PLUG

	write_eol();
	return (-1);
}

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
#if defined COMMUNICATION_PLUG
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/******************************************************************************
* Function:
*		void TxRx_AppStop(void)
* Description:
*		TxRx_AppStop updates isStopped array only if isAppStop is TRUE
*		(the command "app stop" is sent separately)
* Parameters:
*		None
* Return value:
*		None
*******************************************************************************/
void TxRx_AppStop(void) {
		
	if (isBroadcast == FALSE) {	
		// update isStopped array at "EUI_0 of the stone" index:
		if (isAppStop == TRUE) {
			isStopped[(finalDestinationNwkAddress[0])] = TRUE;			
		}
		else {
			isStopped[(finalDestinationNwkAddress[0])] = FALSE;			
		}
	}
	else {	
		// in case of broadcast - update the whole isStopped array:
		BYTE i;	
		if (isAppStop == TRUE) {		
			for (i = 1; i < MAX_NWK_SIZE; i++) {	// index "0" (the plug) is irrelevant
				isStopped[i] = TRUE;
			}
		}
		else {		
			for (i = 1; i < MAX_NWK_SIZE; i++) {	// index "0" (the plug) is irrelevant
				isStopped[i] = FALSE;
			}
		}
	}
	return;
}

/******************************************************************************
* Function:
*		void TxRx_PrintNetworkTopology(void) 
*******************************************************************************/
void TxRx_PrintNetworkTopology(void) {
	
	MIWI_TICK 	t1, t2;
	
	t1 = MiWi_TickGet();	
	while (1) {		
		TxRx_ReceiveJoinInfo();
		t2 = MiWi_TickGet();
		if (currentNwkSize == MAX_NWK_SIZE || 
			MiWi_TickGetDiff(t2, t1) > TIMEOUT_NWK_ESTABLISHMENT) {
			break;
		}
	}
	m_write("******* N E T W O R K   T O P O L O G Y *******");
	write_eol();
	write_eol();
	m_write("NETWORK COORDINATORS: ");
	BYTE eui0;
	for (eui0 = 0; eui0 < MAX_NWK_SIZE; eui0++) {
		if (isCoordinator[eui0] == TRUE) {
			if (eui0 == 0) {
				m_write(byte_to_str(PLUG_NWK_ADDR_EUI0));
			}
			else {
				m_write(byte_to_str(eui0));
			}
			if (eui0 == NWK_STARTER_ADDR_EUI0) {
				m_write(" (PAN)");
			}
			m_write("  ");
		}
	}
	write_eol();
	write_eol();
	m_write("NETWORK MEMBERS:");
	write_eol();
	// the stones:
	for (eui0 = 1; eui0 < MAX_NWK_SIZE; eui0++) {
		m_write("\tSTONE eui0# - ");
		m_write(byte_to_str(eui0));
		if (isNetworkMember[eui0] == TRUE) {
			m_write(": yes, PARENT eui0# - ");
			m_write(byte_to_str(parentDeviceEUI0[eui0]));
		}
		else {
			m_write(": no");
		}
		write_eol();
		// YL 8.11 ... remove if isn't effective
		DelayMs(100);
		// ... YL 8.11
	}
	// the plug:
	m_write("\tPLUG  eui0# - ");
	m_write(byte_to_str(PLUG_NWK_ADDR_EUI0));
	if (isNetworkMember[0] == TRUE) {
		m_write(": yes, PARENT eui0# - ");
		m_write(byte_to_str(parentDeviceEUI0[0]));
	}
	else {
		m_write(": no");
	}
	write_eol();
	write_eol();	
}

/******************************************************************************
* Function:
*		void TxRx_Reconnect(void)
* Description:
*		Run MiApp_ResyncConnection() to reconnect the network connection
* Parameters:
*		None
* Return value:
*		None
*******************************************************************************/
void TxRx_Reconnect(void) {

	// YL 6.9 ... meanwhile
	return;
	// ... YL 6.9

	if (MiApp_ResyncConnection(0, 0xFFFFFFFF)) { // YL 5.8 TODO - check MiApp_ResyncConnection params
		m_write ("RECONNECTED: network connection recovery successful!");
	} 
	else {
		m_write ("NOT-CONNECTED: couldn't recover network connection!");				
	}
	write_eol();
	cmd_ok();
}

/******************************************************************************
* Function:
*		BOOL TxRx_ExecuteIfPlugCommand(void) 
* Description:
*		- check the kind of the received command:
*			- if this is "app stop", update isAppStop 
* 		- check if the command is for the plug:
*			- if the command is for the plug - execute it
*			- otherwise we consider it "stone" command: 
*				- NOTE: this "stone" command might also be some illegal string,
*					and to find out this - the stone should parse it after 
*					receiving it from the plug
*				- update global finalDestinationNwkAddress (the destination is some stone in the nwk)	
* Parameters:
*		None
* Return value:
* 		- TRUE:
*			- if the command is for the plug, or
*			- if the plug found that the input is invalid (err msg is printed)
* 		- FALSE: otherwise (the command is presumable "stone" command)
* Side effects:
*		- updates global finalDestinationNwkAddress (if the input is valid)
*		- updates global isBroadcast if needed	
*		- calls handle_plug_msg() that:
*			- removes the destination from the beginning of g_curr_msg (so only the cmd remains)
*			- updates global isAppStop if we received "app stop"	
******************************************************************************/	
BOOL TxRx_ExecuteIfPlugCommand(void) {
		
	int result = handle_plug_msg();
	if (result == (-1)) {
		return TRUE; 	// to avoid sending illegal command to the stone
	}
	
	isBroadcast = FALSE;
	isAppStop = FALSE;
	finalDestinationNwkAddress[0] = (0xFF & result);	// EUI_0
	finalDestinationNwkAddress[1] = EUI_1;				// constant; 
		
	switch (finalDestinationNwkAddress[0]) {	
		case PLUG_NWK_ADDR_EUI0:	
			TxRx_Reconnect();	// execute the only plug command 
			return TRUE;
		case BROADCAST_NWK_ADDR:	
			isBroadcast = TRUE;	
	}
	if (finalDestinationNwkAddress[0] == PLUG_NWK_ADDR_EUI0) { 	// we should get here only with stone commands
		TxRx_PrintError(TXRX_NWK_UNKNOWN_ADDR);
		return TRUE;
	}
	TxRx_AppStop();
	return FALSE;
}

#endif // #ifdef COMMUNICATION_PLUG

