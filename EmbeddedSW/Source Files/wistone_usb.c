/*******************************************************************************

wistone_usb.c - handle USB
==========================

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
this file contains function for operating the USB.
- initialize the USB
- send data to USB
- receive data from USB

*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include "usb.h"
#include "usb_function_cdc.h"
#include "HardwareProfileUSB.h"
#include "HardwareProfileTxRx.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "usb_device.h"
#include "usb.h"
#include "wistone_usb.h"
#include "SymbolTime.h"
// YL 5.11 ... for "app stop"
#include "parser.h" 
// ... YL 5.11

/***** DEFINE: ****************************************************************/
// YL 5.11 ... added to eliminate magic numbers
#define USB_TX_BUFFER_SIZE 			64 
#define USB_RX_BUFFER_SIZE 			64 
#define END_OF_COMMAND 				('\r')
#define USB_BUFFER_SIZE				40 // either TX or RX 
#define TIMEOUT_WRITING_USB_BUFFER (5 * ONE_SECOND)
// ... YL 5.11	

typedef enum {

	USB_PROCESS_IN = 0,		// YL 5.11 from the terminal to the plug
	USB_PROCESS_OUT,		// YL 5.11 from the plug to the terminal
	USB_PROCESS_IN_AND_OUT

} USB_PROCESS_DIRECTION;


/***** INTERNAL PROTOTYPES: ***************************************************/
void InitializeSystem(void);
USB_STATUS ProcessIO(USB_PROCESS_DIRECTION processInOrOut);
USB_STATUS USB_ProcessIn();
USB_STATUS USB_ProcessOut();
void USB_WriteSingleBuffer(WORD len);
void USBDeviceTasks(void);
void USBCBSendResume(void);
// YL 9.9 ... all the following aren't used
// void YourHighPriorityISRCode();
// void YourLowPriorityISRCode();
// void BlinkUSBStatus(void);
// void UserInit(void);
// ... YL 9.9

// YL 5.11 ...
void USB_WritePrompt(void);  
// ... YL 5.11

/***** GLOBAL VARIABLES: ******************************************************/
typedef struct {

	char txBuffer[USB_TX_BUFFER_SIZE]; // YL 5.11 was: txBuffer[64]
	char rxBuffer[USB_RX_BUFFER_SIZE]; // YL 5.11 was: rxBuffer[64]
	WORD numBytesToWrite;

} USB_BUFFER;

USB_BUFFER usbBuffer;

// YL 5.11: 
// the definitions in this file:
// > TX - for transfers sent to the terminal via USB (only data)
// > RX - for transfers sent to the plug via USB (only commands), whereas:
//
// [PLUG (PIC, IN)] <------USB------> [(PC, OUT) TERMINAL] 

/** VECTOR REMAPPING **********************************************************/
#if defined(__C30__) // TODO - may be removed?
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        /*
         *	ISR JUMP TABLE
         *
         *	It is necessary to define jump table as a function because C30 will
         *	not store 24-bit wide values in program memory as variables.
         *
         *	This function should be stored at an address where the goto instructions 
         *	line up with the remapped vectors from the bootloader's linker script.
         *  
         *  For more information about how to remap the interrupt vectors,
         *  please refer to AN1157.  An example is provided below for the T2
         *  interrupt with a bootloader ending at address 0x1400
         */
//        void __attribute__ ((address(0x1404))) ISRTable(){
//        
//        	asm("reset"); //reset instruction to prevent runaway code
//        	asm("goto %0"::"i"(&_T2Interrupt));  //T2Interrupt's address
//        }
    #endif

#else
	#error "You picked a wrong pic"
#endif

/*******************************************************************************
 * Function:        void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 ******************************************************************************/
void InitializeSystem(void) // YL 3.11 there is one difference with the version in usb_main_boaz.c
{

   
	// AD1PCFGL = 0xFFFF; // YL 3.11 AY added, the addition is commented because it overrides the initialization of this register in analog_init() 
   

//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    TRIS_USB_BUS_SENSE = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
    TRIS_SELF_POWER = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
		
} //end InitializeSystem

/*******************************************************************************
 * Function:        USB_STATUS ProcessIO(USB_PROCESS_DIRECTION processInOrOut)
 *
 * PreCondition:    None
 *
 * Input:           
 *					ProcessInOrOut - indicates whether it is desired to send
 *									 or to receive data.
 *
 * Output:          The status of the operation.
 *
 * Side Effects:    None
 *
 * Overview:        This function should be called regularly for maintaining
 *					sending and receiving packets.
 *					This is an internal function that is used by USB_ReceiveDataFromHost()
 *					and USB_SendData() user interface functions.
 *
 * Note:            None
 ******************************************************************************/
USB_STATUS ProcessIO(USB_PROCESS_DIRECTION processInOrOut) // YL 3.11 - added by AY 
{   
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) {
		return USB_NOT_CONFIGURED;
	}

	USB_STATUS status = USB_NO_ERROR;

	if (processInOrOut == USB_PROCESS_IN_AND_OUT) {
		USB_ProcessOut();
		status = USB_ProcessIn();
	}

	else if (processInOrOut == USB_PROCESS_OUT) {
		status = USB_ProcessOut();
	}

	else if (processInOrOut == USB_PROCESS_IN) {
		status = USB_ProcessIn();
	}

    CDCTxService();

	return status;
}	

/*******************************************************************************
 * Function:        USB_STATUS USB_ProcessIn(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          
 *					USB_NOT_RECEIVED_DATA.
 *					USB_RECEIVED_DATA.
 *
 * Side Effects:    None
 *
 * Overview:        This function is responsible for processing the data that  
 *					was sent to the plug. The function copies the received data 
 * 					to g_curr_msg buffer, and when END_OF_COMMAND is received - 
 *					the command is ready for execution.
 *					This is an internal function that is used by ProcessIO.
 *
 * Note:            None
 ******************************************************************************/
USB_STATUS USB_ProcessIn(void) // YL 3.11 - added by AY; YL 5.11 - edited; the backup is right below // YL 6.12 TODO - if the USB will be eventually used only with the plug - maybe USB_ProcessIn should be better merged with handle_plug_msg
{
	BYTE numBytesRead;
	static BYTE commandPos = 0;
	
	numBytesRead = getsUSBUSART(usbBuffer.rxBuffer, USB_RX_BUFFER_SIZE); // YL 5.11 - "get" into the plug, receive, host_to_device
	if (numBytesRead <= 0) {
		return USB_NOT_RECEIVED_DATA;
	}

	WORD i;
	for (i = 0; i < numBytesRead; i++) {
		g_curr_msg[commandPos + i] = usbBuffer.rxBuffer[i];
		if (usbBuffer.rxBuffer[i] == END_OF_COMMAND) {
			g_curr_msg[commandPos + i] = '\0';
			tokenize(g_curr_msg);	
			if ((strcmp("app", g_tokens[1]) == 0) && (strcmp("stop", g_tokens[2]) == 0)) {	// the command is "app stop"
				USB_WritePrompt();
			}			
			USB_WriteData((BYTE*)g_curr_msg, commandPos + i); 	// YL 14.4 added casting to avoid signedness warning
			USB_WritePrompt();			
			commandPos = 0;
			return USB_RECEIVED_DATA;
		}
	}
	commandPos += numBytesRead;

	return USB_NOT_RECEIVED_DATA;	// didn't receive the entire command, since END_OF_COMMAND was not received
}

/*******************************************************************************
 * Function:		void USB_WritePrompt(void)
 ******************************************************************************/ 
void USB_WritePrompt(void) // YL 5.11 
{
	char* prompt_str = "\r\nWISTONE> ";
	int prompt_len  = strlen(prompt_str);
	
	USB_WriteData((BYTE*)prompt_str, (WORD)prompt_len);
}

// BACKUP:
// USB_STATUS USB_ProcessIn() // YL 3.11 - added by AY
// {
//	BYTE numBytesRead =  getsUSBUSART(usbBuffer.rxBuffer, USB_RX_BUFFER_SIZE);
//	
//	if (numBytesRead <= 0) {
//		return USB_NOT_RECEIVED_DATA;
//	}
//
//	static BYTE commandPos = 0;
//
//	WORD i = 0;
//	for (i = 0; i < numBytesRead; i++) {
//		g_curr_msg[commandPos + i] = usbBuffer.rxBuffer[i];
//
//		if (usbBuffer.rxBuffer[i] == '\r'){
//			g_curr_msg[commandPos + i] = '\0';
//			if (strcmp(g_curr_msg, "app stop") == 0){			
//				USB_WriteData((BYTE*)"\r\nWISTONE> ", 11); 		
//			}			
// 			USB_WriteData((BYTE*)g_curr_msg, commandPos + i); 	// YL 14.4 added casting to avoid signedness warning
//			USB_WriteData((BYTE*)"\r\nWISTONE> ", 11); 		
//			commandPos = 0;
//			return USB_RECEIVED_DATA;
//		}
//	}
//
//	commandPos += numBytesRead;
//
//	return USB_NOT_RECEIVED_DATA;	// Not received all the command, since '\r' was not received
// }

/*******************************************************************************
 * Function:        USB_STATUS USB_ProcessOut(void)
 *
 * PreCondition:    usbBuffer.txBuffer should be filled with the desired data.
 *
 * Input:           None
 *
 * Output:          
 *					USB_NO_ERROR.
 *					USB_NOT_READY_TO_SEND_DATA.
 *
 * Side Effects:    None
 *
 * Overview:        This function is responsible for sending the data to the terminal.
 *					Therefore, usbBuffer.txBuffer should be filled with the desired data
 *					before calling this function.
 *
 * Note:            None
 ******************************************************************************/
USB_STATUS USB_ProcessOut(void) // YL 3.11 - added by AY
{
    if (USBUSARTIsTxTrfReady()) {		
		putUSBUSART(usbBuffer.txBuffer,	usbBuffer.numBytesToWrite); // YL 5.11 - "put" to the terminal, send, device_to_host
		return USB_NO_ERROR;
	}
	return USB_NOT_READY_TO_SEND_DATA;
}

/*******************************************************************************
 * Function:        void USB_WriteData(BYTE *str, WORD len)
 *
 * PreCondition:    None
 *
 * Input:           
 *					str - The data to write through usb.
 *					len - The length of the data to be written.
 *
 * Output:          None					
 *
 * Side Effects:    None
 *
 * Overview:        This is the primary user interface function for sending data
 *					through usb.
 *					Any sending to the usb should be done through this function.
 *
 * Note:            None
 ******************************************************************************/
void USB_WriteData(BYTE *str, WORD len) // YL 3.11 - added by AY
{
	WORD bufferPos = 0;
	WORD i;

	for (i = 0; i < len; i++) {
		bufferPos = i%(USB_BUFFER_SIZE);				// YL 5.11 was: i%40
		if ((bufferPos == 0) && (i > 0)) {
			USB_WriteSingleBuffer(USB_BUFFER_SIZE);		// YL 5.11 was: USB_WriteSingleBuffer(40);
		}
		usbBuffer.txBuffer[bufferPos] = str[i];
	}
	// write the remainder of the data
	USB_WriteSingleBuffer((bufferPos + 1));
	// YL 6.12 ... commented because we get here after USB_ReceiveDataFromHost is called every main iteration
	// USB_ReceiveDataFromHost(); // YL 25.11 added for periodic tasks 
	// ... YL 6.12
}

/*******************************************************************************
 * Function:        void USB_WriteSingleBuffer(WORD len)
 *
 * PreCondition:    The data should be written to usbBuffer.txBuffer.
 *
 * Input:           
 *					len - The length of the data to be written.
 *
 * Output:          None					
 *
 * Side Effects:    None
 *
 * Overview:        This function writes single buffer of USB_BUFFER_SIZE.
 *					The data should be written to usbBuffer.txBuffer before
 *					calling this function.				
 *
 * Note:            None
 ******************************************************************************/
void USB_WriteSingleBuffer(WORD len) // YL 3.11 - added by AY
{
	USB_STATUS status;
	TICK t1, t2;		// YL 5.11 was: MIWI_TICK
	
	usbBuffer.numBytesToWrite = len;

	t1 = TickGet(); 	// YL 5.11 was: MiWi_TickGet
	while (1) {
		status = ProcessIO(USB_PROCESS_OUT);
		if (status == USB_NO_ERROR) {
			break;
		}
		t2 = TickGet();
		if (TickGetDiff(t2, t1) > TIMEOUT_WRITING_USB_BUFFER) { // YL 5.11 was: MiWi_TickGetDiff
			break;
		}
	}
}

/*******************************************************************************
 * Function:        USB_STATUS USB_ReceiveDataFromHost(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Any status found in USB_STATUS, yet the relevant are 
 *					USB_RECEIVED_DATA and USB_NOT_RECEIVED_DATA.				
 *
 * Side Effects:    None
 *
 * Overview:        This is the primary user interface function for receiving data
 *					and commands through usb.
 *					This function should be called regularly (every iteration of the
 *					the main loop), as it performs periodic tasks of the usb.
 *
 * Note:            None
 ******************************************************************************/
USB_STATUS USB_ReceiveDataFromHost(void) // YL 3.11 - added by AY; YL 7.11 was: USB_ReceiveData, renamed to USB_ReceiveDataFromHost
{
	USB_STATUS status = ProcessIO(USB_PROCESS_IN); 	
	return status;
}

// YL 7.11 ... added USB_SendDataToHost
/******************************************************************************
 * Function:        USB_STATUS USB_SendDataToHost(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None				
 *
 * Side Effects:    None
 *
 * Overview:        This is the primary user interface function for sending data
 *					through usb.
 *
 * Note:            None
 *****************************************************************************/
void USB_SendDataToHost(void) 	// YL 5.11 the massive data flow is expected from device to the host
{
	ProcessIO(USB_PROCESS_OUT);
}
// ... YL 7.11


// *****************************************************************************
// ************************** USB Callback Functions ***************************
// *****************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response 
// to certain USB related events.  For example, if the host PC is powering down, 
// it will stop sending out Start of Frame (SOF) packets to your device.  
// In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this
// condition (which according to the USB specifications is 3+ms of no bus 
// activity/SOF packets) and then calls the USBCBSuspend() function.  You should 
// modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code
// that will decrease power consumption from Vbus to <2.5mA (such as by clock
// switching, turning off LEDs, putting the microcontroller to sleep, etc.).  
// Then, in the USBCBWakeFromSuspend() function, you may then wish to add 
// code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not 
// automatically call this function.  This function is meant to be called from 
// the application firmware instead.  See the additional comments near the function.
/*******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 ******************************************************************************/
void USBCBSuspend(void) // YL 3.11 identical to the version in usb_main_boaz.c
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behaviour.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

    #if defined(__C30__)
        USBSleepOnSuspend();
    #endif
}

/*******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 ******************************************************************************/
void USBCBWakeFromSuspend(void) // YL 3.11 identical to the version in usb_main_boaz.c
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/*******************************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 ******************************************************************************/
void USBCB_SOF_Handler(void) // YL 3.11 identical to the version in usb_main_boaz.c
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 ******************************************************************************/
void USBCBErrorHandler(void) // YL 3.11 identical to the version in usb_main_boaz.c
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}

/*******************************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfil the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 ******************************************************************************/
void USBCBCheckOtherReq(void) // YL 3.11 identical to the version in usb_main_boaz.c
{
    USBCheckCDCRequest();
}//end

/*******************************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 ******************************************************************************/
void USBCBStdSetDscHandler(void) // YL 3.11 identical to the version in usb_main_boaz.c
{
    // Must claim session ownership if supporting this request
}//end

/*******************************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 ******************************************************************************/
void USBCBInitEP(void) // YL 3.11 identical to the version in usb_main_boaz.c
{
    CDCInitEP();
}

/*******************************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behaviour, 
 *                  as a USB device that has not been armed to perform remote 
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *                  
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are 
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex: 
 *                  any time application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup. 
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in 
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signalling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 ******************************************************************************/
void USBCBSendResume(void)	// YL 3.11 identical to the version in usb_main_boaz.c, besides RFIE that AY added
{
    static WORD delay_count;
    
    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager 
    //properties page for the USB device, power management tab, the 
    //"Allow this device to bring the computer out of standby." checkbox 
    //should be checked).

	BYTE oldRFIE = RFIE; 	// YL 3.11 AY

    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
			RFIE = 0; 		// YL 3.11 AY
            USBMaskInterrupts();
            
            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at 
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yields
            //5ms+ total delay since start of idle).
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signalling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
			RFIE = oldRFIE;				// YL 3.11 AY
        }
    }
}

/*******************************************************************************
 * Function:        void USBCBEP0DataReceived(void)
 *
 * PreCondition:    ENABLE_EP0_DATA_RECEIVED_CALLBACK must be
 *                  defined already (in usb_config.h)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called whenever a EP0 data
 *                  packet is received.  This gives the user (and
 *                  thus the various class examples a way to get
 *                  data that is received via the control endpoint.
 *                  This function needs to be used in conjunction
 *                  with the USBCBCheckOtherReq() function since 
 *                  the USBCBCheckOtherReq() function is the apps
 *                  method for getting the initial control transfer
 *                  before the data arrives.
 *
 * Note:            None
 ******************************************************************************/
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK) // YL 3.11 identical to the version in usb_main_boaz.c
void USBCBEP0DataReceived(void)
{
}
#endif

/*******************************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occurred.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 ******************************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size) // YL 3.11 identical to the version in usb_main_boaz.c, besides the coercion that AY added
{
    switch( (INT)event ) // no coercion in usb_main_boaz.c; INT = signed int
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was 
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was 
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT 
            //      endpoints).
            break;
        default:
            break;
    }      
    return TRUE; 
}

/** EOF wistone_usb.c *********************************************************/
