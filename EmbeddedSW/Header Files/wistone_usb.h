#ifndef _WISTONE_USB_H_
#define _WISTONE_USB_H_

/********************************************************************
	YL 3.11 - from comparison to usb_main_boaz.h - 
	only InitializeSystem appears there (with a few changes)
********************************************************************/

#include "GenericTypeDefs.h"

#define MAX_CMD_LEN 100	

typedef enum {

	USB_NO_ERROR = 0,
	USB_NOT_CONFIGURED,
	USB_RECEIVED_DATA,			
	USB_NOT_RECEIVED_DATA,		
	USB_NOT_READY_TO_SEND_DATA

} USB_STATUS;

/********************************************************************
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
 *******************************************************************/
void InitializeSystem(void);

/******************************************************************************
 * Function:        void USB_WriteData(BYTE *str, WORD len)
 *
 * PreCondition:    None.
 *
 * Input:           
 *					str - The data to write through usb.
 *					len - The length of the data to be written.
 *
 * Output:          None					
 *
 * Side Effects:    None
 *
 * YL 7.11 - changed overview after adding USB_SendDataToHost
 * Overview:        This is the primary user interface function for writing data
 *					and commands using usb. 
 * was: Overview:   This is the primary user interface function for sending data
 *					through usb.
 *					Any sending to the usb should be done through this function.
 *
 * Note:            None
 *****************************************************************************/
void USB_WriteData(BYTE *str, WORD len);

/******************************************************************************
 * Function:        USB_STATUS USB_ReceiveDataFromHost(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Any status found in USB_STATUS, yet the relevant are USB_RECEIVED_DATA
 *					and USB_NOT_RECEIVED_DATA.				
 *
 * Side Effects:    None
 *
 * Overview:        This is the primary user interface function for receiving data
 *					and commands through usb. - YL commands only?
 *					This function should be called regularly (every iteration of the
 *					the main loop), as it performs periodic tasks of the usb.
 *
 * Note:            None
 *****************************************************************************/
USB_STATUS USB_ReceiveDataFromHost(void);

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
 *					This function should be called regularly (every iteration of the
 *					the main loop), as it performs periodic tasks of the usb.
 *
 * Note:            None
 *****************************************************************************/
void USB_SendDataToHost(void);
// ... YL 7.11

#endif // _WISTONE_USB_H_
