//DOM-IGNORE-BEGIN
/*********************************************************************
 * The following lines are used by VDI.
 * GUID=E537A0C0-6FEE-4afd-89B9-0C35BF72A80B
 * GUIInterfaceVersion=1.00
 * LibraryVersion=2.4
 *********************************************************************/
//DOM-IGNORE-END
/*******************************************************************************

    wistone_USB Header File

Summary:


Description:
    
********************************************************************/
//DOM-IGNORE-END

#ifndef _WISTONE_USB_H_
#define _WISTONE_USB_H_
//DOM-IGNORE-END

#include "GenericTypeDefs.h"

#define MAX_CMD_LEN 100		

typedef enum{

	USB_NO_ERROR				= 0,
	USB_NOT_CONFIGURED			= 1,
	USB_RECEIVED_DATA			= 2,
	USB_NOT_RECEIVED_DATA		= 3,
	USB_NOT_READY_TO_SEND_DATA	= 4

}USB_STATUS;


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
 *					len - Thhe length of the data to be written.
 *
 * Output:          None.					
 *
 * Side Effects:    None
 *
 * Overview:        This is the primary user interface function for sending data
 *					through usb.
 *					Any sending to the usb sould be done through this function.
 *
 * Note:            None
 *****************************************************************************/
void USB_WriteData(BYTE *str, WORD len);


/******************************************************************************
 * Function:        USB_STATUS USB_ReceiveData()
 *
 * PreCondition:    None.
 *
 * Input:           None.
 *
 * Output:          Any status found in USB_STATUS, yet the relevant are USB_RECEIVED_DATA
 *					and USB_NOT_RECEIVED_DATA.				
 *
 * Side Effects:    None
 *
 * Overview:        This is the primary user interface function for receiving data
 *					and commands through usb.
 *					This function should be called regulary (every iteration of the
 *					the main loop), as it performes periodic tasks of the usb.
 *
 * Note:            None
 *****************************************************************************/
USB_STATUS USB_ReceiveData();



#endif // _WISTONE_USB_H_
