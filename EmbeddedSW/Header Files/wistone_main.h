#ifndef __WISTONE_MAIN_H__					
#define __WISTONE_MAIN_H__

/***** INCLUDE FILES: *********************************************************/

#include "GenericTypeDefs.h"
#include "command.h"

/***** DEFINE: ****************************************************************/

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//Uncomment ONE of the 2 following lines to choose application type: 
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//#define WISDOM_STONE
#define COMMUNICATION_PLUG

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//Uncomment ONE of the 2 following lines to choose mode of communication:
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
#define USBCOM
//#define RS232COM

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//FOR DEBUG:
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
#define WISTONE_BOARD	
//#define EXPLORER16
//#define LCD_INSTALLED			// indicates we have the LCD installed for debug messages
//#define DEBUG_PRINT				

#define USE_AND_OR				// indicates for the compiler to use bit-wise flags for configurations
#define MAX_BLOCK_SIZE 512		// sample/transmit/store block size in bytes

//YL 8.12 moved here CYCLIC_BUFFER_SIZE definitions to eliminate the dependency between ads1282 and accmtr
#define	CYCLIC_BUFFER_SIZE	4

#if CYCLIC_BUFFER_SIZE == 2
	#define CYCLIC_BLOCK_MASK	0x03FF
#elif CYCLIC_BUFFER_SIZE == 4
	#define CYCLIC_BLOCK_MASK	0x07FF
#elif CYCLIC_BUFFER_SIZE == 8
	#define CYCLIC_BLOCK_MASK	0x0FFF
#else
	#error "CYCLIC_BUFFER_SIZE should be 2,4 or 8"
#endif

extern BOOL g_usb_connected;
extern BYTE g_sleep_request;
extern char g_curr_msg[MAX_CMD_LEN]; 

#endif //__WISTONE_MAIN_H__
