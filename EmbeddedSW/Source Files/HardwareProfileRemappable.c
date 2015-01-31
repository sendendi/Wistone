/*******************************************************************************

HarwareProfileRemappable.c 
===========================

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
this file contains functions for configuring remappable pins (uart, mrf, flash);
mrf and flash both need to configure OSCCON register, and it's locking
and unlocking is possible only once a process.

*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
//Common
#include "Compiler.h"
#include "HardwareProfileRemappable.h"
#include "HardwareProfileTxRx.h"	//YL 12.8 for BUTTON_1_TRIS ...
#include "analog.h"

/***** function prototypes: ***************************************************/
void PPS_accmtr_config(void);
void PPS_ads1282_config(void);
void PPS_uart2_config(void);
void PPS_flash_config(void);
void PPS_mrf49_config(void);
void PPS_mrf24_config(void);

//************************************************************
// PPS_config():
// configure all remappable pins of the HW blocks we use.
//************************************************************
void PPS_config(){

	analog_init();				// init the analog inputs, all the rest are digital

	// Unlock Registers
	asm volatile ( "MOV #OSCCON, w1 \n"
	"MOV #0x46, w2 \n"
	"MOV #0x57, w3 \n"
	"MOV.b w2, [w1] \n"
	"MOV.b w3, [w1] \n"
	"BCLR OSCCON,#6");
	
#if defined RS232COM	
	PPS_uart2_config();			// configure UART ports
#endif // #if defined RS232COM
	// configure RXTX:
#if defined (MRF49XA)		
	PPS_mrf49_config();			// configure MRF49
#elif defined (MRF24J40)
	PPS_mrf24_config(); 		// configure MRF24
#endif // #if defined (MRF49XA)
	PPS_flash_config();			// configure FLASH
	PPS_accmtr_config();		// configure Accelerometer
	PPS_ads1282_config();		// configure ADS1282
#if defined (EXPLORER16)
	led_and_switch_init();
#endif // #if defined (EXPLORER16)
	
	// Re-Lock Registers
	asm volatile ( "MOV #OSCCON, w1 \n"
	"MOV #0x46, w2 \n"
	"MOV #0x57, w3 \n"
	"MOV.b w2, [w1] \n"
	"MOV.b w3, [w1] \n"
	"BSET OSCCON, #6" );
}

//************************************************************
// Function PPS_uart2_config()
//************************************************************
void PPS_uart2_config(void){
	// Configure Input Functions (Table 9-1))
	RPINR19bits.U2RXR = 10;			// Assign U2RX To Pin RP0
	RPINR19bits.U2CTSR = 32;		// Assign U2CTS To Pin RP1
	// Configure Output Functions (Table 9-2)
	RPOR8bits.RP17R = 5;			// Assign U2TX To Pin RP17
	RPOR15bits.RP31R = 6; 			// Assign U1RTS To Pin RP3
}

//************************************************************
// Function PPS_flash_config()
// Initialize the SPI for flash
// make sure these pins are digital
// make sure we unlock the pins mapping before
//************************************************************
void PPS_flash_config(void) {

#if defined (EXPLORER16)
	RPINR20bits.SDI1R = 19; 		// SPI1SDI input
	RPOR10bits.RP21R = 8; 			// SPI1CLK output
	RPOR13bits.RP26R = 7;			// SPI1DO output
	//enable a pull-up for the card detect, just in case the SD-Card isn't attached
	//then lets have a pull-up to make sure we don't think it is there.
	CNPU1bits.CN6PUE = 1; 
#elif defined (WISTONE_BOARD)
	RPINR20bits.SDI1R = 31; 	    // SPI1SDI input (SC changed to RP4), was RPINR20bits.SDI1R = 19
	RPOR1bits.RP3R = 8; 			// SPI1CLK output (SC changed to RP3), was RPOR10bits.RP21R = 8
	RPOR2bits.RP4R = 7;				// SPI1DO output  (SC changed to RP31), was RPOR13bits.RP26R = 7			
#endif // #if defined (EXPLORER16)
}

//************************************************************
// Function PPS_mrf49_config()
// make sure we nulock the pins mapping before
//************************************************************
void PPS_mrf49_config(void){

#if defined (EXPLORER16)
	// Input functions			
	//***************************
	// Assign SDI1 to Pin RPI43 = RD14
	//***************************
	RPINR22bits.SDI2R = 43;	 //.43. represents RPI43

	//*********************************************
	// Assign External Interrupt 1 to Pin RPI33 = RE8	
	//*********************************************
	RPINR0bits.INT1R = 33;	 //.23 represents RP23	

	// Output functions			
	//***************************
	// Assign SDO1 to Pin RP15 = RF8
	//***************************
	RPOR7bits.RP15R = 10; 		//.7. represents function SDO1

	//***************************
	// Assign SCK1OUT to Pin RP0 = RB0
	//***************************
	RPOR0bits.RP0R = 11; 		//.8. represents function SCK1OUT

	//***************************
	// Assign SS1OUT to Pin RP13 = RB2
	//***************************
	RPOR6bits.RP13R = 12; 		//.8. represents function SS1OUT
#elif defined (WISTONE_BOARD)
	// Input functions			
	//***************************
	// Assign SDI2 to Pin RPI43 = RD14
	//***************************
	RPINR22bits.SDI2R = 27;      	// SC changed to RP27, was RPINR22bits.SDI2R = 43;

	//*********************************************
	// Assign External Interrupt 1 to Pin RPI33 = RE8	
	//*********************************************
	RPINR0bits.INT1R = 11;       	// SC changed to RP11 and commented, was RPINR0bits.INT1R = 33;	 

	// Output functions			
	//***************************
	// Assign SDO2 to Pin RP15 = RF8
	//***************************
	RPOR8bits.RP16R = 10;          	// SC changed to RP16, was RPOR7bits.RP15R = 10; //.10. represents function SDO2

	//***************************
	// Assign SCK1OUT to Pin RP0 = RB0
	//***************************
	RPOR9bits.RP18R = 11;        	// SC changed to RP18, was RPOR0bits.RP0R = 11;  //.11. represents function SCK2OUT

	//***************************
	// Assign SS2OUT to Pin RP13 = RB2	
	//***************************
	//RPOR6bits.RP13R = 12; 		//.12. represents function SS2OUT // SC void SS2OUT for debug
#endif // #if defined (EXPLORER16)
}


//************************************************************
// Function PPS_mrf24_config()
// make sure we nulock the pins mapping before
//************************************************************
void PPS_mrf24_config(void){

	// Input functions
	//*********************************************
	// Assign SDI1 to Pin RP4 = RD9
	//*********************************************
	//RPINR22bits.SDI2R = 4;	 //.4. represents RP4 - SC void for debug
	
	//*********************************************
	// Assign External Interrupt 1 to Pin RP23 = RD2	
	//*********************************************
	RPINR0bits.INT1R = 23;	 //.23. represents RP23	
	
	
	// Output functions
		//**********************************************
	// Assign SDO1 to Pin RP22 = RD3
	//**********************************************
	RPOR11bits.RP22R = 10; 		//.10. represents function SDO2

	//**********************************************
	// Assign SCK1OUT to Pin RP2 = RD8
	//**********************************************
	RPOR1bits.RP2R = 11; 		//.11.  represents function SCK2OUT

	//**********************************************
	// Assign SDO1 to Pin RP3 = RD10
	//**********************************************
	//RPOR1bits.RP3R = 12; 		//.12. represents function SS2 out - SC void for debug
}

//************************************************************
// Function PPS_accmtr_config()
// map the interrupt pin received from the accelerometer
//************************************************************
void PPS_accmtr_config(void){	

#if defined (EXPLORER16)
	// Assign External Interrupt 1 to Pin RP30 = RF2	
	RPINR1bits.INT2R = 30;	 //.30 represents RP30	
#elif defined (WISTONE_BOARD)
	// Assign External Interrupt 1 to Pin RP13 = RB2	
	RPINR1bits.INT2R = 13;	 //.13 represents RP13	
#endif // #if defined (EXPLORER16)
}

//************************************************************
// Function PPS_ads1282_config()
// map the interrupt pin for DRDY received from the ADS1282
//************************************************************
void PPS_ads1282_config(void){	

#if defined (EXPLORER16)
#elif defined (WISTONE_BOARD)
	// Assign External Interrupt 3 to Pin RP12
	RPINR1bits.INT3R = 12;	 // 12 represents RP12	
#endif // #if defined (EXPLORER16)
}

//************************************************************
// Function led_and_switch_init()
// map the LEDs and switches for the Explorer board
//************************************************************
void led_and_switch_init(void){
	// set I/O ports
#if defined (EXPLORER16) //YL 12.8 - added the ifdef on button trises (?)
	BUTTON_1_TRIS = 1;
	BUTTON_2_TRIS = 1;
	BUTTON_3_TRIS = 1;
	BUTTON_4_TRIS = 1;
	LED_1_TRIS = 0;
	LED_2_TRIS = 0;   
	// initial LED status:
	LED_1 = 1;
	LED_2 = 0;
#endif // #if defined (EXPLORER16)
	return;
}
