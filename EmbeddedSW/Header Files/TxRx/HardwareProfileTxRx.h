/********************************************************************
* FileName:		HardwareProfile.h
* Dependencies:    
* Processor:	PIC18, PIC24, PIC32, dsPIC30, dsPIC33
*               tested with 18F4620, dsPIC33FJ256GP710	
* Complier:     Microchip C18 v3.04 or higher
*				Microchip C30 v2.03 or higher	
*               Microchip C32 v1.02 or higher	
* Company:		Microchip Technology, Inc.
*
* Copyright and Disclaimer Notice
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
*********************************************************************
* File Description:
*
*  This file defines functions used for demo board hardware
*
* Change History:
*  Rev   Date         Author    Description
*  1.0   2/15/2009    yfy       Initial revision
*  2.0   4/15/2009    yfy       MiMAC and MiApp revision
********************************************************************/

#ifndef _HARDWARE_PROFILE_TXRX_H
    #define _HARDWARE_PROFILE_TXRX_H
    
    #include "GenericTypeDefs.h"
    #include "ConfigApp.h"
	#include "TxRx.h"
	#include "wistone_main.h"
        
    
    #if defined(EXPLORER16) 

   // Transceiver Configuration

		#if defined (MRF49XA)
    		#define RF_INT_PIN      	PORTEbits.RE8   
            #define RF_INT_TRIS     	TRISEbits.TRISE8
	
			#define PHY_CS              LATBbits.LATB2
       		#define PHY_CS_TRIS         TRISBbits.TRISB2
       		//#define PHY_RESETn        LATFbits.LATF3
       		//#define PHY_RESETn_TRIS   TRISFbits.TRISF3

	        #define SPI_SDI             PORTDbits.RD14		// RF7 => RD14
	        #define SDI_TRIS            TRISDbits.TRISD14
	        #define SPI_SDO             LATFbits.LATF8 
	        #define SDO_TRIS            TRISFbits.TRISF8
	        #define SPI_SCK             LATBbits.LATB0 		// RF6 => RB0
	        #define SCK_TRIS            TRISBbits.TRISB0

	        #define nFSEL           	LATBbits.LATB1              
	        #define nFSEL_TRIS      	TRISBbits.TRISB1
	        #define FINT		    	PORTEbits.RE9 
	        #define FINT_TRIS       	TRISEbits.TRISE9

		#elif defined (MRF24J40)
			#define RF_INT_PIN      	PORTDbits.RD2   	//  INT is connected to RD2
            #define RF_INT_TRIS     	TRISDbits.TRISD2

     		#define PHY_CS              LATDbits.LATD10		// CS is connected to RD10
     		#define PHY_CS_TRIS         TRISDbits.TRISD10
     		#define PHY_RESETn          LATDbits.LATD0		// RESET is connected to RD0
     		#define PHY_RESETn_TRIS     TRISDbits.TRISD0
	

			#define SPI_SDI             PORTDbits.RD9		// SDI is connected to RD9
       		#define SDI_TRIS            TRISDbits.TRISD9
       		#define SPI_SDO             LATDbits.LATD3 		// SDO is connected to RD3		
       		#define SDO_TRIS            TRISDbits.TRISD3
       		#define SPI_SCK             LATDbits.LATD8 		// SCK is connected to RD8
       		#define SCK_TRIS            TRISDbits.TRISD8

         	#define PHY_WAKE        	LATDbits.LATD1		// WAKE is connected to RD1
         	#define PHY_WAKE_TRIS   	TRISDbits.TRISD1

		#endif


	#elif defined (WISTONE_BOARD)

		#if defined (MRF49XA)
    		#define RF_INT_PIN      	PORTBbits.RB15 		//SC changed to RB15 (HW Bug:INT PIN is named RESET), was PORTEbits.RE8   
            #define RF_INT_TRIS     	TRISBbits.TRISB15 	//SC changed to RB15, was TRISEbits.TRISE8
	
			#define PHY_CS              LATAbits.LATA4  	//SC changed to RA4, was LATBbits.LATB2
       		#define PHY_CS_TRIS         TRISAbits.TRISA4  	//SC changed to RA4, was TRISBbits.TRISB2
       		//#define PHY_RESETn        LATBbits.LATB15  	//SC changed to RB15, was LATFbits.LATF3
       		//#define PHY_RESETn_TRIS   TRISBbits.TRISB15 	//SC changed to RB15, was TRISFbits.TRISF3

	        #define SPI_SDI             PORTGbits.RG9		//SC changed to RG9, was PORTDbits.RD14	// RF7 => RD14
	        #define SDI_TRIS            TRISGbits.TRISG9   	//SC changed to RG9, was TRISDbits.TRISD14
	        #define SPI_SDO             LATFbits.LATF3     	//SC changed to RF3, was LATFbits.LATF8 
	        #define SDO_TRIS            TRISFbits.TRISF3   	//SC changed to RF3, was TRISFbits.TRISF8
	        #define SPI_SCK             LATBbits.LATB5 		//SC changed to RB5, was LATBbits.LATB0 		// RF6 => RB0
	        #define SCK_TRIS            TRISBbits.TRISB5    //SC changed to RB5, was TRISBbits.TRISB0

	        #define nFSEL           	LATDbits.LATD5      //SC changed to RD5, was LATBbits.LATB1              
	        #define nFSEL_TRIS      	TRISDbits.TRISD5    //SC changed to RD5, was TRISBbits.TRISB1
	        #define FINT		    	PORTGbits.RG0       //SC changed to RG0, was PORTEbits.RE9 
	        #define FINT_TRIS       	TRISGbits.TRISG0    //SC changed to RG0, was TRISEbits.TRISE9

		#elif defined (MRF24J40)
			#define RF_INT_PIN      	PORTDbits.RD2   	// INT is connected to RD2
            #define RF_INT_TRIS     	TRISDbits.TRISD2

     		//#define PHY_CS            LATDbits.LATD10	// CS is connected to RD10 - SC void for debug
     		//#define PHY_CS_TRIS       TRISDbits.TRISD10
     		#define PHY_RESETn          LATDbits.LATD0		// RESET is connected to RD0
     		#define PHY_RESETn_TRIS     TRISDbits.TRISD0
	

			//#define SPI_SDI           PORTDbits.RD9		// SDI is connected to RD9 - SC void for debug
       		//#define SDI_TRIS          TRISDbits.TRISD9
       		#define SPI_SDO             LATDbits.LATD3 		// SDO is connected to RD3		
       		#define SDO_TRIS            TRISDbits.TRISD3
       		#define SPI_SCK             LATDbits.LATD8 		// SCK is connected to RD8
       		#define SCK_TRIS            TRISDbits.TRISD8

         	#define PHY_WAKE        	LATDbits.LATD1		// WAKE is connected to RD1
         	#define PHY_WAKE_TRIS   	TRISDbits.TRISD1

		#endif
	#else
		#error "You need to pick a board - either EXPLORER16 or WISTONE_BOARD"
	#endif



      #if !defined(MRF89XA)
          #define RFIF            IFS1bits.INT1IF
          #define RFIE            IEC1bits.INT1IE
      #endif

      #define CLOCK_FREQ      20000000 //YL 11.8 was 32000000

      #define TMRL TMR2
      
      //#define USE_EXTERNAL_EEPROM
             

   
    // Following definitions are used for LCD display on the demo board
    #if defined(EXPLORER16)
    
    	#define LCD_DATA0_TRIS		(TRISEbits.TRISE0)		// Multiplexed with LED6
    	#define LCD_DATA0_IO		(LATEbits.LATE0)
    	#define LCD_DATA1_TRIS		(TRISEbits.TRISE1)
    	#define LCD_DATA1_IO		(LATEbits.LATE1)
    	#define LCD_DATA2_TRIS		(TRISEbits.TRISE2)
    	#define LCD_DATA2_IO		(LATEbits.LATE2)
    	#define LCD_DATA3_TRIS		(TRISEbits.TRISE3)		// Multiplexed with LED3
    	#define LCD_DATA3_IO		(LATEbits.LATE3)
    	#define LCD_DATA4_TRIS		(TRISEbits.TRISE4)		// Multiplexed with LED2
    	#define LCD_DATA4_IO		(LATEbits.LATE4)
    	#define LCD_DATA5_TRIS		(TRISEbits.TRISE5)
    	#define LCD_DATA5_IO		(LATEbits.LATE5)
    	#define LCD_DATA6_TRIS		(TRISEbits.TRISE6)
    	#define LCD_DATA6_IO		(LATEbits.LATE6)
    	#define LCD_DATA7_TRIS		(TRISEbits.TRISE7)
    	#define LCD_DATA7_IO		(LATEbits.LATE7)
    	#define LCD_RD_WR_TRIS		(TRISDbits.TRISD5)
    	#define LCD_RD_WR_IO		(LATDbits.LATD5)
    	#define LCD_RS_TRIS			(TRISBbits.TRISB15)
    	#define LCD_RS_IO			(LATBbits.LATB15)
    	#define LCD_E_TRIS			(TRISDbits.TRISD4)
    	#define LCD_E_IO			(LATDbits.LATD4)


        #define PUSH_BUTTON_1       PORTDbits.RD6
        #define PUSH_BUTTON_2       PORTDbits.RD7
        #define PUSH_BUTTON_3       PORTAbits.RA7
        #define PUSH_BUTTON_4       PORTDbits.RD13
        #define LED_1               LATAbits.LATA6
        #define LED_2               LATAbits.LATA5
        
        #define BUTTON_1_TRIS       TRISDbits.TRISD6
        #define BUTTON_2_TRIS       TRISDbits.TRISD7
        #define BUTTON_3_TRIS       TRISAbits.TRISA7
        #define BUTTON_4_TRIS       TRISDbits.TRISD13
        #define LED_1_TRIS          TRISAbits.TRISA6
        #define LED_2_TRIS          TRISAbits.TRISA5
        
        // Define SUPPORT_TWO_SPI if external EEPROM use the second SPI
        // port alone, not sharing SPI port with the transceiver
        //#define SUPPORT_TWO_SPI
        
        // External EEPROM SPI chip select pin definition
        #define EE_nCS_TRIS         TRISDbits.TRISD12
        #define EE_nCS              LATDbits.LATD12
   
    #endif
    
    // Following definition is for delay functionality
    #if defined(__18CXX)
        #define GetInstructionClock()	(CLOCK_FREQ/4)
    #elif defined(__C30__) 
        #define GetInstructionClock()	(CLOCK_FREQ/2)
    #elif defined(__PIC32MX__)
        #define GetInstructionClock()	(CLOCK_FREQ)
    #endif

    void led_and_switch_init(); 

 
    
#endif
