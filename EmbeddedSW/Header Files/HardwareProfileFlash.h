/******************************************************************************
 *
 *                Microchip Memory Disk Drive File System
 *
 ******************************************************************************
 * FileName:        HardwareProfileFlash.h
 * Dependencies:    None
 * Processor:       PIC18/PIC24/dsPIC30/dsPIC33/PIC32
 * Compiler:        C18/C30/C32
*****************************************************************************/


#ifndef _HARDWAREPROFILE_FLASH_H_
#define _HARDWAREPROFILE_FLASH_H_

// Define your clock speed here
//#define GetSystemClock()        32000000
#define GetSystemClock()        20000000
#define GetPeripheralClock()    GetSystemClock()
#define GetInstructionClock()   (GetSystemClock() / 2)

// Clock values
#define MILLISECONDS_PER_TICK       10                      // Definition for use with a tick timer
#define TIMER_PRESCALER             TIMER_PRESCALER_8       // Definition for use with a tick timer
#define TIMER_PERIOD                20000                   // Definition for use with a tick timer


/*********************************************************************/
/******************* Pin and Register Definitions ********************/
/*********************************************************************/

/* SD Card definitions: Change these to fit your application when using
   an SD-card-based physical layer                                   */


// Description: SD-SPI Chip Select Output bit
#define SD_CS				LATDbits.LATD2				
// Description: SD-SPI Chip Select TRIS bit
#define SD_CS_TRIS          TRISDbits.TRISD2 		

// Description: SD-SPI Card Detect Input bit
#define SD_CD               PORTFbits.RF0 				
// Description: SD-SPI Card Detect TRIS bit
#define SD_CD_TRIS          TRISFbits.TRISF0 	

// Registers for the SPI module you want to use

// Description: The main SPI control register
#define SPICON1             SPI1CON1
// Description: The SPI status register
#define SPISTAT             SPI1STAT
// Description: The SPI Buffer
#define SPIBUF              SPI1BUF
// Description: The receive buffer full bit in the SPI status register
#define SPISTAT_RBF         SPI1STATbits.SPIRBF
// Description: The bitwise define for the SPI control register (i.e. _____bits)
#define SPICON1bits         SPI1CON1bits
// Description: The bitwise define for the SPI status register (i.e. _____bits)
#define SPISTATbits         SPI1STATbits
// Description: The enable bit for the SPI module
#define SPIENABLE           SPISTATbits.SPIEN

// Tris pins for SCK/SDI/SDO lines

// Description: The TRIS bit for the SCK pin
#define SPICLOCK            TRISDbits.TRISD10		
// Description: The TRIS bit for the SDI pin
#define SPIIN               TRISFbits.TRISF13 		
// Description: The TRIS bit for the SDO pin
#define SPIOUT              TRISDbits.TRISD9		

// Will generate an error if the clock speed is too low to interface to the card
#if (GetSystemClock() < 100000)
   #error Clock speed must exceed 100 kHz
#endif    

#endif // #ifndef _HARDWAREPROFILE_FLASH_H_
