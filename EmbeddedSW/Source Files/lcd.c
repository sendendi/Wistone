/*******************************************************************************

lcd.c - low level commands for LCD panel
========================================

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
- the LCD is 16 characters by 2 rows
- 4-bit(nibble) interface
- initialization sequence and write operation are based on NOVATEK NT7603 datasheet

using: Tianma Tm162jcawg1

********************************************************************************
This file contains functions for handling the LCD, using SW configurations (not PMP).
Data Structures Description:
In total there are 2 groups of 2 shadow registers (each register is 1x16 char matrix), 
used in double buffering manner: each pair of registers displays single screen content 
(32 chars) for predefined period of time. These registers are continuously updated by 
clear_screen\print_string functions and the new content is displayed by refresh_screen(),
that is activated on each wistone_main.c loop iterartion. 
*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include "command.h"			//Application
#include "error.h"				//Application
#include "parser.h"				//Application
#include "HardwareProfile.h" 	//Common
#include "misc_c.h"				//Common
#include "p24FJ256GB110.h"		//Common
#include "lcd.h"				//Devices
#include "TimeDelay.h"			//TxRx - Common

/***** DEFINE: ****************************************************************/
typedef enum { _CTRL = 0, _DATA = 1} commandType;	// enums for LCD_Write_Nibble

/***** GLOBAL VARIABLES: ******************************************************/
BYTE   g_screen[Y_DIM][X_DIM];	// shadow memory for screen that will copied to LCD.	

/***** INTERNAL PROTOTYPES: ***************************************************/
void write_nibble_lcd (BYTE data, commandType cmd);

/*******************************************************************************
// write_nibble_lcd()
// write 4 bits of data or control (depends on cmd param) to the LCD
// when byte is written - 4 upper (most significant) bites are written as first nibble,
// 4 lower - as second one  (based on NOVATEK NT7603 write operation waveform)
*******************************************************************************/
void write_nibble_lcd (BYTE data, commandType cmd)
{
    mLCD_RS = cmd;					// cmd = 0 for control write, and 1 for data write
//  mLCD_R_W = 0;					// no need, since always "0"
	mLCD_D4 =  data       % 2; 		// lower 4 bits
	mLCD_D5 = (data >> 1) % 2;		
	mLCD_D6 = (data >> 2) % 2;
	mLCD_D7 = (data >> 3) % 2;		
    mLCD_E  = 1;					// toggle E
	mLCD_E  = 1; 					// repeat to allow 300nSec min Elable high width
    mLCD_E  = 0; 
//  no need for delay, since in main loop we process single nibble operation each time. this will give sufficient delay.
}

/*******************************************************************************
// init_lcd()
// - configure LCD pins:
// 		- E 	- RD4  - output
//		- R/W	- RD5  - output
//		- RS	- RB15 - output
//		- DB4	- RE4  - output
//		- DB5	- RE5  - output
//		- DB6	- RE6  - output
//		- DB7	- RE7  - output
// - initialize the LCD according to NOVATEK NT7603 initialization sequence. 
// Input - none, Output - none
*******************************************************************************/
 void init_lcd(void)
{
	// configure all relevant pins to output
	// then, set them to zero.
	// use &, to refrain from changing other pins
	LATD &=  0b1111111111001111;
	TRISD &= 0b1111111111001111;
	LATE &=  0b1111111100001111;
	TRISE &= 0b1111111100001111;
	LATB &=  0b0111111111111111;
	TRISB &= 0b0111111111111111;
	
	write_nibble_lcd(0x2, _CTRL); 	// set to 4-bit interface mode
	write_nibble_lcd(0x2, _CTRL); 	// set to 4-bit interface mode
	write_nibble_lcd(0x8, _CTRL); 	// 2-line mode, 5x7 pixels per character
	DelayMs(1);						// need to wait at least 40uSec
	write_nibble_lcd(0x0, _CTRL);	
	write_nibble_lcd(0xC, _CTRL);	// display=on, cursor=off,  blink=off
	DelayMs(1);						// need to wait at least 40uSec
	write_nibble_lcd(0x0, _CTRL);	// clear display
	write_nibble_lcd(0x1, _CTRL);
	DelayMs(5);						// need to wait at least 1.6 mSec
	write_nibble_lcd(0x0, _CTRL);	
	write_nibble_lcd(0x6, _CTRL);	// increment mode, entire shift off
	DelayMs(1);
	clear_screen(); 				// clear the screen (clear 4 shadow registers) 
}

/*******************************************************************************
// clear_screen()
// clear all shadow registers for the screen by writing space to each cell.
// Input - none, Output - 0 for success (to keep handling function format YL 10.5)
*******************************************************************************/
int clear_screen(void)
{
	UINT8 i,j;

	//the screen is 4x16 char matrix (global main variable)
	for (i=0;i<Y_DIM;i++)
		for (j=0;j<X_DIM;j++)
			g_screen[i][j]=0x20;	 // fill with blanks
	return 0;
}

/*******************************************************************************
// print_string() 
// print a string in the screen, at a given position.
// Input - the string and position (starting at 0,0) on the screen to display 
// 		   the string on;
// Output - none;
// The actual print is done to the shadow register (screen array) only.
// It will be automatically updated later to LCD, using refresh_screen().
*******************************************************************************/
void print_string(BYTE x, BYTE y, char* string)
{
	BYTE i;
	BYTE len;

	len = strlen_ws(string);
	// make sure coordination is inside active screen
	if (x > 15) 
		x = 15;
	if (y > 3)
		y = 3;
	// make sure string length will not get out of the line
	if (x + len > X_DIM)
		len = X_DIM - x;
	for(i = x; i < (x + len); i++){
		g_screen[y][i] = string[i - x];
	}
}

/*******************************************************************************
// refresh_screen()
// displays the screen matrix (= 4 shadow registers) content on LCD.
// Input - none, Output - none;
// refresh_screen is activated once each wistone_main.c loop iteration to display 
// a single nibble each time. In order to display the screen matrix, it toggles 
// between 2 matrix halves (active_screen var):
// first 2 rows (i.e 2 shadow registers) are the first screen to display,
// and the last 2 rows are the second one.
// The function works like a state machine:
// in total there are 68 states in order to display a single screen (2x16 chars);
// after SCREEN_TIME period the second screen is displayed for SCREEN_TIME period;
//  then the first screen is displayed again, and so on. 
// The states 0 and 1 are control states for the first screen row (16 chars)
// and the states 34 and 35 are control for the second screen row. 
// These states move the cursor to desired position.
// The display states are 2-33 for the first screen row, and 36-67 for the second one.
// The char to be displayed is divided in 2 nibbles:
// each even state displays the upper char part, 
// and the following odd state displays it's lower part.
*******************************************************************************/
void refresh_screen() 
{
	static BYTE state=0;
	static BYTE screen_delay=0; 		// counts the time for displaying a single screen (2 rows of 16 chars)
	static BYTE active_screen = 0; 	// the part of the screen matrix to display -
										// 0 for the first screen, 1 for the second screen.

	if (state==0)			write_nibble_lcd(0x8, _CTRL); 		
	else if (state==1)		write_nibble_lcd(0x0, _CTRL);		// the states 0 and 1 both move the cursor to the beginning of the upper LCD line 
																// using DD RAM address set instruction : DB7=1, DB6..DB0=0 (instruction length is 8 bits, hence it is written to by using 2 nibbles)
	else if ((state>1) && (state<34)){							// print to upper LCD line
		if ((state%2)==0)
			write_nibble_lcd(g_screen[active_screen*2][(state-2)/2]/16, _DATA);	// upper nibble
		else
			write_nibble_lcd(g_screen[active_screen*2][(state-2)/2]%16, _DATA);	// lower nibble
		}
	else if (state==34)		write_nibble_lcd(0xA, _CTRL);		
	else if (state==35)		write_nibble_lcd(0x8, _CTRL);		// the states 34 and 35 both move the cursor to the beginning of the lower LCD line (address 0x28)
	else if ((state>35) && (state<68)){							// print to lower LCD line
		if ((state%2)==0) 
			write_nibble_lcd(g_screen[active_screen*2+1][((state-2)-34)/2]/16, _DATA); //((state-2)-34) in order to refer to 0..15 coloumns in screen matrix.
		else
			write_nibble_lcd(g_screen[active_screen*2+1][((state-2)-34)/2]%16, _DATA); //((state-2)-34) in order to refer to 0..15 coloumns in screen matrix.
		}
	state++;
	if (state==68) {
		state=0;
		screen_delay++;
	}
	if (screen_delay==SCREEN_TIME) {			// the same screen is displayed over and over again until SCREEN_TIME period is over.
		active_screen = (active_screen + 1)%2;	// toggle between 2 screens after SCREEN_TIME period is over.						
		screen_delay = 0;
		}
}

/*******************************************************************************
// handle_lcd()
// if first token was "lcd", then handle LCD commands message:
// - according to sub command, call the relevant function
// - if command failed, display error message, and return error code
// - if command executed OK, display OK message and return 0.
*******************************************************************************/
int handle_lcd(int sub_cmd)	//YL 10.5 added for clear screen
{
	int res = 0;

	// dispatch to the relevant sub command function according to sub command
	switch (sub_cmd) {
		case SUB_CMD_CLRSCR:
			res = clear_screen();
			break;

		default:
			err(ERR_INVALID_SUB_CMD);
			cmd_error(0);
			break;
	}
	if (res < 0)
		return cmd_error(0);

	cmd_ok();
	return 0;
}
