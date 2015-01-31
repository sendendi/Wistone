#ifndef __LCD_H__	
#define __LCD_H__

#include "GenericTypeDefs.h"

/***** DEFINE: ****************************************************************/
#define Y_DIM 4					// LCD Y dimension  
#define X_DIM 16 				// LCD X dimension
#define SCREEN_TIME 30

/***** FUNCTION PROTOTYPES: ***************************************************/
void init_lcd(void);			 
void print_string(BYTE x, BYTE y, char* string);
void refresh_screen(void);
int  clear_screen(void);
int  handle_lcd(int sub_cmd);

#endif //__LCD_H__
