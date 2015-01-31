#ifndef __TEMP_H__	
#define __TEMP_H__

#include "GenericTypeDefs.h"

/***** DEFINE: ****************************************************************/
#define TEMP_REG	0x00
#define CONFIG_REG 	0x01
#define TEMP_ADD 	0b1001000 // sensor slave address (without r/w bit) - //schematics: A2, A1 and A0 are all 0

typedef struct {
	char integer_part;
	BYTE fractional_part;
} Temperature;

/***** FUNCTION PROTOTYPES: ***************************************************/
int init_temp_sensor(void);
int get_temp(Temperature* temp);
int handle_temp(int sub_cmd);

#endif //__TEMP_H__
