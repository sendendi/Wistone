#ifndef __MISC_C_H__	
#define __MISC_C_H__

#include "GenericTypeDefs.h"

/***** FUNCTION PROTOTYPES: ***************************************************/
unsigned int 	strlen_ws(const char* str);
char*			strcpy_ws(char* str_to, const char* str_from);
BOOL			strcmp_ws(const char* str1, const char* str2);
BYTE 			long_to_byte(long num);
BYTE 			int_to_byte(int num);	
char*			long_to_str(long num); 	
char*			uint_to_str(unsigned int num); 	
char* 			char_to_str(char num);  
char*			byte_to_str(BYTE num); 	

#endif //#ifndef __MISC_C_H__	
