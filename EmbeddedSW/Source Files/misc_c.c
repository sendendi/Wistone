/*******************************************************************************

wistone_c.c - standard C functions' implementations
===================================================

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
This file contains several standard C functions' implementations adjusted to
WISDOM_STONE project purposes. These functions reflect the functions that appear
in the following standard C libraries:
- string.h
- ...
 
*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include <string.h>
#include "error.h"			//Application
#include "misc_c.h"			//Common

/*******************************************************************************
// strlen_ws()
// calculates string length excluding terminal '\0'
*******************************************************************************/
unsigned int strlen_ws(const char * str) 
{
	unsigned int i = 0;
	
	if (!str)
		return NULL;
		
	while (str[i] != '\0')
		i++;
	
	return i;
}

/*******************************************************************************
// strcpy_ws()
// copies the contents of given string to destination string
*******************************************************************************/
char* strcpy_ws(char* str_to, const char* str_from)
{
    int i;
		
	if (!str_to || !str_from)
		return NULL;
	
	for(i = 0; str_from[i] != '\0'; ++i)
		str_to[i] = str_from[i];
	str_to[i] = str_from[i];	//append '\0'

	return str_to;
}

/*******************************************************************************
// strcmp_ws()
// compares 2 strings and returns TRUE if the strings are identical
*******************************************************************************/
BOOL strcmp_ws(const char* str1, const char* str2) //YL 19.9
{
	while ((*str1) && (*str2)) {
		if (*str1 != *str2)
			return FALSE;
		str1++;	
		str2++;
	}
	if (strlen_ws(str1) || strlen(str2))
		return FALSE;
	return TRUE;
}

/*******************************************************************************
// long_to_byte()
*******************************************************************************/
BYTE long_to_byte(long num) 
{
	if (num >= 0)
		return(num & 0x000000FF);
	else
		err(ERR_INVALID_NUM);
	return 0;
}


/*******************************************************************************
// int_to_byte()
*******************************************************************************/
BYTE int_to_byte(int num) 	//YL 27.8 //NOTE positive/negative
{			
	return(num & 0x00FF);
}

/*******************************************************************************
// long_to_str()
// convert signed long number(32 bits) to string 
*******************************************************************************/
char *long_to_str(long num)
{
	static char buf[12];	//MAX_NUM is 2147483647 - 10 digits; another 2 chars - for sign and null.
	char *p;
	long sign = num;
	
	buf[11] = 0;			//end of string
	p = buf + 11;
	do {
		p--;
		*p = '0' + num % 10;
		num /= 10;
	} while ((num) && (p > buf));
	if (sign < 0){
		p--;
		*p = '-';
	}
	
	return p;
}

/*******************************************************************************
// uint_to_str()
// convert unsigned int number(16 bits) to string 
*******************************************************************************/
char* uint_to_str(unsigned int num)  //YL 22.8
{
	return long_to_str((long)(0x0000FFFF & num));
}

/*******************************************************************************
// char_to_str()
// convert signed char number(8 bits) to string 
*******************************************************************************/
char* char_to_str(char num)  //YL 30.8
{
	return long_to_str(0x000000FF & num);
}

/*******************************************************************************
// byte_to_str()
// convert unsigned char number(8 bits) to string 
*******************************************************************************/

char* byte_to_str(BYTE num)
{
	static char buf[4];	//MAX_NUM is 255 - 3 digits; another char is for null.
	char *p;
		
	buf[3] = 0;	//'\0'?
	p = buf + 3;
	while ((num != 0) && (p >= buf)) {
		p--;
		*p = '0' + num % 10;
		//debug...
		// m_write("p: ");
		// m_write(p);
		// write_eol();
		//...debug
		num /= 10;
	}
	//debug...
	// m_write("byte_num_to_str: ");
	// m_write(p);
	// write_eol();
	//...debug
	return p;
}
