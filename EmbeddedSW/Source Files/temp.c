/*******************************************************************************

temp.c - handle read temperature 
================================

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
this file contains functions for operating the temperature sensor.
- initialize the sensor
- READ temperature in Celsius
using SW implemented I2C protocol.

using temperature sensor TMP75, Texas Instruments
*******************************************************************************/
#include "wistone_main.h"
#ifdef WISDOM_STONE

/***** INCLUDE FILES: *********************************************************/
#include "command.h"		//Application
#include "error.h"			//Application
#include "parser.h"			//Application
#include "misc_c.h"			//Common
#include "temp.h"			//Devices
#include "i2c.h"			//Protocols

/***** INTERNAL PROTOTYPES: ***************************************************/
static void f_write(BYTE num);	//YL 29.8 for formatted printing of fractional temperature part - 2 decimal digits only

/*******************************************************************************
// init_temp_sensor()
// initialize temperature sensor by shutting it down untill get_temp command
// input - none, output - error code if any
*******************************************************************************/
int init_temp_sensor(void)
{
	BYTE config_reg_address[2] = {CONFIG_REG, 0b00000001};		// D0 bit is set to 1 to shut the sensor down (for low power consumption).							
	if (device_write_i2c_ert(TEMP_ADD, 2, config_reg_address, I2C_WRITE) != 0)
		return ERR_TEMP_INIT;
	return ERR_NONE;
}

/*******************************************************************************
// get_temp()
// read the temperature from the sensor (in Celsius)
// input - none, 
// output - temperature in 2 parts - integer and fractional, error code if any
// the temperature is read from the temperature register which holds the 12 bit signed data in 2-s complement format, in 2 bytes:
// the zero (msb) byte - holds integer temperature part in D8,...,D15
// the first byte - holds fractional temperature part in D4,..., D7
// sensor temperature range: [-40, +125] Celsius
*******************************************************************************/
int get_temp(Temperature* temp)
{
	// the sensor stays in shut down mode (D0 = 1), but since D7 is set to 1 - the device will start single temperature conversion
	// and will return to shut down state after it is completed. i.e there is no continuos temperature monitoring (for low power consumption)
	// in addition D5 and D6 are both set to 1 for maximal resolution conversion of 0.0625 Celsius (i.e. conversion of all 12 data bits)
	BYTE 	config_reg_address[2] = {CONFIG_REG, 0b11100001};	// configuration register address + register setting data 
	BYTE 	read_data[2];		// for reading temperature
	char 	sign = 1;			// default sign of temperature value is positive
	BYTE	temp_reg_address = TEMP_REG; // temperature register address			
	
	// start single temperature conversion:
	if (device_write_i2c_ert(TEMP_ADD, 2, config_reg_address, I2C_WRITE) != 0)
		return ERR_GET_TEMP;
	// first, set the pointer to temperature register, then read the temperature:
	if (device_write_i2c_ert(TEMP_ADD, 1, &temp_reg_address, I2C_READ) != 0)
		return ERR_GET_TEMP;
	if (device_read_i2c_ert(TEMP_ADD, 2, read_data, I2C_READ) != 0)
		return ERR_GET_TEMP;
	// temperature register stores the data in D4-D15 bits in 2-s complement format.
	// extract temperature sign:
	if (read_data[0] & 0b10000000)
		sign = -1;
	// D4, ..., D7 store the fractional part of the temperature. - the fractional part is rounded to 2 decimal digits
	temp->fractional_part =  (BYTE) (read_data[1]/2.56); // same as: (uchar)((1 / 16)* (read_data[1] / 16) * 100)
	// remove the sign (bit7 in the zero byte), and get the integer temperature part:
	if (sign == 1) // positive value	
		temp->integer_part = (read_data[0] & 0b0111111);
	else // negative value
		temp->integer_part = 1 + ~(read_data[0] & 0b0111111);	
	return ERR_NONE; // the device shut itself down once single temperature conversion is completed
}

/*******************************************************************************
// f_write()
// prints fractional temperature part in 2 digits after the point format 
*******************************************************************************/
static void f_write(BYTE num)
{
	if (num > 99)				// max BYTE num is 255
		num /= 10; 				// extract max 2 upper digits
	if (num == 0) {
		m_write("00");			// output '0' as 00
	} else if (num < 10) {
		m_write("0");			// add leading zero before 1 decimal digit number
		m_write(byte_to_str(num));
	} else {
		m_write(byte_to_str(num));
	}	
}

/*******************************************************************************
// handle_temp()
// if first token was "temp", then handle TEMP commands message:
// - according to sub command, call the relevant function
// - if command failed, display error message, and return error code
// - if command executed OK, display OK message and return 0.
*******************************************************************************/
int handle_temp(int sub_cmd)
{
	int res = 0;
	Temperature temp;

	// dispatch to the relevant sub command function according to sub command
	switch (sub_cmd) {
		case SUB_CMD_GTEMP:
			res = get_temp(&temp);
			m_write("Wistone TEMP: ");		//YL 11.8 instead sprintf 
			m_write(char_to_str(temp.integer_part));
			m_write(".");
			f_write(temp.fractional_part); 
			m_write(" Celsius");
			write_eol();
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
#endif //#ifdef WISDOM_STONE
 