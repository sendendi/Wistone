/*******************************************************************************

i2c.c - low level commands for I2C handling
===========================================

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
this file contains functions for handling the I2C bus.
- read / write bytes
using:
- SW implemented I2C protocol 
- HW implemented I2C protocol, using:
		- I2C1 for accelerometer
		- I2C2 for EEPROM, RTC and Temperature sensor
Accelerometer is used only in HW mode
EEPROM, RTC and Temperature sensor may be used in SW or HW mode
*******************************************************************************/
#include "wistone_main.h"
//#ifdef WISDOM_STONE //YL 23.4 commented to let the plug use the eeprom too

/***** INCLUDE FILES: *********************************************************/
#include "ports.h"
#include "command.h"			//Application
#include "error.h"				//Application
#include "HardwareProfile.h"	//Common
#include "i2c.h"				//Protocols

/***** INTERNAL PROTOTYPES: ***************************************************/
//#ifndef HARDWARE_I2C_ERT
void 	delay_i2c(void);
void 	data_out(void);
void 	data_in(void);
void 	set_clock(int x);
void 	set_data(int x);
int 	get_data(void);
void 	reset_i2c(void);
//#endif // ifndef HARDWARE_I2C_ERT

/*******************************************************************************
// init_i2c()
// I2C HW mode:
// - I2C1 - used for the mma8451 Accelerometer
// - I2C2 - used for EEPROM, RTC and Temperature sensor
// I2C SW mode:
// 	- set SCL and SDA directions, and release them (pull to "1").
// 	- then, reset the I2C bus
// HARDWARE_I2C mode:
// 	 - appropriate interrupt bit locations for I2C2 Master Events are the Flag IFS3<2> and Enable IEC3<2> (PIC24F family, p. 79) //YL 5.6 
// make sure that I2C2 module is disabled when calling init_i2c
*******************************************************************************/
void init_i2c(void)
{
	// I2C for the Accelerometer:
	I2C1CONbits.I2CEN = 1;	//I2CxCON<15>: enables the I2Cx module and configures the SDAx and SCLx pins as serial port pins; I2CxCON<10> is 0 by default, for 7-bit slave address
	I2C1BRG = 13;			//I2CxBRG: 667 KHz Baud rate, calculated according to Fosc = 20MHz, Fcy = 10MHz, Fscl = 667KHz, using formula: I2CxBRG = Fcy/Fscl - Fcy/10M - 1		
	// I2C for EEPROM, RTC, TEMP:
#if defined HARDWARE_I2C_ERT	
	I2C2CONbits.I2CEN = 1; 	//I2CxCON<15>: enables the I2Cx module and configures the SDAx and SCLx pins as serial port pins; I2CxCON<10> is 0 by default, for 7-bit slave address
	I2C2BRG = 23;			//I2CxBRG: 400 KHz Baud rate, calculated according to Fosc = 20MHz, Fcy = 10MHz, Fscl = 400KHz, using formula: I2CxBRG = Fcy/Fscl - Fcy/10M - 1		
#else
	// init for SW mode:
	set_clock(1);			// set SCL value to default (inactive)
	set_data(1); 
	I2C2_TRIS_SCL = 0;		// set SCL direction to output	
	data_in();				// set SDA to input
	reset_i2c();
#endif // #ifdef HARDWARE_I2C_ERT	
}

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//ACCELEROMETER:
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/*******************************************************************************
// start_i2c_accmtr()
// This function generates the start bit or repeated start bit of the I2C protocol.
// If the start bit is applied, then the SEN bit is set, otherwise,
// (meaning repeated start bit is applied), RSEN bit is set.
// Finally, the function waits for interrput indicating the start condition succeed.
*******************************************************************************/
inline __attribute__((always_inline)) void start_i2c_accmtr(int is_restart)
{
	if (is_restart) 
		I2C1CONbits.RSEN = 1; 		//I2CxCON<1>: initiate Repeated Start Condition on SDAx and SCLx pins; automatically cleared		
	else 
		I2C1CONbits.SEN = 1;		//I2CxCON<0>: initiates Start Event; automatically cleared
	while (!IFS1bits.MI2C1IF);		//IFS1<2>: MI2C1IF (master) interrupt is generated at completion of the Start Condition
	IFS1bits.MI2C1IF = 0;			
}

/*******************************************************************************
// stop_i2c_accmtr()
// This function generates the stop bit of the I2C protocol.
// To accomplish this, the PEN bit is set, and then the function waits for 
// interrput indicating the stop condition succeed.
*******************************************************************************/
inline __attribute__((always_inline)) void stop_i2c_accmtr(void)
{
	I2C1CONbits.PEN = 1; 		//I2CxCON<2>: enables generation of the master Stop Event; automatically cleared
	while (!IFS1bits.MI2C1IF);	//IFS1<2>: MI2C1IF (master) interrupt is generated at completion of the Stop Condition
	IFS1bits.MI2C1IF = 0;		
}

/*******************************************************************************
// out_byte_i2c_accmtr()
// This function send a byte through the I2C. 
// To do so, the byte is placed in I2C1TRN register, and then wait for interrupt
// indication the data sent succeed.
// Notice that the ack arrival we be dealt using the hardware.
*******************************************************************************/
inline __attribute__((always_inline)) int out_byte_i2c_accmtr(int dat)
{
	I2C1TRN = dat;				//I2CxTRN - transmit R/W register, gets the data byte to send out (only 0 - 7 bits are used)
	while (!IFS1bits.MI2C1IF);	//IFS1<2>: MI2C1IF (master) interrupt is generated at completion of the transmitting of data transfer byte
	IFS1bits.MI2C1IF = 0;
	if (I2C1STATbits.ACKSTAT) 	//check whether ACK was detected by I2C PIC HW	//BM 12.9
		return (-1);			//indicates failure
	return 0;					//indicates success
}

/*******************************************************************************
// in_byte_i2c_accmtr()
// This function receives a byte through the I2C. 
// To do so, the RCEN bit is set, and then it waits for interrupt 
// indication the data was recived. Afterwards an ack or nack is sent by setting
// or clearing the ACKDT bit, and then setting the ACKEN bit.
*******************************************************************************/
inline __attribute__((always_inline)) BYTE in_byte_i2c_accmtr(int is_last)
{	
	BYTE dat;	

	I2C1CONbits.RCEN = 1; 		//I2CxCON<3>: enables receive data from a slave device; automatically cleared; the data is shifted from I2CxRSR into I2CxRCV
	while (!IFS1bits.MI2C1IF);	//IFS1<2>: MI2C1IF (master) interrupt is generated at completion of the receiving of data transfer byte
	IFS1bits.MI2C1IF = 0;		
	dat = I2C1RCV;				//I2CxRCV - transmit RO register, gets the data to send in (only 0 - 7 bits are used)
	if (is_last) 
		I2C1CONbits.ACKDT = 1; 	//I2CxCON<5>: sets NACK
	else
		I2C1CONbits.ACKDT = 0; 	//I2CxCON<5>: sets ACK	
	I2C1CONbits.ACKEN = 1;	   	//I2CxCON<4>: enables sending the ACK/NACK
	while (!IFS1bits.MI2C1IF);	//IFS1<2>: MI2C1IF (master) interrupt is generated at completion of acknowledge transmit
	IFS1bits.MI2C1IF = 0;		
	return dat;
}

/*******************************************************************************
// device_write_i2c_accmtr() 
//  - write_before_read - 1 if the writing is done as a part of byte reading, 0 if only writing takes place 
*******************************************************************************/
int device_write_i2c_accmtr(BYTE address, int n, BYTE *data, BYTE write_before_read) //YL 15.8 write_before_read added; 15.9 changed void to int return type - to indicate success/failure
{
	int j;

	start_i2c_accmtr(0);
	if (out_byte_i2c_accmtr((address << 1) & 0xFE)) 		// send address + write (after address shift write bit (R/W = 0) is added)
		goto device_write_err_accmtr;
	for (j = 0; j < n; j++) { 
		if (out_byte_i2c_accmtr(data[j]))
			goto device_write_err_accmtr;
	}
	if (!write_before_read)
		stop_i2c_accmtr();		//writing is completed
	return 0;
device_write_err_accmtr:
		stop_i2c_accmtr();
	err(ERR_DEVICE_WRITE_I2C);
	return -1;	
}

/*******************************************************************************
// device_read_i2c_accmtr()
//  - write_before_read - 1 if the writing is done as a part of byte reading, 0 if only writing takes place //YL 22.8
*******************************************************************************/
int device_read_i2c_accmtr(BYTE address, int n, BYTE *data, BYTE write_before_read) //YL 15.8 write_before_read added; 15.9 changed void to int return type - to indicate success/failure
{
	int j;		

	start_i2c_accmtr(write_before_read); 			
	if (out_byte_i2c_accmtr((address << 1) | 0x01))	// send address + read (after address shift read bit (R/W = 1) is added)
		goto device_read_err_accmtr;
	for (j = 0; j < n-1; j++)
		data[j] = in_byte_i2c_accmtr(0); 			// read data bytes excluding last one (by sending 0 to in_byte_i2c); 
	data[n-1] = in_byte_i2c_accmtr(1);
	stop_i2c_accmtr();
	return 0;
device_read_err_accmtr:
	stop_i2c_accmtr();
	err(ERR_DEVICE_WRITE_I2C);
	return -1;
}

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//EEPROM, RTC, TEMP:
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/*******************************************************************************
// start_i2c_ert()
// in SW mode:
//  - generate the "start" bit:
//  - reset SDA while SCL is "0"
*******************************************************************************/
void start_i2c_ert(int is_restart) 
{
#ifdef HARDWARE_I2C_ERT
	if (is_restart) 
		I2C2CONbits.RSEN = 1; 		//I2CxCON<1>: initiate Repeated Start Condition on SDAx and SCLx pins; automatically cleared		
	else 
		I2C2CONbits.SEN = 1;		//I2CxCON<0>: initiates Start Event; automatically cleared
	while (!IFS3bits.MI2C2IF);		//IFS3<2>: MI2C2IF (master) interrupt is generated at completion of the Start Condition
	IFS3bits.MI2C2IF = 0;			
#else
	if (is_restart)
		set_clock(0);
	delay_i2c();
	set_data(1);
	data_out();
	delay_i2c();
	set_clock(1);
	set_data(0);
	delay_i2c();
#endif	// #ifdef HARDWARE_I2C_ERT
}

/*******************************************************************************
// stop_i2c_ert()
// in SW mode:
//  - generate the "stop" bit:
//  - set SDA after SCL is "1"
*******************************************************************************/
void stop_i2c_ert(void)
{
#ifdef HARDWARE_I2C_ERT
	I2C2CONbits.PEN = 1; 		//I2CxCON<2>: enables generation of the master Stop Event; automatically cleared
	while (!IFS3bits.MI2C2IF);	//IFS3<2>: MI2C2IF (master) interrupt is generated at completion of the Stop Condition
	IFS3bits.MI2C2IF = 0;		
#else
	set_clock(0);
	set_data(0);
	data_out();
	delay_i2c();
	set_clock(1);
	set_data(1);
	data_in();
#endif // #ifdef HARDWARE_I2C_ERT
}

/*******************************************************************************
// out_byte_i2c_ert()
// in SW mode:
//  - send single Byte to I2C bus
//  - check for positive acknowledgement from slave.
//  - if not Acked - return error
*******************************************************************************/
int out_byte_i2c_ert(int dat)
{
#ifdef HARDWARE_I2C_ERT
	I2C2TRN = dat;				//I2CxTRN - transmit R/W register, gets the data byte to send out (only 0 - 7 bits are used)
	while (!IFS3bits.MI2C2IF);	//IFS3<2>: MI2C2IF (master) interrupt is generated at completion of the transmitting of data transfer byte
	IFS3bits.MI2C2IF = 0;	
	return 0;					//indicates success
#else
	int i;
	data_out();
	for (i = 0; i < 8; i++) {
		set_clock(0);
		set_data(dat & 0x80);
		delay_i2c();
		set_clock(1);
		dat <<= 1;
		delay_i2c();
	}
	// generate 9th clock to read acknowledgement
	set_clock(0);
	data_in();
	i = get_data();
	delay_i2c();
	set_clock(1);
	delay_i2c();
	if (i)	// positive ACK = 0
		err(ERR_I2C_BUSY);
	return i;
#endif // #ifdef HARDWARE_I2C_ERT
}

/*******************************************************************************
// in_byte_i2c_ert()
// in SW mode:
//  - read single Byte from I2C bus
//  - in case of last READ byte transaction - set SDA to "1"
*******************************************************************************/
int in_byte_i2c_ert(int is_last)
{	
	int dat = 0;  
	
#ifdef HARDWARE_I2C_ERT
	I2C2CONbits.RCEN = 1; 		//I2CxCON<3>: enables receive data from a slave device; automatically cleared; the data is shifted from I2CxRSR into I2CxRCV
	while (!IFS3bits.MI2C2IF);	//IFS3<2>: MI2C2IF (master) interrupt is generated at completion of the receiving of data transfer byte
	IFS3bits.MI2C2IF = 0;		
	dat = I2C2RCV;				//I2CxRCV - transmit RO register, gets the data to send in (only 0 - 7 bits are used)
	if (is_last) 
		I2C2CONbits.ACKDT = 1; 	//I2CxCON<5>: sets NACK
	else
		I2C2CONbits.ACKDT = 0; 	//I2CxCON<5>: sets ACK	
	I2C2CONbits.ACKEN = 1;	   	//I2CxCON<4>: enables sending the ACK/NACK
	while (!IFS3bits.MI2C2IF);	//IFS3<2>: MI2C2IF (master) interrupt is generated at completion of acknowledge transmit
	IFS3bits.MI2C2IF = 0;		
#else
	int i;
	set_clock(0);
	data_in();
	delay_i2c();
	for (i = 0; i < 8; i++) {
		dat <<= 1;
		if (get_data())
			dat |= 1;
		set_clock(1);
		delay_i2c();
		set_clock(0);
		delay_i2c();
	}
	data_out();
	set_data(is_last); //PIC acks every accepted byte by sending 0 to EEPROM, accept for last read data byte.
	set_clock(1);
	delay_i2c();	
#endif // #ifdef HARDWARE_I2C_ERT
	return dat;
}

/*******************************************************************************
// device_write_i2c_ert()
// in SW mode:
// write transaction to I2C bus.
// input: 
//	- device address (7 bits, excluding the r/w bit)
//	- number of bytes to write
//	- pointer to the array of bytes
//  - write_before_read - 1 if the writing is done as a part of byte reading, 0 if only writing takes place //YL 22.8
// this function generates a start bit, then address + data bytes
// and then stop bit. if failed, it retries for I2C_MAX_RETRY times.
*******************************************************************************/
int device_write_i2c_ert(BYTE address, int n, BYTE *data, BYTE write_before_read) //YL 22.8 write_before_read 
{
	int i, j;
	
	for (i = 0; i < I2C_MAX_RETRY; i++) { 
		start_i2c_ert(0);
		if (out_byte_i2c_ert((address << 1) + 0)) 	// send address + write (after address shift write bit (R/W = 0) is added)
			goto device_write_err_ert;
		for (j = 0; j < n; j++) { 
			if (out_byte_i2c_ert(data[j])) 			// send data byte (first high and then low bytes of address, and then data bytes); EEPROM acks by 0 each written byte
				goto device_write_err_ert;
		}
		if (!write_before_read) //YL 25.8 
			stop_i2c_ert();
		return 0;
device_write_err_ert:
		stop_i2c_ert();
	}	
	err(ERR_DEVICE_WRITE_I2C);
	return -1;	
}

/*******************************************************************************
// device_read_i2c_ert()
// in SW mode:
// read transaction from I2C bus.
// input: 
//	- device address (7 bits, excluding the r/w bit)
//	- number of bytes to read
//	- pointer to array of bytes for the result
//  - write_before_read - 1 if the writing is done as a part of byte reading, 0 if only writing takes place //YL 22.8
// this function generates a start bit, then address + read the data bytes
// and then stop bit. if failed, it retries for I2C_MAX_RETRY times.
*******************************************************************************/
int device_read_i2c_ert(BYTE address, int n, BYTE *data, BYTE write_before_read) //YL 22.8 write_before_read added
{
	int i, j;
	
	for (i = 0; i < I2C_MAX_RETRY; i++) {
		start_i2c_ert(write_before_read);	//YL 25.8
		if (out_byte_i2c_ert((address << 1) + 1)) 	// send address + read (after address shift read bit (R/W = 1) is added)
			goto device_read_err_ert;
		for (j = 0; j < n - 1; j++) 
			data[j] = in_byte_i2c_ert(0); 			// read data bytes excluding last one (by sending 0 to in_byte_i2c); 
		data[n-1] = in_byte_i2c_ert(1);
		stop_i2c_ert();
		return 0;
device_read_err_ert:
		stop_i2c_ert();
	}
	err(ERR_DEVICE_READ_I2C);
	return -1;
}


//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
#ifndef HARDWARE_I2C_ERT
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

/*******************************************************************************
// data_out()
// set SDA direction to output
*******************************************************************************/
void data_out(void)	
{
	I2C2_TRIS_SDA = 0;	
}

/*******************************************************************************
// data_in()
// set SDA direction to input
*******************************************************************************/
void data_in(void) 
{
	I2C2_TRIS_SDA = 1;	
}

/*******************************************************************************
// set_clock()
// set SCL to "0" or "1"
*******************************************************************************/
void set_clock(int x) 
{
	if (x) {
		I2C2_LAT_SCL	= 1;	
	} else {
		I2C2_LAT_SCL	= 0;	
	}
}

/*******************************************************************************
// set_data()
// when configured to output, set SDA data to "0" or "1"
*******************************************************************************/
void set_data(int x) 
{
	if (x) {
		I2C2_LAT_SDA	= 1;	
	} else {
		I2C2_LAT_SDA	= 0;	
	}
}

/*******************************************************************************
// get_data()
// when configured to input, read value form SDA
*******************************************************************************/
int get_data(void) 
{
	return I2C2_PORT_SDA;		
}

/*******************************************************************************
// delay_i2c()
// add delay to SCL transaction
*******************************************************************************/
void delay_i2c(void)
{
	int i;
	for (i = 0; i < 15; i++);
}

/*******************************************************************************
// reset_i2c()
// generate up to 10 clocks, and sample data
// until SDA is released (pulled to "1")
// then, start and stop bits
*******************************************************************************/
void reset_i2c(void) 
{
	int i;	
	data_in();
	for (i = 0; i < 9; i++) {
		delay_i2c();
		set_clock(0);
		delay_i2c();
		set_clock(1);
		if (get_data())
			break;
	}
	start_i2c_ert(0);
	stop_i2c_ert();
}

#endif //HARDWARE_I2C_ERT
//#endif //WISDOM_STONE //YL 23.4 commented to let the plug use the eeprom too
