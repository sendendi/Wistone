/*******************************************************************************

rtc.c - Real Time Clock driver
==============================

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
following are the functions for RTC handling.
it supports I2C configuration for: RTC-8564JE
*******************************************************************************/
#include "wistone_main.h"
#ifdef WISDOM_STONE

/***** INCLUDE FILES: *********************************************************/
#include <string.h>			//YL 6.8 to use strlen
#include "command.h"		//Application
#include "error.h"			//Application
#include "parser.h"			//Application	
#include "misc_c.h"			//Common
#include "eeprom.h"			//Devices	//YL 19.9
#include "rtc.h"			//Devices
#include "i2c.h"			//Protocols

/***** GLOBAL VARIABLES: ******************************************************/	
BOOL    g_rtc_wakeup = FALSE;			//YL 17.9
BOOL	g_wakeup_is_set = FALSE;		//YL 15.10 indicates whether or not "rtc swakeup" cmd determines the next wakeup time
BOOL	g_alarm_is_disabled = FALSE;	//YL 15.10 indicates that "rtc salarm x x x x" was received

/***** INTERNAL PROTOTYPES: ***************************************************/
void 	rtc_on_off(int option);
BYTE 	init_rtc_needed(void);	
BYTE 	num_to_bcd(BYTE num);
BYTE 	bcd_to_num(BYTE bcd);
void 	time_date_to_rtc(TimeAndDate* tad, BYTE* rtc);
void 	rtc_to_time_date(TimeAndDate* tad, BYTE* rtc); 
int 	rtc_get_time_date(TimeAndDate* tad);
int 	rtc_set_time_date(TimeAndDate* tad); 
int 	rtc_set_wakeup(BYTE delay);						//YL 16.9
char* 	weekday_to_str(BYTE weekday);					//YL 21.8
BOOL 	check_date(BYTE day, BYTE month, BYTE year);	//YL 27.8
static 	void	f_write(char* str, char** f_str);		//YL 29.12

/*******************************************************************************
// init_rtc()
// Configures RTC registers
*******************************************************************************/
void init_rtc(void) //YL 21.8
{
	BOOL vl_is_set = FALSE;				//YL 28.8 if FALSE - it means that time, date and alarm registers do not need to be restarted
			
	//writes to CTRL_REG_1: zeros all register bits (all register bits, except for bit #5, must be 0 as long the RTC is used) 
	BYTE ctrl_reg_1[2] = {CTRL_REG_1, 0x00};
	
	//writes to CTRL_REG_2: zeros #5 and #7 bits (must); zeros #0 (TIE) and #1 (AIE) bits to prevent unintended interrupts; zeros #2 (TF) and #3 (AF) bits - to clear interrupt signal (so /INT status changes from low to high) 
	BYTE ctrl_reg_2[2] = {CTRL_REG_2, 0x00};
	
	//reads #7 (VL) bit in SECONDS_REG; if VL is '0' - time, date, alarm and timer registers are already set, and need not be reinitialized
	BYTE seconds_reg[1] = {SECONDS_REG};
	if (device_write_i2c_ert(RTC_ADD, 1, seconds_reg, I2C_READ)) 
		err(ERR_RTC_INIT);
	if (device_read_i2c_ert(RTC_ADD, 1, seconds_reg, I2C_READ)) 
		err(ERR_RTC_INIT);
	if (seconds_reg[0] & 0x80)
		vl_is_set = TRUE;
		
	//in case a restart is not needed - reads ctrl_reg_2 in order to keep it's current setting and clears only alarm and timer interrupt flags
	if (vl_is_set == FALSE) { 					
		if (device_write_i2c_ert(RTC_ADD, 1, &ctrl_reg_2[0], I2C_READ)) 
			err(ERR_RTC_INIT);
		if (device_read_i2c_ert(RTC_ADD, 1, &ctrl_reg_2[1], I2C_READ)) 
			err(ERR_RTC_INIT);
		if (ctrl_reg_2[1] & 0x08) 					//if #3 (AF) bit is set - the wakeup source is rtc
			g_rtc_wakeup = TRUE;
		ctrl_reg_2[1] = ctrl_reg_2[1] & 0xF7;		//clear #3 (AF) bit
	} 		
	
	//starts writing SECONDS register in RTC: fills SECONDS, MINUTES, HOURS, DAYS, WEEKDAYS, MONTHS and YEARS registers with initial time and date: 22:22:22 11/11/11/FRI(= 5)  
	//each time data is transferred RTC increments current address by one byte, so after SECONDS (0x02) register is written to, MINUTES (0x03) register is written to, and so on 
	//in addition - #7 bit (VL) in SECONDS register is cleared (after it was set to 1 during initial power-on), so it would be ready for next voltage detections
	BYTE time_date_regs[8] = {SECONDS_REG, 0x22, 0x22, 0x22, 0x11, 0x5, 0x11, 0x11}; 
		
	//sets the #7 bit (AE) to 1 in MINUTE_ALARM / HOUR_ALARM / DAY_ALARM / WEEKDAY_ALARM registers - so no alarm interrupt events will occur
	BYTE alarm_regs[5]  = {MINUTE_ALARM_REG, 0x80, 0x80, 0x80, 0x80};
	
	//YL 21.8 default values are used in CLKOUT_FREQ register: the power-on function sets "1" to FE (CLKOUT pin is in output mode), and clears both FD0 and FD1 bits, so the output is at 32.768 KHz
	
	//writes zeros to TIMER_CTRL register to clear #7 bit (TE)
	BYTE timer_ctrl_reg[2] = {TIMER_CTRL_REG, 0x00};
	
	rtc_on_off(OFF);			//stops the RTC clocking, to allow time programming
	if (device_write_i2c_ert(RTC_ADD, 2, ctrl_reg_1, I2C_WRITE)) 
		err(ERR_RTC_INIT);
	if (device_write_i2c_ert(RTC_ADD, 2, ctrl_reg_2, I2C_WRITE)) 
		err(ERR_RTC_INIT);
	if (vl_is_set == TRUE) {	//VL is set, so time, date, alarm and timer registers must be reset
		if (device_write_i2c_ert(RTC_ADD, 8, time_date_regs, I2C_WRITE)) //each time data is transferred RTC increments current address by one byte
			err(ERR_RTC_INIT);	
		if (device_write_i2c_ert(RTC_ADD, 5, alarm_regs, I2C_WRITE)) 
			err(ERR_RTC_INIT);
		if (device_write_i2c_ert(RTC_ADD, 2, timer_ctrl_reg, I2C_WRITE)) 
			err(ERR_RTC_INIT);
		if (eeprom_write_byte(ALARM_ADDRESS, 0))	//YL 15.10 clear "rtc salarm" indication
			err(ERR_RTC_INIT);
	}
	rtc_on_off(ON);				//restart RTC clocking	
}

/*******************************************************************************
// init_rtc_needed()
// Checks if the RTC needs initialization.
// Input:
//    None
// When RTC voltage (battery source) falls below some threshold level, the VL bit
// ADD 0x02, bit 7, is latched. This indicates that the RTC has erased the time,
// and therefore needs to be re-initialized.
// Return Values:
//    1 - if it needs initializaion.
//    0 - otherwise.
*******************************************************************************/
BYTE init_rtc_needed(void)
{
	BYTE data_to_write[1];
	BYTE read_byte;

	data_to_write[0] = SECONDS_REG; 
	// first, write the address we would like to start read from
	if (device_write_i2c_ert(RTC_ADD, 1, data_to_write, I2C_READ))
		return 1;
	// then, read the data byte
	if (device_read_i2c_ert(RTC_ADD, 1, &read_byte, I2C_READ))	
		return 1;
	return (0b10000000 & read_byte); // checks the voltage low bit
}

/*******************************************************************************
// rtc_on_off()
// switch RTC power ON/OFF using CTRL_REG1 for start/stop clock function
// input: 0 - OFF, 1 - ON;
// this function set the STOP bit at the RTC control register 1 according to ON/OFF
*******************************************************************************/
void rtc_on_off(int option) //YL 20.8
{
	BYTE enable[2] = {CTRL_REG_1, 0b00000000};	//CTRL_REG_1 bits are all set to 0 
	
	if (option == OFF)
		enable[1] = 0b00100000; 	 			//CTRL_REG_1<5> is STOP bit; if set to 1 - clock function is stopped
	device_write_i2c_ert(RTC_ADD, 2, enable, I2C_WRITE);
}

/*******************************************************************************
// num_to_bcd()
// translates BYTE number to its BCD form.
// Input:
//    num - the number
// Return Values:
//    the BCD form of the number
*******************************************************************************/
BYTE num_to_bcd(BYTE num)
{
	BYTE res;
	BYTE temp = num;

	res = ((temp / 10) << 4);
	res = res | (temp % 10);
		
	return res;
}
	
/*******************************************************************************
// bcd_to_num()
// translates BCD form of a uchar number to its normal form.
// Input:
//    bcd - the number in BCD form
// Return Values:
//    the number
*******************************************************************************/
BYTE bcd_to_num(BYTE bcd)
{
	BYTE res;
	BYTE temp = bcd;
	
	res = 10 * (temp >> 4);
	res += (temp & 0x0F);
	
	return res;
}

/*******************************************************************************
// time_date_to_rtc()
*******************************************************************************/
void time_date_to_rtc(TimeAndDate* tad, BYTE* rtc)
{	
	rtc[0] = num_to_bcd(tad->time.second);
	rtc[1] = num_to_bcd(tad->time.minute);	
	rtc[2] = num_to_bcd(tad->time.hour);
	rtc[3] = num_to_bcd(tad->date.day);	
	rtc[4] = num_to_bcd(tad->weekday);
	rtc[5] = num_to_bcd(tad->date.month);
	rtc[6] = num_to_bcd(tad->date.year);
}

/*******************************************************************************
// rtc_to_time_date()
*******************************************************************************/
void rtc_to_time_date(TimeAndDate* tad, BYTE* rtc)
{
	tad->time.second 	= bcd_to_num(rtc[0]);
	tad->time.minute	= bcd_to_num(rtc[1]);
	tad->time.hour		= bcd_to_num(rtc[2]);
	tad->date.day 		= bcd_to_num(rtc[3]);
	tad->weekday	 	= bcd_to_num(rtc[4]);
	tad->date.month 	= bcd_to_num(rtc[5]);
	tad->date.year 		= bcd_to_num(rtc[6]); 
}	

/*******************************************************************************
// weekday_to_str()
// convert the byte representing the weekday to string
*******************************************************************************/
char* weekday_to_str(BYTE weekday)	//YL 21.8	
{	
	switch (weekday) {
	case 0x00:	return "SUN";
	case 0x01:	return "MON";
	case 0x02:	return "TUE";
	case 0x03:	return "WED";
	case 0x04:	return "THU";
	case 0x05:	return "FRI";
	case 0x06:	return "SAT";
	}
	return NULL;
}

/*******************************************************************************
// f_write()
// formatted printing: add leading '0' if needed to output any number as 2 digit string 
*******************************************************************************/
static void f_write(char* str, char** f_str)  // YL 29.12 for more effective wireless transmissions
{	
	int len = strlen(str);
	if (len == 1) {
		// was:
		// m_write("0");
		// m_write(str);
		strcpy(*f_str, "0");
		strcat(*f_str, str);
	} else if (len == 2) {
		// was: m_write(str);
		strcpy(*f_str, str);
	} else
		// was: m_write("00");
		strcpy(*f_str, "00");
		// ... YL 29.12
}

/*******************************************************************************
// check_date()
// check whether day, month and year yield legal setting
*******************************************************************************/
BOOL check_date(BYTE day, BYTE month, BYTE year)	//YL 27.8	
{	
	switch (month) {
	case 1:	case 3:	case 5: case 7: case 8:	case 10: case 12:
		return (day > 31) ? FALSE : TRUE;
	case 4:	case 6:	case 9: case 11:
		return (day > 30) ? FALSE : TRUE;
	case 2:	
		if (year % 4 == 0)	//leap year
			return (day > 29) ? FALSE : TRUE;
		else
			return (day > 28) ? FALSE : TRUE;
	default:
		return FALSE;
	}
}

/*******************************************************************************
// rtc_set_time_date()
// Sets the RTC values to a given time or date.
// Input:
//    tad - a pointer to a TimeAndDate struct with the wanted values.
// Return Values:
//    0 - if no error detected
// Remarks:
//   Values in the struct should be in regular form (not BCD).
*******************************************************************************/
int rtc_set_time_date(TimeAndDate* tad) 
{
	BYTE bcd[8] = {0};

	time_date_to_rtc(tad, bcd);
	bcd[7] = bcd[6]; 
	bcd[6] = bcd[5];
	bcd[5] = bcd[4];	
	bcd[4] = bcd[3];	
	bcd[3] = bcd[2];	
	bcd[2] = bcd[1];	
	bcd[1] = bcd[0];
	bcd[0] = SECONDS_REG;	//YL 20.8 starts writing from SECONDS register in RTC, fills date and/or time registers with date and/or time input (bcd[1] to bcd[7])
	
	rtc_on_off(OFF);		// stops the RTC clocking, to allow time programming
	if (device_write_i2c_ert(RTC_ADD, 8, bcd, I2C_WRITE)) 	// each time data is transferred RTC increments current address by one byte
		return err(ERR_RTC_INIT);
	rtc_on_off(ON);			// restart RTC clocking
	
	return 0;
}

/*******************************************************************************
// rtc_get_time_date()
// Gets the RTC time or date values.
// Input:
//    tad - a pointer to a TimeAndDate struct that will be updated with the values.
// Return Values:
//    error indication if any
// Remarks:
//    Values in the struct will be in regular form (not BCD).
*******************************************************************************/
int rtc_get_time_date(TimeAndDate* tad) 
{	
	BYTE temp_time[7]= {0};
	BYTE data_to_write[1];

	data_to_write[0] = SECONDS_REG; 	
	// first, write the address we would like to start read from
	if (device_write_i2c_ert(RTC_ADD, 1, data_to_write, I2C_READ))
		return err(ERR_RTC_GET_TIME);
	// then, read the data bytes
	if (device_read_i2c_ert(RTC_ADD, 7, temp_time, I2C_READ))
		return err(ERR_RTC_GET_TIME);
	temp_time[0] = temp_time[0] & MASK_SECONDS_REG;
	temp_time[1] = temp_time[1] & MASK_MINUTES_REG;	
	temp_time[2] = temp_time[2] & MASK_HOURS_REG;	
	temp_time[3] = temp_time[3] & MASK_DAYS_REG;	
	temp_time[4] = temp_time[4] & MASK_WEEKDAYS_REG;	
	temp_time[5] = temp_time[5] & MASK_MONTHS_REG;		
	rtc_to_time_date(tad, temp_time);	
	return 0;		
}

/*******************************************************************************
// rtc_set_wakeup()
// Sets the time for the system to wake up once (by RTC)
// Input:
//    delay - time duration in minutes before the RTC wakes the system up
// Return Values:
//    0 - if no error detected, (-1) - otherwise
*******************************************************************************/
int rtc_set_wakeup(BYTE delay) //YL 16.9
{
	TimeAndDate tad;
	Alarm alm;
	
	if (rtc_get_time_date(&tad)) 
		return err(ERR_INVALID_SWAKEUP);		
	alm.minute = tad.time.minute + delay;
	if (alm.minute > 59) {
		BYTE minute_temp = alm.minute;
		alm.minute = minute_temp % 60;
		alm.hour = tad.time.hour + minute_temp / 60;	//should not exceed 24 hours period
		if (alm.hour > 23) 
			return err(ERR_INVALID_SWAKEUP);
	}
	else
		alm.hour = IRRELEVANT;
	alm.day = IRRELEVANT;
	alm.weekday = IRRELEVANT;
	if (rtc_set_alarm(&alm)) 
		return err(ERR_INVALID_SWAKEUP);
	return 0;
}

/*******************************************************************************
// rtc_set_alarm()
// Sets the RTC alarm values to a given time and\or date; the alarm operation is periodic
// Input:
//    alm - a pointer to a Alarm struct with the wanted values.
// Return Values:
//    0 - if no error detected
// Remarks:
//   Values in the struct should be in regular form (not BCD).
*******************************************************************************/
int rtc_set_alarm(Alarm* alm) 
{
	int i = 0;	
	BYTE ctrl_reg_2[2] = {CTRL_REG_2, 0};
	BYTE alarm_regs[5] = {MINUTE_ALARM_REG, MASK_AE_OFF, MASK_AE_OFF, MASK_AE_OFF, MASK_AE_OFF};

	// set alarm registers: 
	if (alm->minute != IRRELEVANT) 
		alarm_regs[1] = MASK_AE_ON & num_to_bcd(alm->minute);
	else 
		i++;
	if (alm->hour != IRRELEVANT) 
		alarm_regs[2] = MASK_AE_ON & num_to_bcd(alm->hour);
	else 
		i++;
	if (alm->day != IRRELEVANT) 
		alarm_regs[3] = MASK_AE_ON & num_to_bcd(alm->day);
	else
		i++;
	if (alm->weekday != IRRELEVANT)
		alarm_regs[4] =  MASK_AE_ON & num_to_bcd(alm->weekday);
	else
		i++;
	// enable/disable alarm interrupts by setting #0 (AIE) bit in control register 2: 
	BYTE ctrl_reg_2_mask;
	if (i == 4) {  							// 4 'x' were entered to cancel all periodic alarm settings
		ctrl_reg_2_mask = DISABLE_ALARM_INTR;
		g_alarm_is_disabled = TRUE;			// YL 15.10 to indicate 4 'x' were entered in "rtc salarm" cmd
	}
	else {
		ctrl_reg_2_mask = ENABLE_ALARM_INTR;
		g_alarm_is_disabled = FALSE;		// YL 15.10 to indicate alarm needs to be set
	}
	rtc_on_off(OFF);						// stops the RTC clocking, to allow time programming - make sure that RTC needs to be stopped only when time or date are set 
	if (device_write_i2c_ert(RTC_ADD, 1, &ctrl_reg_2[0], I2C_READ))
		return err(ERR_INVALID_SALARM);
	if (device_write_i2c_ert(RTC_ADD, 1, &ctrl_reg_2[1], I2C_READ))
		return (ERR_INVALID_SALARM);
	ctrl_reg_2[1] |= ctrl_reg_2_mask;
	if (device_write_i2c_ert(RTC_ADD, 2, ctrl_reg_2, I2C_WRITE))
		return err(ERR_INVALID_SALARM);
	if (device_write_i2c_ert(RTC_ADD, 5, alarm_regs, I2C_WRITE))
		return err(ERR_INVALID_SALARM);
	rtc_on_off(ON);							// restart RTC clocking
		
	return 0;
}

/*******************************************************************************
// handle_rtc()
// if first token was "rtc", then handle RTC commands message:
// - parse PARAMETERS
// - according to sub command, call the relevant function
// - if command failed, display error message, and return error code
// - if command executed OK, display OK message and return 0.
*******************************************************************************/
int handle_rtc(int sub_cmd)	
{		
	TimeAndDate tad;
	Alarm alm;
	int res = 0;
	
	// YL 29.12 ... added f_str, ptr_f_str and data_to_print for more effective wireless transmissions
	char f_str[10];
	char* ptr_f_str = f_str;
	char data_to_print[40];	
	
	// dispatch to the relevant sub command function according to sub command
	switch (sub_cmd) {
	case SUB_CMD_GTIME:
		res = rtc_get_time_date(&tad);
		// was: 
		// m_write("Wistone TIME: "); 				
		// f_write(byte_to_str(tad.time.hour));
		// m_write(":");
		// f_write(byte_to_str(tad.time.minute));
		// m_write(":");
		// f_write(byte_to_str(tad.time.second));
		strcpy(data_to_print, "Wistone TIME: ");
		f_write(byte_to_str(tad.time.hour), (&ptr_f_str));
		strcat(data_to_print, ptr_f_str);
		strcat(data_to_print, ":");
		f_write(byte_to_str(tad.time.minute), (&ptr_f_str));
		strcat(data_to_print, ptr_f_str);
		strcat(data_to_print, ":");
		f_write(byte_to_str(tad.time.second), (&ptr_f_str));
		strcat(data_to_print, ptr_f_str);
		m_write(data_to_print);
		// ... YL 29.12
		write_eol();
	break;
		
	case SUB_CMD_GDATE:	
		res = rtc_get_time_date(&tad);
		// was:
		// m_write("Wistone DATE: ");				 
		// f_write(byte_to_str(tad.date.day));
		// m_write("/");
		// f_write(byte_to_str(tad.date.month));
		// m_write("/");
		// f_write(byte_to_str(tad.date.year));
		// m_write(" - ");
		// m_write(weekday_to_str(tad.weekday));
		strcpy(data_to_print, "Wistone DATE: ");
		f_write(byte_to_str(tad.date.day), (&ptr_f_str));
		strcat(data_to_print, ptr_f_str);
		strcat(data_to_print, "/");	
		f_write(byte_to_str(tad.date.month), (&ptr_f_str));
		strcat(data_to_print, ptr_f_str);
		strcat(data_to_print, "/");
		f_write(byte_to_str(tad.date.year), (&ptr_f_str));
		strcat(data_to_print, ptr_f_str);
		strcat(data_to_print, " - ");
		strcat(data_to_print, weekday_to_str(tad.weekday));
		m_write(data_to_print);		
		// ... YL 29.12
		write_eol();
	break;
		
	case SUB_CMD_STIME:	
		if (g_ntokens < 5)
			return cmd_error(ERR_INVALID_STIME);
		// first read the time, date and weekday (in order to use current rtc date and weekday)
		if (rtc_get_time_date(&tad) < 0)	
			return cmd_error(0);
		// then set the time sent in <hour> <minute> <seconds> format
		
		// YL 5.8 changed parse_byte_num...
		//tad.time.hour 	= parse_byte_num(g_tokens[2]);
		//tad.time.minute = parse_byte_num(g_tokens[3]);
		//tad.time.second = parse_byte_num(g_tokens[4]);
		res = parse_byte_num(g_tokens[2]);
		if (res == (-1))
			return cmd_error(ERR_INVALID_NUM);
		tad.time.hour = 0xFF & res;
		
		res = parse_byte_num(g_tokens[3]);
		if (res == (-1))
			return cmd_error(ERR_INVALID_NUM);
		tad.time.minute = 0xFF & res;
		
		res = parse_byte_num(g_tokens[4]);
		if (res == (-1))
			return cmd_error(ERR_INVALID_NUM);
		tad.time.second = 0xFF & res;		
		//...YL 5.8
		
		if ((tad.time.hour > 23) || (tad.time.minute > 59) || (tad.time.second > 59)) 
			return cmd_error(ERR_INVALID_STIME);
		res = rtc_set_time_date(&tad); 
	break;
		
	case SUB_CMD_SDATE:
		if (g_ntokens < 6)
			return cmd_error(ERR_INVALID_SDATE);
		// first read the time, date and weekday (in order to use current rtc time and weekday)
		if (rtc_get_time_date(&tad) < 0) 
			return cmd_error(0);
		// then set the date sent in <day> <month> <year> <weekday> format
		
		//YL 5.8 changed parse_byte_num...
		//tad.date.day 	= parse_byte_num(g_tokens[2]);
		//tad.date.month 	= parse_byte_num(g_tokens[3]);
		//tad.date.year	= parse_byte_num(g_tokens[4]); 
		//tad.weekday 	= parse_byte_num(g_tokens[5]) - 1; //RTC weekdays start with 0 for SUN (= 1 by input)
		
		res = parse_byte_num(g_tokens[2]);
		if (res == (-1))
			return cmd_error(ERR_INVALID_NUM);
		tad.date.day = 0xFF & res;
		
		res = parse_byte_num(g_tokens[3]);
		if (res == (-1))
			return cmd_error(ERR_INVALID_NUM);
		tad.date.month = 0xFF & res;
		
		res = parse_byte_num(g_tokens[4]);
		if (res == (-1)) 
			return cmd_error(ERR_INVALID_NUM);
		tad.date.year = 0xFF & res;
		
		 // RTC weekdays start with 0 for SUN (= 1 by input)
		res = parse_byte_num(g_tokens[5]);
		if ((res == (-1)) || (res == 0)) 
			return cmd_error(ERR_INVALID_NUM);
		tad.weekday = 0xFF & (res - 1);		
		//...YL 5.8
		
		if ((check_date(tad.date.day, tad.date.month, tad.date.year) == FALSE) || (tad.weekday > 6))
			return cmd_error(ERR_INVALID_SDATE);
		res = rtc_set_time_date(&tad); 
	break;
	
	case SUB_CMD_SWAKEUP:	//YL 15.9
		if (g_ntokens < 3)
			return cmd_error(ERR_INVALID_SWAKEUP);
		// rtc swakeup <delay> 
		//YL 5.8 changed parse_byte_num...
		res = parse_byte_num(g_tokens[2]);
		if (res == (-1))
			return cmd_error(ERR_INVALID_SWAKEUP);
		BYTE delay = 0xFF & res; 	
		//...YL 5.8
		res = rtc_set_wakeup(delay);
		if (res == 0) {
			g_wakeup_is_set = TRUE;							//YL 15.10 "rtc swakeup" cmd determines the next wakeup time
			if (eeprom_write_byte(ALARM_ADDRESS, 0))		//YL 15.10 clear "rtc salarm" indication - because the current cmd that determines the next wakeup time is "rtc swakeup"
				return err(ERR_INVALID_SWAKEUP);
		}
	break;
	
	case SUB_CMD_SALARM:
		if (g_ntokens < 6)
			return cmd_error(ERR_INVALID_SALARM);
		// rtc salarm <hour> <minute> <day> <weekday>
		//YL 5.8 changed parse_byte_num...
		//alm.hour 	= parse_byte_num(g_tokens[2]);
		//alm.minute 	= parse_byte_num(g_tokens[3]);
		//alm.day 	= parse_byte_num(g_tokens[4]);
		//alm.weekday	= parse_byte_num(g_tokens[5]);
		
		res = parse_byte_num(g_tokens[2]);
		if (res == (-1))
			return cmd_error(ERR_INVALID_SALARM);
		alm.hour = 0xFF & res;
		
		res = parse_byte_num(g_tokens[3]);
		if (res == (-1))
			return cmd_error(ERR_INVALID_SALARM);
		alm.minute = 0xFF & res;
		
		res = parse_byte_num(g_tokens[4]);
		if (res == (-1))
			return cmd_error(ERR_INVALID_SALARM);
		alm.day = 0xFF & res;
		
		res = parse_byte_num(g_tokens[5]);
		if (res == (-1))
			return cmd_error(ERR_INVALID_SALARM);
		alm.weekday = 0xFF & res;
		//...YL 5.8
		
		if ((alm.hour != IRRELEVANT && alm.hour > 23) || (alm.minute != IRRELEVANT && alm.minute > 59) 
			|| (alm.day != IRRELEVANT && (alm.day < 1 || alm.day > 31)) || (alm.weekday != IRRELEVANT && alm.weekday > 6))
			return cmd_error(ERR_INVALID_SALARM);			
		res = rtc_set_alarm(&alm);
		if (res == 0) {
			g_wakeup_is_set = FALSE;						//YL 15.10 "rtc swakeup" settings are no longer valid
			if (g_alarm_is_disabled == FALSE) {				//YL 15.10 "rtc salarm" cmd determines the next wakeup time					
				if (eeprom_write_byte(ALARM_ADDRESS, 1))	//YL 15.10 set "rtc salarm" indication - because the current cmd that determines the next wakeup time is "rtc salarm"
					return err(ERR_INVALID_SALARM);
			}
			else {
				if (eeprom_write_byte(ALARM_ADDRESS, 0))	//YL 15.10 clear "rtc salarm" indication - because the alarm settings were disabled
					return err(ERR_INVALID_SALARM);
			}
		}
	break;
		
	default:
		err(ERR_UNKNOWN_SUB_CMD);
		cmd_error(0);
	break;
	}
	if (res < 0)
		return cmd_error(0);

	cmd_ok();
	return 0;
}
#endif // #ifdef WISDOM_STONE
