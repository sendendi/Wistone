/*******************************************************************************

system.c - low level handle system commands
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
following are the functions for handling system commands, like get version...
*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include "wistone_main.h"					//Application
#include "command.h"						//Application
#include "error.h"							//Application	
#include "parser.h"							//Application
#include "system.h"							//Application
#include "HardwareProfile.h"				//Common
#include "HardwareProfileRemappable.h"		//Common
#include "misc_c.h"							//Common	//YL 19.9
#include "p24FJ256GB110.h"					//Common	
#include "accelerometer.h"					//Devices	
#include "analog.h"							//Devices
#include "flash.h"							//Devices
#include "lcd.h"							//Devices
#include "led_buzzer.h"						//Devices
#include "rtc.h"							//Devices			
#include "temp.h"							//Devices
#include "i2c.h"							//Protocols		
#ifdef USBCOM							
#include "wistone_usb.h"					//USB_UART
#include "usb.h"							//USB_UART
#elif defined RS232COM
#include "rs232.h"							//USB_UART
#endif // #ifdef USBCOM
#include "TxRx.h"							//TxRx - Application
#include "SymbolTime.h"						//TxRx - Application
#include "HardwareProfileTxRx.h"			//TxRx - Common	
#include "TimeDelay.h"						//TxRx - Common	

/***** GLOBAL VARIABLES: ******************************************************/
char	*g_version = "1.07.0";		 		// program version number. to be incremented each new release
char   	g_curr_boot_cmd[MAX_BOOT_CMD_LEN];	// YL 17.9 the next boot command to be executed
BOOL   	g_sn_write = FALSE;					// YL 11.11 to protect the last EEPROM's byte which stores the serial number of a stone 
int		g_vbat_level;						// the voltage sampled at AN10, updated by Timer4 state machine

/***** INTERNAL PROTOTYPES: ***************************************************/
void handle_get_error(void);
void handle_gver();	
void display_welcome(void);

#define MAX_USB_RETRIES 1000 // YS 17.8
//defines the time we wait for a USB connection - each retry is 2mSec long
//For example - 1000 means we wait for 2 seconds.

/*******************************************************************************
// init_all()
// main initialization for the entire HW
// used in both WISDOME_STONE and COMMUNICATION_PLUG modes
*******************************************************************************/
void init_all(void) {
	// remappable pins configs
	PPS_config(); 	// YL 5.8 moved here from main of the plug and of the stone		
	init_power();		
#ifdef RS232COM
	init_rs232(); 					// initialize RS232
#elif defined USBCOM
    long USBRetries = 0;
	InitializeSystem(); 			// initialize USB
	USBDeviceAttach();	
	// wait for the USB to connect:
	// if USB could not connect for some reason, then this is endless loop
	// therefore, when Wistone is under ground without USB, undef USBCOM
	while ((USBGetDeviceState() < CONFIGURED_STATE) && (USBRetries < MAX_USB_RETRIES)) {
		USBRetries++;				// YS 17.8
		DelayMs(2);
	}
	if (USBRetries == MAX_USB_RETRIES) {
		g_usb_connected = FALSE; 	// YS 17.8
		USBDisableInterrupts();
	} else {
		g_usb_connected = TRUE; 	// YS 17.8
	}
#endif //#ifdef RS232COM 
	init_leds();
	init_buzzer(); 
	init_timer4(); 					// needed for activating power LED blink and buzzer
	init_i2c();						// YL 18.7 init_i2c() for the plug too - to let it read the EUI for TxRx
#ifdef WISDOM_STONE
	init_rtc();	
	init_temp_sensor();		
	init_flash();
	// YL 25.8 init_accmtr(); 		// if the SensorI board is not assembled the SW gets stuck here. need to fix this. - the code exists; make sure the problem is fixed
	// init_ads1282();				// YL 11.12
	// YL 16.8 ...	
	TxRx_Init(FALSE);	// YL 16.8 to start TxRx along with USB;
	//	was: 
	//  if (g_usb_connected == FALSE){
	//		TxRx_Init(FALSE);			// YS 17.8 // YS 17.11
	//	}
	// ... YL 16.8

#elif defined COMMUNICATION_PLUG 	// YS 17.8
	USBEnableInterrupts();			// YS 22.12
	if (g_usb_connected == FALSE){
		while ((USBGetDeviceState() < CONFIGURED_STATE)); // YS 17.8
		//comm_plug has to have a USB connection
		g_usb_connected = TRUE; 	// YS 17.8
	}
	TxRx_Init(FALSE);				// YS 17.8 // YS 17.11
#endif //#ifdef WISDOM_STONE

#ifdef LCD_INSTALLED
	init_lcd();						//YL 9.8 instead of separate init_io_lcd() and init_screen()
#endif //#ifdef LCD_INSTALLED 
#ifdef WISDOM_STONE
	err_clear();
#endif //#ifdef WISDOM_STONE
	display_welcome();
	// YL 24.9 ...
#if defined COMMUNICATION_PLUG	
	TxRx_PrintNetworkTopology();
#endif
	// ... YL 24.9	
	return;
}

/*******************************************************************************
// init_power()
// init power board related IO and their pull-ups: status and control pins.
// disable all DC-DC circuits, excluding 3.3v
// then, hold PWR_SHUTDOWN so the power switch can be released
*******************************************************************************/
void init_power(void)
{
	// set control and status pins direction:
	// '0' - means output
	// '1' - means input
	PWR_SHUTDOWN_TRIS			= 0;
	PWR_SHUTDOWN_DETECT_TRIS	= 1;
	PWR_CHRG_STAT1_TRIS			= 1;
	PWR_CHRG_STAT2_TRIS			= 1;
	PWR_EN_5V_TRIS				= 0;
	PWR_EN_12V_TRIS				= 0;
	PWR_EN_M12V_TRIS			= 0;
	PWR_VBAT_STAT_TRIS			= 1;
	PWR_CHRG_USBPG_TRIS			= 1;
	PWR_CHRG_ACPG_TRIS			= 1;
	
	PWR_SHUTDOWN = 1; 	// hold power enable signal for the 3.3v DC-DC
	// disable all voltage sources, besides 3.3v (that must be enabled as long as the system is on).
	// the rest of the sources can be enabled by "spower" command if desired 
	PWR_EN_5V	= 0; 
	PWR_EN_12V	= 0;
	PWR_EN_M12V	= 0;
	
	// define PULLUPS on the following inputs:
	CNPU2bits.CN19PUE = 1; // STAT1 - CN19
	CNPU4bits.CN61PUE = 1; // STAT2 - CN61
	CNPU4bits.CN59PUE = 1; // USBPG - CN59
	CNPU6bits.CN81PUE = 1; // ACPG  - CN81

	return;
}

/*****************************************************************************/
// display_welcome()
/*****************************************************************************/
void display_welcome(void) //YL 2.5 added USB_ReceiveData() after most m_write cmd to enable the printing in init stage too
{	
   	m_write("\r\nWisdom-Stone, Welcome!\r\n");
	// YL 12.9 USB_ReceiveData(); 		// YL 2.5
	write_ver(g_version);	// return initial message to terminal
	#if defined (COMMUNICATION_PLUG)
    m_write("\r\n     Starting COMMUNICATION_PLUG Mode");
	// YL 12.9 USB_ReceiveData(); 		// YL 2.5
	#elif defined (WISDOM_STONE)
	m_write("\r\n     Starting WISDOM_STONE Mode");
	// YL 12.9 USB_ReceiveData(); 		// YL 2.5
	#endif //COMMUNICATION_PLUG
    #if defined (MRF24J40)
    m_write("\r\n     RF Transceiver: MRF24J40");
	// YL 12.9 USB_ReceiveData(); 		// YL 2.5
    #elif defined (MRF49XA)
    m_write("\r\n     RF Transceiver: MRF49XA");
	// YL 12.9 USB_ReceiveData(); 		// YL 2.5
    #endif //MRF24J40
    #ifdef EXPLORER16
	m_write("\r\n   Demo Instruction:");
    m_write("\r\n                     Power on the board until LED 1 lights up");
    m_write("\r\n                     to indicate it is ready to establish new");
    m_write("\r\n                     connections. Push Button 1 to perform");
    m_write("\r\n                     frequency agility procedure. Push Button");
    m_write("\r\n                     2 to unicast encrypted message. LED 2 will");
    m_write("\r\n                     be toggled upon receiving messages. ");
    m_write("\r\n\r\n");
	#endif //EXPLORER16
    #if defined (PROTOCOL_P2P)
	m_write("\r\n     Feature P2P Mode \r\n");
	// YL 12.9 USB_ReceiveData(); 		// YL 2.5
	#elif defined (PROTOCOL_MIWI) // YL 4.5 added MiWi case
	m_write("\r\n     Feature MiWi Mode \r\n");
	// YL 12.9 USB_ReceiveData();
	write_eol();
    #endif //PROTOCOL_P2P

	/*******************************************************************/
    // Following block display demo instructions on LCD based on the
    // demo board used. 
    /*******************************************************************/
    #if defined(EXPLORER16)
 	print_string(0, 0, "Wisdom-Stone);
	print_string(0, 1, "SW Ver:");
	print_string(8, 1, g_version);
	print_string(3, 2, "Welcome!");
	print_string(2, 3, "and Shalom!");
   #endif //EXPLORER16
}

#ifdef WISDOM_STONE

/*******************************************************************************
// prepare_for_shutdown()
// perform actions needed before going to sleep:
// 1. check whether the alarm is set; if not - 
//    set it to default - 10:00 AM on the first day of the next month;
//	  if alarm setting fails - do not shut down
*******************************************************************************/
void prepare_for_shutdown(void) //YL 18.9 moved here from app, edited 19.9
{
	Alarm alm;
	BYTE alarm_is_set = eeprom_read_byte(ALARM_ADDRESS);	 //YL 15.10 even if no alarm or wakeup cmds were received this time, the alarm might still be set (previously when the system was on)
		
	if (alarm_is_set == FALSE && g_wakeup_is_set == FALSE) { //YL 15.10 both alarm and wakeup aren't set, so set wakeup to a default value - 10:00 AM on the first day of the next month							
		m_write("setting wakeup alarm to a default - 10:00 AM on the first day of the next month"); 
		write_eol();
		alm.hour 	= 10;										
		alm.minute 	= 0;
		alm.day 	= 1;
		alm.weekday	= IRRELEVANT;	
		if (rtc_set_alarm(&alm)) {
			err(ERR_INVALID_SALARM);
			g_sleep_request = 0;
			m_write("failed to set the alarm, still working...");
			write_eol();
			main();		//if alarm setting did not succeed - do not shut down (go back to main)	
		}
	}
	m_write("shutting down... good bye.");
	DelayMs(1000);
	PWR_SHUTDOWN = 0;	
	while(1) {}
}

/*******************************************************************************
// handle_system()
// if first token was SYS, then handle system commands message:
// - parse PARAM
// - according to sub command, call the relevant function
// - if command failed, display error message, and return error code
// - if command executed OK, display OK message and return 0.
*******************************************************************************/
int handle_system(int sub_cmd)
{
	int res = 0;

	// dispatch to the relevant sub command function according to sub command
	switch (sub_cmd) {
		case SUB_CMD_GERR:
			handle_get_error();
			break;
		
		case SUB_CMD_GVER:
			handle_gver(g_version);
			write_eol();
			break;
		
		case SUB_CMD_SID:
			res = handle_set_id();
			break;
		
		case SUB_CMD_GPOWER:	
			handle_get_power_status();
			break;
			
		case SUB_CMD_SPOWER:
			handle_set_power_enable();
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

//-----------------------------------------------
//?
void handle_get_error()
{
	m_write(get_last_error_str());
	write_eol();
	err_clear();
}

/*******************************************************************************
// handle_gver()
// get version function
*******************************************************************************/
void handle_gver()
{
	write_ver(g_version);
}

/*******************************************************************************
// handle_set_id()
// write the Wistone ID into the reserved last address of the EEPROM
*******************************************************************************/
int handle_set_id()
{
	int res;
	int id;		

	if (g_ntokens != 3)
		return cmd_error(ERR_INVALID_PARAM_COUNT);
	id = parse_long_num(g_tokens[2]);
	if ((id <= 0) || (id > MAX_SN))
		return cmd_error(ERR_INVALID_PARAM);
	g_sn_write = TRUE; // to enable SN write (to protected last eeprom addr) //YL 11.11
	res = eeprom_write_byte(SN_ADDRESS, id); 
	g_sn_write = FALSE;
	if (res < 0)
		return cmd_error(0);
	return 0;
}

/*******************************************************************************
// handle_get_power_status()
// get power status from all power sources
*******************************************************************************/
void handle_get_power_status(void)	//YL 13.8 - is it possible to check whether the battery/accelerometer are connected (like FLASH crad detect?...)
{
	m_write("Power Status:");
	m_write("\r\n\tAC Power Good: ");
	if (PWR_CHRG_ACPG == 0)
		m_write("0");
	else
		m_write("1");
	m_write("\r\n\tUSB Power Good: ");
	if (PWR_CHRG_USBPG == 0)
		m_write("0");
	else
		m_write("1");
	m_write("\r\n\tSTAT1: ");
	if (PWR_CHRG_STAT1 == 0)
		m_write("0");
	else
		m_write("1");
	m_write("\r\n\tSTAT2: ");
	if (PWR_CHRG_STAT2 == 0)
		m_write("0");
	else
		m_write("1");
	m_write("\r\n");

	m_write("\r\n\tBattery status: ");
	m_write(int_to_str(g_vbat_level));	//YL 11.11 instead of disp_num_to_term
	m_write(" - ");
	if (g_vbat_level >= (VBAT_STAT_MAX - VBAT_STAT_UNIT))
		m_write("completely full");
	else if (g_vbat_level >= (VBAT_STAT_MAX - (3 * VBAT_STAT_UNIT)))
		m_write("3/4 to full");
	else if (g_vbat_level >= (VBAT_STAT_MAX - (5 * VBAT_STAT_UNIT)))
		m_write("1/2 to 3/4 full");
	else if (g_vbat_level >= (VBAT_STAT_MAX - (7 * VBAT_STAT_UNIT)))
		m_write("3/4 to 1/2 full");
	else
		m_write("almost empty");
	m_write("\r\n");
	return;
}

/*******************************************************************************
// handle_set_power_enable()
// enables/disables specified power sources, according to following CLI:
//	"sys spower <5v enable> <12v enable> <-12v enable>"
//	- <5v enable>   - 0 - disable, or 1 - enable 5v   power source
//	- <12v enable>  - 0 - disable, or 1 - enable 12v  power source
//	- <-12v enable> - 0 - disable, or 1 - enable -12v power source
*******************************************************************************/
int handle_set_power_enable(void) {
	long enable_power;

	if (g_ntokens != 5)
		return(err(ERR_INVALID_PARAM_COUNT));
		
	// extruct and parse 5v parameter:
	enable_power = parse_long_num(g_tokens[2]);
	if (enable_power == 1) {
		PWR_EN_5V = 1;
		m_write(" 5V enabled;");
	}
	//YL 5.8 changed the following to avoid invalid input...
	//else {
	//	PWR_EN_5V = 0;
	//	m_write(" 5V disabled;");
	//}
	else if (enable_power == 0) {
		PWR_EN_5V = 0;
		m_write(" 5V disabled;");
	}
	else {
		return(err(ERR_INVALID_PARAM));
	}
	//...YL 5.8
	
	// extruct and parse 12v parameter:
	enable_power = parse_long_num(g_tokens[3]);
	if (enable_power == 1) {
		PWR_EN_12V = 1;
		m_write(" 12V enabled;");
	}
	//YL 5.8 changed the following to avoid invalid input...
	//else {
	//	PWR_EN_12V = 0;
	//	m_write(" 12V disabled;");
	//}
	else if (enable_power == 0) {
		PWR_EN_12V = 0;
		m_write(" 12V disabled;");
	}
	else {
		return(err(ERR_INVALID_PARAM));
	}
	//...YL 5.8
	
	// extruct and parse -12v parameter:
	enable_power = parse_long_num(g_tokens[4]);
	if (enable_power == 1) {
		PWR_EN_M12V = 1;
		m_write(" -12V enabled;");
	}
	//YL 5.8 changed the following to avoid invalid input...
	//else {
	//	PWR_EN_M12V = 0;
	//	m_write(" -12V disabled;");
	//}
	else if (enable_power == 0) {
		PWR_EN_M12V = 0;
		m_write(" -12V disabled;");
	}
	else {
		return(err(ERR_INVALID_PARAM));
	}
	//...YL 5.8
	
	write_eol();

	return(0);
}

#endif // #ifdef WISDOM_STONE

/*******************************************************************************
// write_ver()
// return the following data:
// - SW version number: g_version
// - HW version number: Wisdom Stone ID from eeprom_read_byte(SN_ADDRESS)
*******************************************************************************/
void write_ver()
{
	m_write("Wisdome Stone SW Version: ");
	m_write(g_version);

	int serial;		
	int res;
	
	res = eeprom_read_byte(SN_ADDRESS); 
	if (res < 0) {
		cmd_error(ERR_EEPROM_READ_BYTE);
	}
	if (res > MAX_SN) 
		serial = 0;
	else
		serial = res;
	m_write("\t HW Version: ");
	m_write(int_to_str(serial));	//YL 11.11 instead of disp_num_to_term

}
