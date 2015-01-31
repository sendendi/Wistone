#ifndef __RTC_H__
#define __RTC_H__

#include "GenericTypeDefs.h"

/***** DEFINE: ****************************************************************/
#define RTC_ADD 				0b01010001	// RTC device I2C address
//RTC registers:
#define CTRL_REG_1 				0x00
#define CTRL_REG_2 				0x01
#define SECONDS_REG				0x02
#define MINUTES_REG				0x03
#define HOURS_REG				0x04
#define DAYS_REG				0x05
#define WEEKDAYS_REG			0x06
#define MONTHS_REG				0x07
#define YEARS_REG				0x08
#define MINUTE_ALARM_REG		0x09
#define HOUR_ALARM_REG			0x0A
#define DAY_ALARM_REG			0x0B
#define WEEKDAY_ALARM_REG		0x0C
#define CLKOUT_FREQ_REG			0x0D
#define TIMER_CTRL_REG			0x0E
#define TIMER_REG				0x0F
//enable/disable alarm interrupts in CTRL_REG_2:
#define ENABLE_ALARM_INTR		0x02
#define DISABLE_ALARM_INTR		0x00
#define SINGLE_SHOT_ALARM		0x10		//TI/TP bit is used NOT as intended (according to data sheet), but to set single shot alarm operation (which is periodic by default)
//AE is the upper bit in all alarm registers; if AE is 1 - the alarm is disabled, if 0 - enabled:
#define MASK_AE_ON 				0x7F	
#define MASK_AE_OFF 			0x80
//masks all undefined read-only bits:
#define MASK_SECONDS_REG		0x7F		// in addition masks voltage-low bit
#define MASK_MINUTES_REG		0x7F	
#define MASK_HOURS_REG			0x3F
#define MASK_DAYS_REG			0x3F
#define MASK_WEEKDAYS_REG		0x07
#define MASK_MONTHS_REG			0x9F		
#define MASK_MINUTE_ALARM_REG	0x7F		// in addition masks AE bit
#define MASK_HOUR_ALARM_REG		0x3F		// in addition masks AE bit
#define MASK_DAY_ALARM_REG		0x3F		// in addition masks AE bit
#define MASK_WEEKDAY_ALARM_REG	0x07		// in addition masks AE bit
#define MASK_CLKOUT_FREQ_REG	0x83	
//temp:
#define IRRELEVANT				72 //TODO - meanwhile represents irrelevant alarm parameters (indicated by pressing 'x')

typedef struct
{
	BYTE	second;
	BYTE	minute;
	BYTE	hour;
} Time;

typedef struct
{
	BYTE	day;
	BYTE	month;
	BYTE	year;
} Date;

typedef struct
{
	Time	time;
	Date	date;
	BYTE	weekday;
} TimeAndDate;

typedef struct	
{
	BYTE 	minute;
	BYTE	hour;
	BYTE	day;
	BYTE	weekday;
} Alarm;

enum on_off_option 
{
	OFF = 0,
	ON = 1
};

extern BOOL	g_wakeup_is_set;
extern BOOL g_rtc_wakeup;

/***** FUNCTION PROTOTYPES: ***************************************************/
void	init_rtc(void); 
int 	rtc_set_alarm(Alarm* alm);	
int	 	handle_rtc(int sub_cmd);			
#endif //__RTC_H__
