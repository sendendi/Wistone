#ifndef __LED_BUZZER_H__	
#define __LED_BUZZER_H__

/***** DEFINE: ****************************************************************/
#define LED_1 1
#define LED_2 2
#define SWITCH_1 1
#define SWITCH_2 2

// YL 12.9 ...
#if defined USBCOM  && defined COMMUNICATION_PLUG
#define MAX_USB_PACKETS_TO_HOST 2		// after USBTransferOnePacket() is called at least MAX_USB_PACKETS_TO_HOST times 
										// (= USBTransferOnePacket() with IN_TO_HOST parameter), 
										// USB_ReceiveData() is called by timer4 ISR to avoid buffer overflow
extern 	BYTE g_usb_packets_to_host;
#endif // USBCOM, COMMUNICATION_PLUG
// ... YL 12.9

/***** FUNCTION PROTOTYPES: ***************************************************/
void init_timer4(void);
void init_buzzer(void);
void init_leds(void);
int	 play_buzzer(int period);
int  set_led(int state, int led_num);
int  get_switch(int switch_num);

#endif //__LED_BUZZER_H__
