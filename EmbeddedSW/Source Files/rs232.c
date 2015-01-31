/*******************************************************************************

rs232.c - parse the input message
=================================

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
handle RS232 connection for debug.
use HyperTerminal.
*******************************************************************************/

/***** INCLUDE FILES: *********************************************************/
#include <uart.h>
#include <PPS.h>
#include "command.h"		//Application
#include "rs232.h"			//Devices
	
/***** GLOBAL VARIABLES: ******************************************************/
static char rs232_in_buff[RS232_BUFFLEN];
static char rs232_out_buff[RS232_BUFFLEN];
static int rs232_in_ptr = 0;
static int rs232_out_head = 0; 
static int rs232_out_tail = 0; 
static int rs232_out_full = 0; 

/***** INTERNAL PROTOTYPES: ***************************************************/
void resend_last_line();

/*******************************************************************************
// init_rs232()
// initialize the RS232 connection:
// - use high speed
// - boud rate 115,200
// - bits 8N1
*******************************************************************************/
void init_rs232(void) 
{
	U1MODE= 1;
	iPPSInput(IN_FN_PPS_U1RX,IN_PIN_PPS_RP24);
	iPPSOutput(OUT_PIN_PPS_RP23,OUT_FN_PPS_U1TX);
	CloseUART1();
	ConfigIntUART1(UART_RX_INT_DIS | UART_TX_INT_DIS);
	OpenUART1(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_UEN_00, UART_TX_ENABLE, 34);
	U1MODEbits.BRGH = 1; 
}

/*******************************************************************************
// resend_last_line()
*******************************************************************************/
void resend_last_line()
{
	rs232_in_ptr = 0;
	while (rs232_in_buff[rs232_in_ptr]) {
		rs232_putc(rs232_in_buff[rs232_in_ptr]);
		rs232_in_ptr++;
	}
}

/*******************************************************************************
// rs232_poll() 
*******************************************************************************/
char *rs232_poll(void)
{
	int ch;
	static int esc_mode = 0;
	
	// output data from buffer
	while (rs232_out_full || (rs232_out_head != rs232_out_tail)) {
		if (U1STAbits.UTXBF) // buffer full
			break;
		WriteUART1(rs232_out_buff[rs232_out_tail++]);
		if (rs232_out_tail >= RS232_BUFFLEN)
			rs232_out_tail = 0;
		rs232_out_full = 0;
	}

	// look for input
	if (DataRdyUART1()) {
		ch = ReadUART1();
		if (esc_mode || (ch == 27)) {
			// look for up arrow, to reshow cmd
			if (esc_mode == 0) {
				esc_mode = 1;
			} else if (esc_mode == 1) {
				if (ch == '[') esc_mode = 2;
				else esc_mode = 0;
			} else if (esc_mode == 2) {
				if (ch == 'A') {
					resend_last_line();
					esc_mode = 3;
				} else	esc_mode = 0;
			}
		}
		if (!esc_mode) {
			 // echo
			if (ch == '\r') {
				write_eol();
				rs232_in_buff[rs232_in_ptr] = 0;
				rs232_in_ptr = 0;
				return rs232_in_buff;
			}
			if (ch == '\b') {
				if (rs232_in_ptr > 0) {
					rs232_in_ptr--;
					WriteUART1('\b');
					WriteUART1(' ');  // delete chars
					WriteUART1('\b');  // delete chars
				}
			} else {
				WriteUART1(ch);
				if (rs232_in_ptr < (RS232_BUFFLEN - 1)) {
					rs232_in_buff[rs232_in_ptr++] = ch;
				}
			}
		}
		if (esc_mode == 3) 
			esc_mode = 0;
 	}
	return 0;
}

/*******************************************************************************
// rs232_flush()
*******************************************************************************/
void rs232_flush(void)
{
	while (rs232_out_full || (rs232_out_head != rs232_out_tail))
		rs232_poll();
}

/*******************************************************************************
// rs232_putc()
*******************************************************************************/
void rs232_putc(char data)
{
	while (rs232_out_full)
		rs232_poll();

	rs232_out_buff[rs232_out_head++] = data;
	if  (rs232_out_head >= RS232_BUFFLEN)
		rs232_out_head = 0;
	if (rs232_out_head == rs232_out_tail)
		rs232_out_full = 1;
}
