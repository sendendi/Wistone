#ifndef __RS232_H__				
#define __RS232_H__

/***** DEFINE: ****************************************************************/
#define RS232_BUFFLEN 100

/***** FUNCTION PROTOTYPES: ***************************************************/
void init_rs232(void);
char *rs232_poll(void);
void rs232_putc(char data);
void rs232_flush(void);

#endif //__RS232_H__
