#ifndef __COMMAND_H__					
#define __COMMAND_H__

/***** INCLUDE FILES: *********************************************************/
#include "GenericTypeDefs.h"

/***** DEFINE: ****************************************************************/
#define MAX_TOKENS  10 				// max number of tokens allowed in message
#define MAX_CMD_LEN 100				

extern char *g_tokens[MAX_TOKENS]; 
extern int  g_ntokens; 				
extern char	g_in_msg[MAX_CMD_LEN]; 	
extern BOOL	g_boot_seq_pause;
extern BYTE g_is_cmd_received;

/***** FUNCTION PROTOTYPES: ***************************************************/
int 	exec_message_command(void);
void 	cmd_ok(void);
int 	cmd_error(int errid);
void 	write_eol(void);
void 	b_write(BYTE* block_buffer, int len);	
void 	m_write(char *str);	
void 	m_write_debug(char *str); //YL 20.4 added m_write_debug
void 	PrintChar(BYTE toPrint);
void 	PrintDec(BYTE toPrint);
void 	ConsolePut(BYTE c);
void 	blink_led();
void	blink_buzz(); //YL 7.5

#endif //#ifndef __COMMAND_H__
