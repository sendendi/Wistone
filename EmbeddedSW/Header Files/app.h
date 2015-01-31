#ifndef __APP_H__	
#define __APP_H__

extern int g_mode;

/***** FUNCTION PROTOTYPES: ***************************************************/
void 	handle_SS(void);
void 	handle_TS(void);
void 	handle_OST(void);
int 	handle_application(int sub_cmd);
BOOL 	runPlugCommand();

#endif // #ifndef __APP_H__
