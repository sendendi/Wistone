#ifndef __ANALOG_H__	
#define __ANALOG_H__

/***** DEFINE: ****************************************************************/
// sample stages:
#define SAMPLE_STAGE		1		// first stage: sample
#define CONVERT_STAGE		2		// second stage: convert
#define UPDATE_STAGE		3		// third stage: update global variables

/***** FUNCTION PROTOTYPES: ***************************************************/
void analog_init(void);
int detect_analog_input(int adc_stage);

#endif //__ANALOG_H__
