#ifndef __ANALOG_H__	
#define __ANALOG_H__

/***** DEFINE: ****************************************************************/
//#define ANALOG_MASK 		0xFBFF		//only AN10 is analog (0), all the rest are digital IOs (1) 		
//#define N_ANALOG_SAMPLES 	16
// sample stages:
#define SAMPLE_STAGE		1		// first stage: sample
#define CONVERT_STAGE		2		// second stage: convert
#define UPDATE_STAGE		3		// third stage: update global variables

/***** FUNCTION PROTOTYPES: ***************************************************/
void analog_init(void);
//int analog_sample(void);
int detect_analog_input(int adc_stage);

#endif //__ANALOG_H__
