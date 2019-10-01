#ifndef MYDELAY_H
#define MYDELAY_H
/*
 *  Delays for use in RTOS Tasks. Blocks tasks. 
*/


#include <Arduino.h>

#define portTICK_PERIOD_US			( ( TickType_t ) 1000000 / configTICK_RATE_HZ )

void myDelayUs(int us);
void myDelayMs(int ms);
void myDelayMsUntil(TickType_t *previousWakeTime, int ms);




#endif
