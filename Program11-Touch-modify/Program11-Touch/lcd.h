#ifndef LCD_H
#define LCD_H

#include "semphr.h"
#include "queue.h"

//void vStartLcd( unsigned portBASE_TYPE uxPriority, xQueueHandle xLCDTouchedQ, xQueueHandle xSensorTouchedQ, xSemaphoreHandle xLcdSemphr );
void vStartLcd( unsigned portBASE_TYPE uxPriority, xQueueHandle xLCDTouchedQ, xSemaphoreHandle xLcdSemphr );

#endif
