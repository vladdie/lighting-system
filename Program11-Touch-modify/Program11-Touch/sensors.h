#ifndef SENSORS_H
#define SENSORS_H

#include "semphr.h"

//void vStartSensors( unsigned portBASE_TYPE uxPriority, xQueueHandle xLCDTouchedQ, xQueueHandle xSensorTouchedQ, xSemaphoreHandle xLcdSemphr );
void vStartSensors( unsigned portBASE_TYPE uxPriority, xQueueHandle xLCDTouchedQ, xSemaphoreHandle xLcdSemphr );
#endif
