/* 
	Sample task that initialises the EA QVGA LCD display
	with touch screen controller and processes touch screen
	interrupt events.

	Jonathan Dukes (jdukes@scss.tcd.ie)
*/

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "lcd.h"
#include "lcd_hw.h"
#include "queue.h"
#include "lcd_grph.h"
#include <stdio.h>
#include "commands.h"
#include <string.h>
#include "timers.h"
#include "UI.h"

#define EVQ_MAX_EVENTS 10


#define P210BIT ( ( unsigned long ) 0x4 )

/* Maximum task stack size */
#define lcdSTACK_SIZE			( ( unsigned portBASE_TYPE ) 256 )

/* Interrupt handlers */
extern void vLCD_ISREntry( void );
void vLCD_ISRHandler( void );

/* The LCD task. */
static void vLcdTask( void *pvParameters );

/* Semaphore for ISR/task synchronisation */
static xSemaphoreHandle xLcdSemphr;

//static xQueueHandle xSensorToLcdQ;
Sensor *triggeredSensor;
TimerHandle_t alarmTimer;
unsigned char alarmFlag;
unsigned long currentState;
//unsigned char masterOn;
xQueueHandle xLCDTouchedQ;
Command cmd;
unsigned char alarmTimerFinished;

void vStartLcd( unsigned portBASE_TYPE uxPriority, xQueueHandle xLCDTouchedQ, xSemaphoreHandle xTouchScreenSemphr )
{
	static xQueueHandle xLcdToSensor;
	
	xLcdSemphr = xTouchScreenSemphr;
	xLcdToSensor = xLCDTouchedQ;
	//xSensorToLcdQ = xSensorTouchedQ;
	/* Spawn the console task. */
	xTaskCreate( vLcdTask, "Lcd", lcdSTACK_SIZE, &xLcdToSensor, uxPriority, ( xTaskHandle * ) NULL );
}

void xTimerCallback(TimerHandle_t xTimer){
	triggeredSensor->state = NOT_PRESSED;
	drawPIRSensor(triggeredSensor);

}

void alarmTimerCallback(TimerHandle_t alarmTimer){
		
		//alarmFlag = 0;
		triggeredSensor->state = NOT_PRESSED;
		alarmTimerFinished = 1;
		cmd.LedIndex = -1;
		cmd.value = ALARM_RESTORE;
		xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
		printf("***Alarm Timer finished\r\n");
		drawScreen();
}



static portTASK_FUNCTION( vLcdTask, pvParameters )
{
	unsigned int pressure;
	unsigned int xPos;
	unsigned int yPos;
	unsigned char masterOn;
	unsigned char alarmBlinkFrequency;

	TimerHandle_t xTimer;
	//unsigned char dimmerCounter;
	portTickType xLastWakeTime;
	
	Button *pressedButton;
//	Sensor *triggeredSensor;
	
	//xQueueHandle xLCDTouchedQ;
	//Command cmd;
	xLCDTouchedQ = * ( ( xQueueHandle * ) pvParameters );

	/* Just to stop compiler warnings. */
	( void ) pvParameters;

	printf("Touchscreen task running\r\n");
	
	

	/* Initialise LCD display */
	/* NOTE: We needed to delay calling lcd_init() until here because it uses
	 * xTaskDelay to implement a delay and, as a result, can only be called from
	 * a task */
	lcd_init();

	lcd_fillScreen(LIGHT_GRAY);
	drawScreen();
	
	PINSEL4 &= ~(3<<20);

	FIO2DIR1 |= P210BIT;
	FIO2SET1 = P210BIT;

	masterOn = 0;
	alarmFlag = 0;
	alarmTimerFinished = 0;
	xTimer = xTimerCreate("Timer",800,pdFALSE,(void*)0, xTimerCallback);
	alarmTimer = xTimerCreate("AlarmTimer",5000,pdFALSE,(void*)0, alarmTimerCallback);
	triggeredSensor = 0;
	/* Infinite loop blocks waiting for a touch screen interrupt event from
	 * the queue. */
	for( ;; )
	{
		
		//unsigned long currentState;
		/* Clear TS interrupts (EINT3) */
		/* Reset and (re-)enable TS interrupts on EINT3 */
		EXTINT = 8;						/* Reset EINT3 */

		/* Enable TS interrupt vector (VIC) (vector 17) */
		VICIntEnable = 1 << 17;			/* Enable interrupts on vector 17 */

		/* Block on a queue waiting for an event from the TS interrupt handler */
		xSemaphoreTake(xLcdSemphr, portMAX_DELAY);
				
		/* Disable TS interrupt vector (VIC) (vector 17) */
		VICIntEnClr = 1 << 17;

		/* +++ This point in the code can be interpreted as a screen button push event +++ */\
		//lcd_fillScreen(GREEN);

		
		currentState = (FIO2PIN1 & P210BIT);
		//xQueueReceive(xSensorToLcdQ, &cmd, 0);
		xQueueReceive(xLCDTouchedQ, &cmd, 0);
		//implement the changing of the word color of correspond button index. button index = cmd.ledID
		//printf("SensorToLEDQ_motionDetected_5?: %i\r\n", cmd.value);
		//printf("SensorToLEDQ_sensorID: %i\r\n", cmd.LedIndex);
		if(cmd.value == PIR_MotionDetected && (!alarmFlag)){
				triggeredSensor = getDetectedSensor(cmd.LedIndex);
				if(masterOn){
					//cmd.LedIndex = 2;
					triggeredSensor->state = PRESSED;
					drawPIRSensor(triggeredSensor);
					cmd.value = PIR_MotionAction;
					xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
					xTimerStart(xTimer,0);
				}	
			
		}
	
		if(cmd.value == FIREALARM){
				alarmFlag = 1;
				cmd.LedIndex = -1;
				cmd.value = AUTO_STORE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
				triggeredSensor = getDetectedSensor(0);
				triggeredSensor->state = PRESSED;
				drawPIRSensor(triggeredSensor);
			  //AlarmTimerCount = 6;
				drawAlarmScreen();
				xTimerStart(alarmTimer,0);
				
		}
		//printf("alarm flag: %i\r\n", alarmFlag);
		if(alarmFlag){
			alarmBlinkFrequency++;
			xSemaphoreGive(xLcdSemphr);
			
			cmd.LedIndex = 0;
			cmd.value = LED_ON;
			xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			if(alarmBlinkFrequency % 150 == 0){
				//printf("HERE: UPDATE\r\n");
				if (triggeredSensor->state == PRESSED )
				{
					triggeredSensor->state = NOT_PRESSED;
				}
				else 
				{
					triggeredSensor->state = PRESSED;
				}
				drawPIRSensor(triggeredSensor);
				currentState = (FIO2PIN1 & P210BIT);
				if (currentState )
				{
					FIO2CLR1 = P210BIT;
				}
				else 
				{
					FIO2SET1 = P210BIT;
				}
			}
			if(alarmTimerFinished){
				alarmFlag = 0;
				if(masterOn){
					//printf("HERE->master state = 1 TURN ON MAIN SWITCH!\r\n");
					FIO2CLR1 = P210BIT;
				}else{
					//printf("HERE->master state = 0 TURN OFF MAIN SWITCH!\r\n");
					FIO2SET1 = P210BIT;
				}
				alarmTimerFinished = 0;
			}
		}
		
		/*if((!alarmFlag) && (!masterOn)){
			printf("HERE->master state = 0 TURN OFF MAIN SWITCH!\r\n");
			FIO2SET1 = P210BIT;
		}
		if((!alarmFlag) && (masterOn)){
			printf("HERE->master state = 0 TURN ON MAIN SWITCH!\r\n");
			FIO2CLR1 = P210BIT;
		}*/
		
		/* Measure next sleep interval from this point */
		xLastWakeTime = xTaskGetTickCount();

		/* Start polling the touchscreen pressure and position ( getTouch(...) ) */
		/* Keep polling until pressure == 0 */
		getTouch(&xPos, &yPos, &pressure);
		
		//printf("TS event x=%i, y=%i\r\n", xPos, yPos);
		pressedButton = getButton(xPos, yPos);
		
		if (pressedButton) {
			//printf("TS button=%i\r\n", pressedButton->id);
			pressedButton->state = PRESSED;
			drawButton(pressedButton);
			
			if(pressedButton->id == 1){
				FIO2SET1 = P210BIT;
				cmd.LedIndex = 0;
				cmd.value = LED_OFF;
				masterOn = 0;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 2){
				FIO2CLR1 = P210BIT;
				cmd.LedIndex = 0;
				cmd.value = LED_ON;
				masterOn = 1;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 5){
				cmd.LedIndex = 1;
				cmd.value = LED_OFF;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 6 && masterOn){
				cmd.LedIndex = 1;
				cmd.value = LED_ON;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 9){
				cmd.LedIndex = 2;
				cmd.value = LED_OFF;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 10 && masterOn){
				cmd.LedIndex = 2;
				cmd.value = LED_ON;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 13){
				cmd.LedIndex = 3;
				cmd.value = LED_OFF;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 14 && masterOn){
				cmd.LedIndex = 3;
				cmd.value = LED_ON;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 17){
				cmd.LedIndex = 4;
				cmd.value = LED_OFF;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 18 && masterOn ){
				cmd.LedIndex = 4;
				cmd.value = LED_ON;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 4 && masterOn){
				cmd.LedIndex = 0;
				cmd.value = DIM_DECREASE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 8 && masterOn){
				cmd.LedIndex = 1;
				cmd.value = DIM_DECREASE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 12 && masterOn){
				cmd.LedIndex = 2;
				cmd.value = DIM_DECREASE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 16 && masterOn){
				cmd.LedIndex = 3;
				cmd.value = DIM_DECREASE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 20 && masterOn){
				cmd.LedIndex = 4;
				cmd.value = DIM_DECREASE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 3 && masterOn){
				cmd.LedIndex = 0;
				cmd.value = DIM_INCREASE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 7 && masterOn){
				cmd.LedIndex = 1;
				cmd.value = DIM_INCREASE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 11 && masterOn){
				cmd.LedIndex = 2;
				cmd.value = DIM_INCREASE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 15 && masterOn){
				cmd.LedIndex = 3;
				cmd.value = DIM_INCREASE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 19 && masterOn){
				cmd.LedIndex = 4;
				cmd.value = DIM_INCREASE;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 21 && masterOn){ //store preset values
				cmd.LedIndex = -1;
				cmd.value = STORE_PRESET;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			if(pressedButton->id == 22 && masterOn){ //using preset values
				cmd.LedIndex = -1;
				cmd.value = USE_PRESET;
				xQueueSendToBack( xLCDTouchedQ, &cmd, portMAX_DELAY);
			}
			
		}
		
			
		while (pressure > 0)
		{
			/* Get current pressure */
			getTouch(&xPos, &yPos, &pressure);

			// printf("master flag value is %u\r\n", masterOn);
		
			/* Delay to give us a 25ms periodic TS pressure sample */
			vTaskDelayUntil( &xLastWakeTime, 25 );
		}		
		
		

		/* +++ This point in the code can be interpreted as a screen button release event +++ */
		//drawDefaultUI();
		if (pressedButton) {
			pressedButton->state = NOT_PRESSED;
			drawButton(pressedButton);
		}
		
		//clear the local variable that stores the command information
        cmd.LedIndex = -1;
				cmd.value = -1;
	
	}
}


void vLCD_ISRHandler( void )
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	
	

	/* Process the touchscreen interrupt */
	/* We would want to indicate to the task above that an event has occurred */
	xSemaphoreGiveFromISR(xLcdSemphr, &xHigherPriorityTaskWoken);

	EXTINT = 8;					/* Reset EINT3 */
	VICVectAddr = 0;			/* Clear VIC interrupt */

	/* Exit the ISR.  If a task was woken by either a character being received
	or transmitted then a context switch will occur. */
	portEXIT_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


