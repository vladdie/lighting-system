/* 
	Sample task that initialises the EA QVGA LCD display
	with touch screen controller and processes touch screen
	interrupt events.

	Jonathan Dukes (jdukes@scss.tcd.ie)
*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc24xx.h"
#include "commands.h"
#include <stdio.h>
#include <string.h>
#include "sensors.h"
#include "timers.h"

#define I2C_AA      0x00000004
#define I2C_SI      0x00000008
#define I2C_STO     0x00000010
#define I2C_STA     0x00000020
#define I2C_I2EN    0x00000040
//#define PRESET1_STATE    0x39
//#define PRESET2_STATE    0x5A


/* Maximum task stack size */
#define sensorsSTACK_SIZE			( ( unsigned portBASE_TYPE ) 256 )

/* The LCD task. */
static void vSensorsTask( void *pvParameters );

//static xQueueHandle xSensorToLcd;
static xSemaphoreHandle xSensorSemphr;

TimerHandle_t sensorTimer;
//unsigned char timerFinished;
unsigned char ledState;
unsigned char ledMask;
unsigned char timerCounter;
unsigned char LEDIndexBak;
//Command cmd;

void vStartSensors( unsigned portBASE_TYPE uxPriority, xQueueHandle xLCDTouchedQ, xSemaphoreHandle xLcdSemphr )
{
	static xQueueHandle xLcdToSensor;
	
	/* Enable and configure I2C0 */
	PCONP    |=  (1 << 7);                /* Enable power for I2C0              */

	/* Initialize pins for SDA (P0.27) and SCL (P0.28) functions                */
	PINSEL1  &= ~0x03C00000;
	PINSEL1  |=  0x01400000;

	/* Clear I2C state machine                                                  */
	I20CONCLR =  I2C_AA | I2C_SI | I2C_STA | I2C_I2EN;
	
	/* Setup I2C clock speed                                                    */
	I20SCLL   =  0x80;
	I20SCLH   =  0x80;
	
	I20CONSET =  I2C_I2EN;
	
	xSensorSemphr = xLcdSemphr;
	xLcdToSensor = xLCDTouchedQ;
//	xSensorToLcd = xSensorTouchedQ;

	
	/* Spawn the console task . */
	xTaskCreate( vSensorsTask, "Sensors", sensorsSTACK_SIZE, &xLcdToSensor, uxPriority, ( xTaskHandle * ) NULL );

	printf("Sensor task started ...\r\n");
}


/* Get I2C button status */
unsigned char getButtons()
{
	unsigned char buttonData;

	/* Initialise */
	I20CONCLR =  I2C_AA | I2C_SI | I2C_STA | I2C_STO;
	
	/* Request send START */
	I20CONSET =  I2C_STA;

	/* Wait for START to be sent */
	while (!(I20CONSET & I2C_SI));

	/* Request send PCA9532 ADDRESS and R/W bit and clear SI */		
	I20DAT    =  0xC0;
	I20CONCLR =  I2C_SI | I2C_STA;

	/* Wait for ADDRESS and R/W to be sent */
	while (!(I20CONSET & I2C_SI));

	/* Send control word to read PCA9532 INPUT0 register */
	I20DAT = 0x00;
	I20CONCLR =  I2C_SI;

	/* Wait for DATA with control word to be sent */
	while (!(I20CONSET & I2C_SI));


	/* Request send repeated START */
	I20CONSET =  I2C_STA;
	I20CONCLR =  I2C_SI;

	/* Wait for START to be sent */
	while (!(I20CONSET & I2C_SI));

	/* Request send PCA9532 ADDRESS and R/W bit and clear SI */		
	I20DAT    =  0xC1;
	I20CONCLR =  I2C_SI | I2C_STA;

	/* Wait for ADDRESS and R/W to be sent */
	while (!(I20CONSET & I2C_SI));

	I20CONCLR = I2C_SI;

	/* Wait for DATA to be received */
	while (!(I20CONSET & I2C_SI));

	buttonData = I20DAT;
	
	/* Request send NAQ and STOP */
	I20CONSET =  I2C_STO;
	I20CONCLR =  I2C_SI | I2C_AA;

	/* Wait for STOP to be sent */
	while (I20CONSET & I2C_STO);

	return buttonData ^ 0xf;
}



unsigned char writeToPCA9532(unsigned char LEDState)
{
	/* Initialise */
	I20CONCLR =  I2C_AA | I2C_SI | I2C_STA | I2C_STO;
	
	/* Request send START */
	I20CONSET =  I2C_STA;

	/* Wait for START to be sent */
	while (!(I20CONSET & I2C_SI));

	/* Request send PCA9532 ADDRESS and R/W bit and clear SI */		
	I20DAT    =  0xC0;
	I20CONCLR =  I2C_SI | I2C_STA;

	/* Wait for ADDRESS and R/W to be sent */
	while (!(I20CONSET & I2C_SI));

	/* Send control word to read PCA9532 LS2 register */
	I20DAT = 0x08;
	I20CONCLR =  I2C_SI;

	/* Wait for DATA with control word to be sent */
	while (!(I20CONSET & I2C_SI));



	I20DAT =  LEDState;
	I20CONCLR =  I2C_SI;

	/* Wait for DATA with control word to be sent */
	while (!(I20CONSET & I2C_SI));
	
	/* Request send STOP */
  I20CONSET =  I2C_STO;
	I20CONCLR =  I2C_SI;
	/* Wait for STOP to be sent */
	while (I20CONSET & I2C_STO);

	return LEDState;
}

void sendDataToPWM0Register()
{
	/* Initialise */
	I20CONCLR =  I2C_AA | I2C_SI | I2C_STA | I2C_STO;
	
	/* Request send START */
	I20CONSET =  I2C_STA;

	/* Wait for START to be sent */
	while (!(I20CONSET & I2C_SI));

	/* Request send PCA9532 ADDRESS and R/W bit and clear SI */		
	I20DAT    =  0xC0;
	I20CONCLR =  I2C_SI | I2C_STA;

	/* Wait for ADDRESS and R/W to be sent */
	while (!(I20CONSET & I2C_SI));

	/* Send control word to write PCA9532 PWM0 register */
	I20DAT = 0x03;
	I20CONCLR =  I2C_SI;

	/* Wait for DATA with control word to be sent */
	while (!(I20CONSET & I2C_SI));


	//255*0.5
	I20DAT =  0x7F;
	I20CONCLR =  I2C_SI;

	/* Wait for DATA with control word to be sent */
	while (!(I20CONSET & I2C_SI));
	
	/* Request send STOP */
  I20CONSET =  I2C_STO;
	I20CONCLR =  I2C_SI;
	/* Wait for STOP to be sent */
	while (I20CONSET & I2C_STO);

	printf("write to PWM0 finished\r\n");
}

void sendDataToPWM1Register()
{
	/* Initialise */
	I20CONCLR =  I2C_AA | I2C_SI | I2C_STA | I2C_STO;
	
	/* Request send START */
	I20CONSET =  I2C_STA;

	/* Wait for START to be sent */
	while (!(I20CONSET & I2C_SI));

	/* Request send PCA9532 ADDRESS and R/W bit and clear SI */		
	I20DAT    =  0xC0;
	I20CONCLR =  I2C_SI | I2C_STA;

	/* Wait for ADDRESS and R/W to be sent */
	while (!(I20CONSET & I2C_SI));

	/* Send control word to write PCA9532 PWM0 register */
	I20DAT = 0x05;
	I20CONCLR =  I2C_SI;

	/* Wait for DATA with control word to be sent */
	while (!(I20CONSET & I2C_SI));


	//255*0.1
	I20DAT =  0x19;
	I20CONCLR =  I2C_SI;

	/* Wait for DATA with control word to be sent */
	while (!(I20CONSET & I2C_SI));
	
	/* Request send STOP */
  I20CONSET =  I2C_STO;
	I20CONCLR =  I2C_SI;
	/* Wait for STOP to be sent */
	while (I20CONSET & I2C_STO);

	printf("write to PWM1 finished\r\n");
}


void vTimerCallback(TimerHandle_t sensorTimer){
	//unsigned char Mask;
	//printf("Timer finished!\r\n");
//	printf("after timer state: %i\r\n", ledState);
	//printf("after timer mask: %i\r\n", ledMask);
	//ledState = ledState & ledMask;
		//writeToPCA9532(ledState);
	switch(timerCounter){
			case 0:
						ledState = ledState & ledMask;
						writeToPCA9532(ledState);
						printf("Action of detected motion finished!\r\n");
						break;
						
			case 1:
						timerCounter--;
						ledState = ledState & ledMask;
						ledState |= (3<<LEDIndexBak * 2);
						writeToPCA9532(ledState);
						xTimerReset(sensorTimer,1000 );
						printf("dimmer2...\r\n");
						break;
			case 2:
						timerCounter--;
						ledState = ledState & ledMask;
						ledState |= (2<<LEDIndexBak * 2);
						writeToPCA9532(ledState);
						xTimerReset(sensorTimer,1000 );
						printf("dimmer1...\r\n");
						break;
	}
	//xTimerReset(sensorTimer,10 );
	//writeToPCA9532(0x00);	

}

void alarmLEDTimerCallback(TimerHandle_t alarmLEDTimer){
	
}


static portTASK_FUNCTION( vSensorsTask, pvParameters )
{
	portTickType xLastWakeTime;
	unsigned char buttonState;
	unsigned char lastButtonState;
	unsigned char changedState;
	unsigned int i;
	unsigned char mask;
	unsigned char singleLEDState;
	unsigned char presetState;
	unsigned char restoreState;
	unsigned char sendFlag;
	//unsigned char ledState;
	//unsigned char ledMask;
	xQueueHandle xLCDTouchedQ;
	//Command cmd, cmd2;
	Command cmd;
	cmd.LedIndex = -1;
	timerCounter = 0;
	sendFlag = 1;
	/* Just to stop compiler warnings. */
	( void ) pvParameters;
	
	xLCDTouchedQ = * ( ( xQueueHandle * ) pvParameters );

	printf("Starting sensor poll ...\r\n");

	/* initialise lastState with all buttons off */
	lastButtonState = 0;
	presetState = 0x39;
	restoreState = 0;
	/* initial xLastWakeTime for accurate polling interval */
	xLastWakeTime = xTaskGetTickCount();
	
	sendDataToPWM0Register();
	sendDataToPWM1Register();
					 
	sensorTimer = xTimerCreate("Timer",1000,pdFALSE,(void*)0, vTimerCallback);
	
	/* Infinite loop blocks waiting for a touch screen interrupt event from
	 * the queue. */
	while( 1 )
	{
		//open all the 4 LEDs
		//sendDataToLEDRegister();
		
		/* Get command from Q */
		//xQueueReceive(xSensorToLcd, &cmd, portMAX_DELAY);
		xQueueReceive(xLCDTouchedQ, &cmd, 0);
		//printf("CMD value is: %i\r\n", cmd.value);
		
		//printf("HERE******current LED State: %i\r\n", ledState);
		/* Execute command */
		if(cmd.value == STORE_PRESET){
			presetState = ledState;
		}
		if(cmd.value == USE_PRESET){
			ledState = presetState;
			writeToPCA9532(ledState);	
		}
		if(cmd.value == AUTO_STORE){
			 //printf("HERE******AUTO STORE value: %i\r\n", restoreState);
			 restoreState = ledState;
		}
		if(cmd.value == ALARM_RESTORE){
			//printf("HERE******RESTORE value: %i\r\n", restoreState);
			ledState = restoreState;
			writeToPCA9532(ledState);	
		}
		if(cmd.LedIndex == 0){
			switch (cmd.value)
	    {
	        case LED_ON:
								//ledState = 0x55;
                writeToPCA9532(0x55);	
								ledState = 0x55;
                break;
	        
	        case LED_OFF:
								//ledState = 0x0;
					      writeToPCA9532(0x0);	
								ledState = 0x0;
                break;
					case DIM_DECREASE:
						for(i = 0; i < 4; i++ ){
							singleLEDState = ledState & (3 << i * 2);
							singleLEDState >>= i * 2;
							if(singleLEDState == 1){
								ledState = ledState & ~(3 << i * 2);
								ledState |= (2<< i * 2);
								writeToPCA9532(ledState);	
							}
							if(singleLEDState == 2){
								ledState = ledState &  ~(3 << i * 2);
								ledState |= (3<< i * 2);
								writeToPCA9532(ledState);	
							}
							/*if(singleLEDState == 3){
								ledState = ledState &  ~(3 << i * 2);
								writeToPCA9532(ledState);	
							}*/
						}
             break;
						
					case DIM_INCREASE:
					
						for(i = 0; i < 4; i++)
					{
							singleLEDState = ledState & (3 << i * 2);
							singleLEDState >>= i * 2;
							/*if(singleLEDState == 0){
								ledState = ledState & ~(3 << i * 2);
								ledState |= (3 << i * 2);
								writeToPCA9532(ledState);	
							}*/
							if(singleLEDState == 3){
								ledState = ledState & ~(3 << i * 2);
								ledState |= (2 << i * 2);
								writeToPCA9532(ledState);	
							}
							if(singleLEDState == 2){
								ledState = ledState & ~(3 << i * 2);//clear correspond led position
								ledState |= (1 << i * 2);
								writeToPCA9532(ledState);
							}
						}
						
             break;
	    }
		}
		else{
			if(cmd.LedIndex != -1){
				ledMask = ~(3 << (cmd.LedIndex - 1) * 2);
			}
			
			switch (cmd.value)
	    {
	        case LED_ON:
								ledState = ledState & ledMask;//clear correspond led position
								ledState |= (1<<(cmd.LedIndex - 1) * 2);
                writeToPCA9532(ledState);	
                break;
	        
	        case LED_OFF:
								ledState = ledState & ledMask;
                writeToPCA9532(ledState);	
                break;
					
					case DIM_DECREASE:
						//printf("HERE******LED ID: %i\r\n", cmd.LedIndex);
						singleLEDState = ledState & (~ledMask);
						singleLEDState = singleLEDState >> (cmd.LedIndex - 1) * 2;
						//printf("HERE******LED state: %i\r\n", singleLEDState);
					  //singleLEDState ^= (1<<(cmd.LedIndex - 1) * 2);
					  //full lighted up
						if(singleLEDState == 1){
							ledState = ledState & ledMask;
							ledState |= (2<<(cmd.LedIndex - 1) * 2);
							writeToPCA9532(ledState);	
						}
						if(singleLEDState == 2){
							ledState = ledState & ledMask;
							ledState |= (3<<(cmd.LedIndex - 1) * 2);
							writeToPCA9532(ledState);	
						}
						/*if(singleLEDState == 3){
							ledState = ledState & ledMask;
							writeToPCA9532(ledState);	
						}*/
             break;
						
					case DIM_INCREASE:
						//printf("HERE******LED ID: %i\r\n", cmd.LedIndex);
						singleLEDState = ledState & (~ledMask);
						singleLEDState >>= (cmd.LedIndex - 1) * 2;
						//printf("HERE******LED state: %i\r\n", singleLEDState);
					  //singleLEDState ^= (1<<(cmd.LedIndex - 1) * 2);
					  //full lighted up
						/*if(singleLEDState == 0){
							ledState = ledState & ledMask;
							ledState |= (3<<(cmd.LedIndex - 1) * 2);
							writeToPCA9532(ledState);	
						}*/
						if(singleLEDState == 3){
							ledState = ledState & ledMask;
							ledState |= (2<<(cmd.LedIndex - 1) * 2);
							writeToPCA9532(ledState);	
						}
						if(singleLEDState == 2){
							ledState = ledState & ledMask;//clear correspond led position
							ledState |= (1<<(cmd.LedIndex - 1) * 2);
              writeToPCA9532(ledState);
						}
             break;
						
					case PIR_MotionAction:
						//detect if the LED is already on:
									if((ledState & (~ledMask))==(0<<(cmd.LedIndex - 1) * 2) ){
									LEDIndexBak = cmd.LedIndex - 1;
									//printf("LCDTOSENSOR_motionAction_6?: %i\r\n", cmd.value);
									ledState = ledState & ledMask;//clear correspond led position
									ledState |= (1<<(cmd.LedIndex - 1) * 2);
									writeToPCA9532(ledState);	
									//printf("Before timer state: %i\r\n", ledState);
									//printf("before timer mask: %i\r\n", ledMask);
									timerCounter = 2;
									xTimerStart(sensorTimer,0);
								}
                break;
	    }
		}

	    
		
		/* Read buttons */
		buttonState = getButtons();

		changedState = buttonState ^ lastButtonState;
		if (buttonState != lastButtonState)
		{
			sendFlag++;
			sendFlag %= 2;
		    /* iterate over each of the 4 LS bits looking for changes in state */
			for (i = 0; i <= 3; i = i++)
            {
                mask = 1 << i;
                
                if (changedState & mask)
                {
                    printf("Button %u is %s\r\n", i,
                        (buttonState & mask) ? "on" : "off");
									  //printf("Button state is %u\r\n", buttonState);
									 if((i == 2) && sendFlag){
											//cmd2.value = FIREALARM;
											cmd.value = FIREALARM;
									 }else{
										 if(sendFlag){
												//cmd2.value = PIR_MotionDetected;
											 cmd.value = PIR_MotionDetected;
										 }
											
									 }
									 //cmd2.LedIndex = i+1;
									 cmd.LedIndex = i+1;
									 //xQueueSendToBack(xSensorToLcd, &cmd2, portMAX_DELAY);
									 xQueueSendToBack(xLCDTouchedQ, &cmd, portMAX_DELAY);
									 xSemaphoreGive(xSensorSemphr);
									
                } 
								
		    }
		   // writeToPCA9532(0x1);	
				//xTimerStart(sensorTimer,0);
			/* remember new state */
			lastButtonState = buttonState; 
		}
		
		//clear the local variable that stores the command information
        cmd.LedIndex = -1;
				cmd.value = -1;
				//cmd2.LedIndex = -1;
				//cmd2.value = -1;
        /* delay before next poll */
    	vTaskDelayUntil( &xLastWakeTime, 20);
	}
}
