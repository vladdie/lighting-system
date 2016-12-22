#ifndef COMMANDS_H
#define COMMANDS_H

enum LED_CMD {LED_OFF, LED_ON, DIM_INCREASE, DIM_DECREASE, PIR_MotionDetected, PIR_MotionAction, STORE_PRESET, USE_PRESET, FIREALARM, AUTO_STORE, ALARM_RESTORE};

typedef struct Command
{
    int value;
	  int LedIndex;

} Command;

#endif /* COMMANDS_H */
