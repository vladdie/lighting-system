#ifndef UI_H
#define UI_H

typedef enum {
	NOT_PRESSED = 0,
	PRESSED = 1
} ButtonState;


typedef struct {
	unsigned int id;
	unsigned int x0, y0, x1, y1;
	char buttonName[32];
	ButtonState state;
	unsigned short buttonColor;
} Button;

typedef struct {
	unsigned int id;
	unsigned int x, y, r;
	ButtonState state;
	unsigned short sensorColor;
} Sensor;


void drawDefaultUI(void);

void drawButton(Button *button);
void drawPIRSensor(Sensor *sensor);

void drawScreen(void);
void drawAlarmScreen(void);

void drawWords(void);

Button * getButton(unsigned int x, unsigned int y);
Sensor * getSensor(unsigned int x, unsigned int y);
Sensor * getDetectedSensor(int sensorID);
#endif /* UI_H */
