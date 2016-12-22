#include "lcd_grph.h"
#include "UI.h"

Button buttons[] = {
				{1,   5,   10,  45,  50, "off", NOT_PRESSED, BLACK},//1: master button off
				{2,  55,   10,  95,  50, "on", NOT_PRESSED, MAGENTA}, //2: master button on
				{3, 140,   10, 180,  50, "+", NOT_PRESSED, YELLOW}, //3: master increase
				{4, 190,   10, 230,  50, "-", NOT_PRESSED, GREEN},
				{5,  5,  60, 45, 100, "off", NOT_PRESSED, BLACK},//LED1
				{6, 55,  60, 95, 100, "on", NOT_PRESSED, MAGENTA},
				{7,   140, 60,  180, 100, "+", NOT_PRESSED, YELLOW},
				{8,  190, 60, 230, 100, "-", NOT_PRESSED, GREEN},
				{9, 5, 110, 45, 150, "off", NOT_PRESSED, BLACK},//LED2
				{10,  55, 110, 95, 150, "on", NOT_PRESSED, MAGENTA},
				{11, 140,   110, 180,  150, "+", NOT_PRESSED, YELLOW}, 
				{12, 190,   110, 230,  150, "-", NOT_PRESSED, GREEN},
				{13,  5,  160, 45, 200, "off", NOT_PRESSED, BLACK},//LED3
				{14, 55,  160, 95, 200, "on", NOT_PRESSED, MAGENTA},
				{15,   140, 160,  180, 200, "+", NOT_PRESSED, YELLOW},
				{16,  190, 160, 230, 200, "-", NOT_PRESSED, GREEN},
				{17,  5,  210, 45, 250, "off", NOT_PRESSED, BLACK},//LED4
				{18, 55,  210, 95, 250, "on", NOT_PRESSED, MAGENTA},
				{19,   140, 210,  180, 250, "+", NOT_PRESSED, YELLOW},
				{20,  190, 210, 230, 250, "-", NOT_PRESSED, GREEN},
				{21,  20, 270, 70, 310, "Store", NOT_PRESSED, BLACK},//preset 1
				{22,  160, 270, 210, 310, "Preset", NOT_PRESSED, BLACK}//preset 2
				
		};

Sensor sensors[] = {
				{0, 115, 290,  20, NOT_PRESSED, CYAN},//0: alarm
				{1, 117, 90, 10, NOT_PRESSED, DARK_GRAY}, //1: sensor 1
				{2, 117, 140, 10, NOT_PRESSED, DARK_GRAY}, //2: sensor 2
				{3, 117, 190, 0, NOT_PRESSED, DARK_GRAY}, //3: alarm
				{4, 117, 240, 10, NOT_PRESSED, DARK_GRAY}//4: sensor4
				
		};

void drawDefaultUI()
{  
		unsigned char pStringOff[] = "off";
		unsigned char pStringOn[] = "on";
		unsigned char pMaster[] = "Master";
		unsigned char pLED1[] = "whiteboard";
		unsigned char pLED2[] = "lecturer";
		unsigned char pLED3[] = "seating";
		unsigned char pLED4[] = "aisle";
		unsigned char pIncrease[] = "+";
		unsigned char pDecrease[] = "-";
	
		lcd_fillScreen(LIGHT_GRAY);
		
		/*master button*/
		lcd_fillRect(5, 10, 45, 50, BLACK); 
		lcd_putString(20, 30, &pStringOff[0]);
		lcd_fillRect(55, 10, 95, 50, RED); 
		lcd_putString(70, 30, &pStringOn[0]);
		lcd_putString(100, 30, &pMaster[0]);
		lcd_fillRect(140, 10, 180, 50, YELLOW); 
		lcd_putString(155, 30, &pIncrease[0]);
		lcd_fillRect(190, 10, 230, 50, GREEN);
	  lcd_putString(205, 30, &pDecrease[0]);		
		/*Led buttons*/
		lcd_fillRect(5, 60, 45, 100, BLACK); 
		lcd_putString(20, 80, &pStringOff[0]);
		lcd_fillRect(55, 60, 95, 100, RED); 
		lcd_putString(70, 80, &pStringOn[0]);
		lcd_putString(100, 80, &pLED1[0]);
		lcd_fillRect(140, 60, 180, 100, YELLOW); 
		lcd_putString(155, 80, &pIncrease[0]);
		lcd_fillRect(190, 60, 230, 100, GREEN);
	  lcd_putString(205, 80, &pDecrease[0]);		
		
		lcd_fillRect(5, 110, 45, 150, BLACK); 
		lcd_putString(20, 130, &pStringOff[0]);
		lcd_fillRect(55, 110, 95, 150, RED); 
		lcd_putString(70, 130, &pStringOn[0]);
		lcd_putString(100, 130, &pLED2[0]);
		lcd_fillRect(140, 110, 180, 150, YELLOW); 
		lcd_putString(155, 130, &pIncrease[0]);
		lcd_fillRect(190, 110, 230, 150, GREEN);
	  lcd_putString(205, 130, &pDecrease[0]);		
		
		lcd_fillRect(5, 160, 45, 200, BLACK); 
		lcd_putString(20, 180, &pStringOff[0]);
		lcd_fillRect(55, 160, 95, 200, RED); 
		lcd_putString(70, 180, &pStringOn[0]);
		lcd_putString(100, 180, &pLED3[0]);
		lcd_fillRect(140, 160, 180, 200, YELLOW); 
		lcd_putString(155, 180, &pIncrease[0]);
		lcd_fillRect(190, 160, 230, 200, GREEN);
	  lcd_putString(205, 180, &pDecrease[0]);		
		
		lcd_fillRect(5, 210, 45, 250, BLACK); 
		lcd_putString(20, 230, &pStringOff[0]);
		lcd_fillRect(55, 210, 95, 250, RED); 
		lcd_putString(70, 230, &pStringOn[0]);
		lcd_putString(100, 230, &pLED4[0]);
		lcd_fillRect(140, 210, 180, 250, YELLOW); 
		lcd_putString(155, 230, &pIncrease[0]);
		lcd_fillRect(190, 210, 230, 250, GREEN);
	  lcd_putString(205, 230, &pDecrease[0]);		
}

void drawButton(Button *button)
{
	lcd_fillRect(button->x0, button->y0, button->x1, button->y1, button->state == PRESSED ?  BLUE:(button->buttonColor) );
	
	lcd_putString( button->x0 + (((button->x1 - button->x0) - (strlen(button->buttonName) * 5)) / 2),
		button->y0 + 29,
		button->buttonName);
}

void drawWords(){
	lcd_fontColor(WHITE, LIGHT_GRAY);
	lcd_putString(98, 30, "Master");
	lcd_putString(100, 65, "board");
	lcd_putString(97, 115, "lecturer");
	lcd_putString(97, 175, "seating");
	lcd_putString(101, 215, "aisle");
	lcd_putString(100, 290, "ALARM");
}
void drawPIRSensor(Sensor *sensor){
	unsigned char temp;
	//alarm
	for(temp = 0; temp <= sensor->r; temp++){
		lcd_circle(sensor->x, sensor->y, temp, sensor->state == PRESSED ?  RED:(sensor->sensorColor));
	}
	
}

void drawAlarmScreen()
{
	int i;
	
	lcd_fillScreen(PURPLE);
	drawWords();
	for (i = 0; i < 22; i++) {
		drawButton(&buttons[i]);
	}
	for (i = 1; i < 5; i++) {
		drawPIRSensor(&sensors[i]);
	}
}

void drawScreen()
{
	int i;
	
	lcd_fillScreen(LIGHT_GRAY);
	
	for (i = 0; i < 22; i++) {
		drawButton(&buttons[i]);
	}
	for (i = 0; i < 5; i++) {
		drawPIRSensor(&sensors[i]);
	}
	drawWords();
}

Sensor * getSensor(unsigned int x, unsigned int y)
{
	unsigned char i;
	Sensor *result = 0; 
	
	for ( i = 0; i < 5 && !result; i++) {
		if (x >= (sensors[i].x - sensors[i].r) && x <= (sensors[i].x + sensors[i].r)
			&& y >= (sensors[i].y - sensors[i].r) && y <= (sensors[i].y + sensors[i].r)) {
				result = &sensors[i];
		}
	}
	
	return result;
}

Sensor * getDetectedSensor(int sensorID)
{
	unsigned char i;
	Sensor *result = 0; 
	
	for ( i = 0; i < 5 && !result; i++) {
		if (sensors[i].id == sensorID) {
				result = &sensors[i];
		}
	}
	
	return result;
}

Button * getButton(unsigned int x, unsigned int y)
{
	int i;
	Button *result = 0; 
	
	for ( i = 0; i < 22 && !result; i++) {
		if (x >= buttons[i].x0 && x <= buttons[i].x1
			&& y >= buttons[i].y0 && y <= buttons[i].y1) {
				result = &buttons[i];
		}
	}
	
	return result;
}

