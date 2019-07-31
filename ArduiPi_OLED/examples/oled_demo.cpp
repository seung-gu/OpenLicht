/*********************************************************************
This is an example for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

This example is for a 128x32|64 size display using SPI or I2C to communicate
4 or 5 pins are required to interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution

02/18/2013 	Charles-Henri Hallard (http://hallard.me)
						Modified for compiling and use on Raspberry ArduiPi Board
						LCD size and connection are now passed as arguments on 
						the command line (no more #define on compilation needed)
						ArduiPi project documentation http://hallard.me/arduipi

Overwritten by Seung-Gu Kang (oled_demo class) ( July, 2019 )
						
*********************************************************************/

// USB charge pin
#define chargePin 1

#define lowBatPin 0
#define battery0 2	//0th bit 
#define battery1 3	//1st bit

// key buttons
#define pinA 22
#define pinB 21

// joystick
#define pinS 22		//south
#define pinN 24		//north
#define pinIN 23	//click

#define NUM_MENU 4


#include <stdio.h>
#include <wiringPi.h>

#include "ArduiPi_OLED_lib.h"
#include "Adafruit_GFX.h"
#include "ArduiPi_OLED.h"


// Instantiate the display
ArduiPi_OLED display;
using namespace std;


// Config Option
struct s_opts
{
	int oled = OLED_ADAFRUIT_SPI_128x64;
	int verbose = false;
}opts;


void startPageMode(void);
void runMode(void);

void batteryDisplay(void);
void chargingDisplay(void);

void lowBatteryCheck(void);
void batteryCheck(void);
void chargeCheck(void);

void pinStateUpdate(void);

void pinN_handler(void);
void initializePinN(void);

void pinS_handler(void);
void initializePinS(void);

void pinIN_handler(void);
void initializePinIN(void);


int mode_state = 0;
int updateScreen_flag = 1;
int wait_flag = 0;

int runProgram_flag = 0;
int systemDown_flag = 0;

/* ======================================================================
Function: main
Purpose : Main entry Point
Input 	: -
Output	: -
Comments: 
====================================================================== */
int main(int argc, char **argv)
{
	// Oled supported display in ArduiPi_SSD1306.h , SPI
	if ( !display.init(OLED_SPI_DC, OLED_SPI_RESET, OLED_SPI_CS, opts.oled) )
		exit(EXIT_FAILURE);	
	display.begin();			// init done
	display.clearDisplay();   	// clears the screen  buffer
	display.display();   		// display it (clear display)
	
	wiringPiSetup();
	
	initializePinN();
	initializePinS();
	initializePinIN();
	
	while(1){
		
		startPageMode();
			
		runMode();
		
		pinStateUpdate();
		
		//lowBatteryCheck();	
		//batteryCheck();	
		//chargeCheck();
		
		delay(30);
	}
	
	
	display.clearDisplay();
	display.display();

	// Free PI GPIO ports
	display.close();
}



void startPageMode(void)
{
	if(!updateScreen_flag) return;
	
	display.clearDisplay();
	display.setCursor(0,8);
	
	char* text_op[NUM_MENU] = {
			"Execute Programm\n",
			"Option\n",
			"Mode\n",
			"System down\n"
		};
	
	for(int item=0; item<NUM_MENU; item++){
		char text[25];
		
		if(mode_state == item)
			sprintf(text, "%s%s", "-> ", text_op[item]);
		else
			sprintf(text, "%s%s", "   ", text_op[item]);
			
		display.setTextSize(1);
		display.setTextColor(WHITE);
		
		display.printf("%s", text);
	}
		
	printf("state : %d\n", mode_state);
	
	//batteryDisplay();
	//chargingDisplay();
	
	display.display();
	
	updateScreen_flag = 0;
	//start_page_flag = 0;
}

void batteryDisplay(void)
{
	int state = 1;
	//battery level
	// state : 0 (<3.2v), 1(3.2~3.4v), 2(3.4~3.6v), 3(3.6~3.8v), 4(>3.8v)
	if(digitalRead(lowBatPin) == 1){	// < 3.2V
		state = 0;
	}else{
		if(digitalRead(battery0) == 1)	
			state++;
		if(digitalRead(battery1) == 1)
			state+=2;
	}
	display.setCursor(0,0);
	
	int bat_pos_x = 107;	
	// battery gauge
	display.setBuffer(bat_pos_x, 0, 0x7e);
	display.setBuffer(bat_pos_x+1, 0, 0x81);	
	for(int gauge=1; gauge<=4; gauge++){
		if(gauge <= state){	// infill gauge
			display.setBuffer(bat_pos_x+gauge*3-1, 0, 0xbd);
			display.setBuffer(bat_pos_x+gauge*3, 0, 0xbd);
		}else{			// empty gauge
			display.setBuffer(bat_pos_x+gauge*3-1, 0, 0x81);
			display.setBuffer(bat_pos_x+gauge*3, 0, 0x81);
		}
		display.setBuffer(bat_pos_x+gauge*3+1, 0, 0x81);	
	}
	display.setBuffer(bat_pos_x+14, 0, 0x7e);
	display.setBuffer(bat_pos_x+15, 0, 0x18);
}

void chargingDisplay()
{
	int lightening_pos_x = 124;
	if(digitalRead(chargePin)){
		display.setBuffer(lightening_pos_x, 0, 0x48);
		display.setBuffer(lightening_pos_x+1, 0, 0x3c);
		display.setBuffer(lightening_pos_x+2, 0, 0x1e);
		display.setBuffer(lightening_pos_x+3, 0, 0x09);
	}else{
		display.setBuffer(lightening_pos_x, 0, 0x00);
		display.setBuffer(lightening_pos_x+1, 0, 0x00);
		display.setBuffer(lightening_pos_x+2, 0, 0x00);
		display.setBuffer(lightening_pos_x+3, 0, 0x00);
	}
}


void runMode(void)
{
	static FILE *handle = NULL;
	static int pre_runProgram_flag = 0;
	
	//initialize
	if(pre_runProgram_flag==0 && runProgram_flag==1)	//rising edge	
	{				
		handle = popen("/home/pi/ProjectKeyMotion/./program.sh", "r");
		// /home/pi/Downloads/OpenLicht/libroyale-3.17.0.56-LINUX-arm-32Bit/samples/cpp/KeyMotion_v1a/./KeyMotion_v1a
		wait_flag = 1;
		
		if (handle == NULL){
			printf("Can't open /home/pi/ProjectKeyMotion/./program.sh \n");
			return;
		}
		
		printf("Program execute!\n");
	}
	
	if(	runProgram_flag==1 )
	{			
		//running
		char buff[3];	
		fgets(buff, 3, handle);		// modified after updated , why???
			
		//it needs time to get data from another process
		//therefore, until it gets data, wait_flag is on not to generate interrupt							
		wait_flag = 0;
		
		printf("%s", buff);
		
		if(buff[0] == '-' && buff[1] == '1'){
			runProgram_flag = 0;
			printf("Camera connection error\n");
		}else if(buff[0] == '0'){
			runProgram_flag = 0;
			printf("Program off\n");
		}
	}
	
	//end
	if(pre_runProgram_flag==1 && runProgram_flag==0)	//falling edge
	{	
		pclose(handle);
		handle = NULL;
		
		updateScreen_flag = 1;
	}
	
	pre_runProgram_flag = runProgram_flag;
}

void chargeCheck(void)
{
	if(digitalRead(chargePin) == 1)
		printf("charging ...\n");
}

void batteryCheck(void)
{
	int state = 1;
	//battery level
	// state : 0 (<3.2v), 1(3.2~3.4v), 2(3.4~3.6v), 3(3.6~3.8v), 4(>3.8v)
	if(digitalRead(lowBatPin) == 1){	// < 3.2V
		state = 0;
	}else{
		if(digitalRead(battery0) == 1)	
			state++;
		if(digitalRead(battery1) == 1)
			state+=2;
	}
		
	switch(state)
	{
	case 0:	//0%
		printf("Battery : |    |\n");
		break;
	case 1:	//0~25% 
		printf("Battery : |*   |\n");
		break;
	case 2:
		printf("Battery : |**  |\n");
		break;
	case 3:
		printf("Battery : |*** |\n");
		break;
	case 4:
		printf("Battery : |****|\n");
		break;
	}
}

// if the voltage drops under 3.2V, xmc sends 'HIGH' to gpio.1
void lowBatteryCheck(void)
{
	static int batCount = 0;
	
	if(digitalRead(lowBatPin) == 1){
		batCount++;
		if(batCount > 100){
		/*	char text[] = "Battery is too low! It will be shut down soon\n";
			display.setCursor(0,0);
			display.setTextSize(1);
			display.setTextColor(WHITE);
			display.printf("%s", text);
			display.display();
		*/	
			printf("Battery is too low! It will be shut down soon\n");
			batCount = 0;
			
			delay(5000);
			
		//	FILE *handle = popen("/home/pi/ProjectKeyMotion/./systemDown.sh", "r");
		//	if (handle == NULL) return;
			
		//	exit(1);
		}	
	}else
		batCount = 0;
}


void pinStateUpdate(void)
{
	// pre-state of pin values.
	static int pre_state_pins[4];
	// current state of pin values -> charge, lowBat, bat0, bat1
	int state_pins[4] = {
		digitalRead(chargePin), 
		digitalRead(lowBatPin),
		digitalRead(battery0),
		digitalRead(battery1)
	};
		
	for(int i=0; i<4; i++){
		if(pre_state_pins[i] != state_pins[i])
			if(!runProgram_flag)
				updateScreen_flag = 1;
		pre_state_pins[i] = state_pins[i];
	}
}



void initializePinS(void)
{
	pinMode(pinS, INPUT);
	pullUpDnControl(pinS, PUD_UP);
	wiringPiISR(pinS, INT_EDGE_RISING, pinS_handler);
}

void pinS_handler(void)
{
	if( mode_state < 3 && !runProgram_flag){
		mode_state++;
		
		updateScreen_flag = 1;
		//start_page_flag = 1;
	}
}


void initializePinN(void)
{
	pinMode(pinN, INPUT);
	pullUpDnControl(pinN, PUD_UP);
	wiringPiISR(pinN, INT_EDGE_RISING, pinN_handler);
}

void pinN_handler(void)
{
	if( mode_state > 0 && !runProgram_flag){
		mode_state--;
		
		updateScreen_flag = 1;
		//start_page_flag = 1;
	}
}



void initializePinIN(void)
{
	pinMode(pinIN, INPUT);
	pullUpDnControl(pinIN, PUD_UP);
	wiringPiISR(pinIN, INT_EDGE_RISING, pinIN_handler);
}

void pinIN_handler(void)
{
	if(mode_state == 0 && wait_flag == 0){
		if(runProgram_flag == 0) runProgram_flag = !runProgram_flag;
	}
	
	if(mode_state == 3){	//system shut down
		
		display.clearDisplay();
		display.display();
		
		FILE *handle = popen("/home/pi/ProjectKeyMotion/./systemDown.sh", "r");
		if (handle == NULL) return;
		
		printf("end\n");
		exit(1);
	}
}

