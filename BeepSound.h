#ifndef BEEPSOUND_H
#define BEEPSOUND_H

#include <softTone.h>

#define LED_NUMBER 4
#define READY 20
#define BEEP 29

#define RIGHT 2
#define DOWN 3
#define LEFT 4
#define UP 5
#define CLOSE 6
#define FAR 7
#define NO_DETECTION 8
#define DETECTION 9

extern int led[LED_NUMBER];
extern int numLED;
extern int move_bit;
extern int readyGauge;

class BeepSound
{
public:
	static void beepMode();
private:
	static void setBeep();
	static void readyBeep();
	static void runBeep();
	static void errorBeep();
	static bool beepReadyFlag;
};

#endif