#include "BeepSound.h"
#include "../MyListener.h"

extern int led[LED_NUMBER];
extern int numLED;
extern int move_bit;
extern int readyGauge;

bool BeepSound::beepReadyFlag;

void BeepSound::beepMode()
{
	softToneWrite(BEEP,0);	//turn off sound after one frame(67ms)

	switch(move_bit=DETECTION)
	{
	case RIGHT:
	case LEFT:
	case DOWN:
	case UP:
	case CLOSE:
	case FAR:
		errorBeep();
		break;
	
	case NO_DETECTION:
	case DETECTION:
		readyBeep();
		runBeep();
		setBeep();
		break;
	}
}

void BeepSound::setBeep()	//when readygauge 1->0
{
	static int pre_readyGauge; 
	if(readyGauge==0 && pre_readyGauge==1)
		softToneWrite(BEEP,5120);

	pre_readyGauge = readyGauge;
}
void BeepSound::readyBeep()//when readygauge (READY-1)->READY
{
	static int pre_readyGauge;
	if(readyGauge==READY && pre_readyGauge==READY-1){
		softToneWrite(BEEP,5120);
		beepReadyFlag = true;
	}
	pre_readyGauge = readyGauge;
}
void BeepSound::runBeep()	//during running mode (readyGauge=READY)
{
	static int pre_gauge;
	if(beepReadyFlag) beepReadyFlag = false;
	else{
		if(pre_gauge!=led[numLED] && readyGauge==READY) 
			softToneWrite(BEEP,2500+led[numLED]*8);
	}
	pre_gauge = led[numLED];
}
void BeepSound::errorBeep()
{
	softToneWrite(BEEP,2300);
}
