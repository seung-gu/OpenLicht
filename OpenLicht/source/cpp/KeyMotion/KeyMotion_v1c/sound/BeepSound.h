#ifndef BEEPSOUND_H
#define BEEPSOUND_H

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
