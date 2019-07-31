/****************************************************************************\
 * Copyright (C) 2017 Infineon Technologies & pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  
 * royale library version 3.17.0.56 for LINUX ARM
 * Overwritten by Seung-Gu Kang (onNewData class) 
 * 				in KeyMotion version 2a ( July, 2019 )
 \****************************************************************************/


#ifndef MYLISTENER_H
#define MYLISTENER_H

#include <royale.hpp>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sample_utils/PlatformResources.hpp>

#include <string>
// -lX11, -lxts library needed for fake key press 
#include <X11/extensions/XTest.h>

#include <wiringPi.h>
#include <softTone.h>

#include "display/sh1106/Adafruit_GFX.h"
#include "display/sh1106/ArduiPi_OLED.h"

using namespace royale;
using namespace sample_utils;
using namespace std;
using namespace cv;

// Linker errors for the OpenCV sample
//
// If this example gives linker errors about undefined references to cv::namedWindow and cv::imshow,
// or QFontEngine::glyphCache and qMessageFormatString (from OpenCV to Qt), it may be caused by a
// change in the compiler's C++ ABI.
//
// With Ubuntu and Debian's distribution packages, the libopencv packages that have 'v5' at the end
// of their name, for example libopencv-video2.4v5, are compatible with GCC 5 (and GCC 6), but
// incompatible with GCC 4.8 and GCC 4.9. The -dev packages don't have the postfix, but depend on
// the v5 (or non-v5) version of the corresponding lib package.  When Ubuntu moves to OpenCV 3.0,
// they're likely to drop the postfix (but the packages will be for GCC 5 or later).
//
// If you are manually installing OpenCV or Qt, you need to ensure that the binaries were compiled
// with the same version of the compiler.  The version number of the packages themselves doesn't say
// which ABI they use, it depends on which version of the compiler was used.

#define MAX_DEPTH 1.0 		// fixed value
// this can be customized (75cm) (real hand detection range, in myGetHistogram function)
#define MAX_DISTANCE 0.75 	
#define READY 20

enum PINS{PINLEFT=21, PINUP, PINPRESS, PINDOWN, PINRIGHT, KEY3=27, KEY2};
enum PINSOUND{BEEP=26};
enum MESSAGEBIT{NO_DETECTION=0, MSG_RIGHT, MSG_LEFT, MSG_DOWN};
enum KEYMESSAGE_TX{KEY_LEFT=0, KEY_RIGHT=2, KEY_DOWN=3};

extern ArduiPi_OLED display;

class Hand;

class MyListener : public IDepthDataListener
{
private:
	// define images for depth and gray
    // and for their 8Bit and scaled versions   
    Mat zImagef, grayImagef;	// float 32bit
    Mat grayImage; 		// 8bit gray image
    Mat maskDepthGray;	// merged (z + gray) 8bit image
    
    Mat handROI;		//with wrist
    Mat trackMat;
	
public:
	MyListener(){ }

    void onNewData (const DepthData *data);
    
private:
	vector<Hand> vHand;    

// image processing
private:    
    void preProcessing();	// grayNormalize+depthNormalize => mergedFrames
    
    void myGetHandROI(InputArray _src, OutputArray _dst);
	int myGetHistogram(InputArray _src, OutputArray _hist = noArray() );
	void myDepthROI(InputArray _src, OutputArray _dst, int min=0, int max=255);

	void myGetContour(InputArray _src, OutputArray _dst, OutputArray _fill = noArray() );
	
	void getHandCenter(InputArray _mask);
	
	int countingFingers(InputArray _src, OutputArray _dst);
	void midPointCircle(vector<Point>&, Size);
	
	double radianToDegree(double angleValue){ return ((angleValue*180.0f)/(double)M_PI); }

// ready mode, run mode, direction decision, transmit message	
private:
	int directionMsg = 0;
	
	const int max_vHand_arr = 10;	// <------ could be customized
	vector<Point> vHandPos;
	vector<int> vHandPos_z;
	vector<float> vRadius;
//ready
	void detectionControlMode(int fingerCount);
	
	bool outOfRange();
	
	void handDetected();
	
	void handNonDetected();
//run	
	void runMode();
	
	void direction();		//direction decision
	
	void keyMessageTX();	//transmitting

// display
private:
	void infoOnDisplay(int message);
	//display is 8bit register, so that binary image (128x56) 
	// should be converted into 8bit register sets (128x7)
	void imageOnDisplay(InputArray _src);	//without display : 0.3ms
};



class Hand{
public:
	Point pos;			//x, y
	int distance;		//z
	float fdistance;	//z (float)
	double palmRadius;
	double palmSize;	
	
	Hand(Point pos, double palmRadius, float fdistance){
		this->pos = pos;
		this->palmRadius = palmRadius;
		this->fdistance = fdistance;
		
		this->distance = cvRound(255.0f/2.0f*fdistance);
		
		this->palmSize = (double)distance*palmRadius;
	}
	
	//sort uppermost values at first
	bool operator <(const Hand &a) const{
		return this->pos.y < a.pos.y;
	}
};

#endif

