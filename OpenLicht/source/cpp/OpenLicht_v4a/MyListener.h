/****************************************************************************\
 * Copyright (C) 2017 Infineon Technologies & pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  
 * 
 * Overwritten by Seung-Gu Kang (onNewData class) 
 * 				in OpenLicht version 3d ( March, 2019 )
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

#include "bluetooth/Bluetooth.h"
#include "sound/BeepSound.h"
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

#define MAX_DEPTH 0.5
#define LED_NUMBER 4
#define READY 20

enum PINS{PINLEFT=21, PINUP, PINPRESS, PINDOWN, PINRIGHT, KEY3=27, KEY2};
enum INPUT_PINS{lowBatPin=0, battery0=2, battery1=3};	//0th bit, //1st bit

enum PINSOUND{BEEP=26};

enum RANGEBIT{RIGHT=2, DOWN, LEFT, UP, CLOSE, FAR, NO_DETECTION, DETECTION};

extern ArduiPi_OLED display;

class Hand;

class MyListener : public IDepthDataListener
{
public:
	MyListener() : undistortImage (false){ }

    void onNewData (const DepthData *data);
	
 	//display is 8bit register, so that binary image (128x56) 
	// should be converted into 8bit register sets (128x7)
	void imageOnDisplay(InputArray _src);	//without display : 0.3ms
	
	void preProcessing();	// grayNormalize+depthNormalize+mergedFrames
	
	int countingFingers(InputArray _src, OutputArray _dst, int handNum);
	
	void midPointCircle(vector<Point>&, Size, int handNum);
	
	int countingNumbers();
	
	void drawCircle(InputArray _src, OutputArray _dst);
	
	////must be more robust
	void getHandCenter(InputArray _mask);
	
	void handSegment();
	
	void setHandNum();
	

public:    
    void setLensParameters (LensParameters lensParameters);
    void toggleUndistort();

private:
    // define images for depth and gray
    // and for their 8Bit and scaled versions   
    Mat zImagef, grayImagef;
    Mat depthRGB, depthGray, grayImage, binaryImage; 
    
    Mat maskDepthRGB, maskDepthGray;
    
    Mat handROI;		//with wrist
    
    
	vector<Hand> vHand;
	
	double circleScale = 2.0;
 
	int numOfHand;
	
	double indexFinger_deg = 0.0;
    
    int temp_move_bit;
    
    // lens matrices used for the undistortion of
    // the image
    Mat cameraMatrix;
    Mat distortionCoefficients;

    std::mutex flagMutex;
    bool undistortImage;
    
	double radianToDegree(double angleValue);
	
	void myMorphology(InputArray _src, OutputArray _dst);
	
	void myGetHandROI(InputArray _src, OutputArray _dst);
	
	void myGetContour(InputArray _src, OutputArray _dst, OutputArray _fill = noArray() );
	
	void mySplitRGBImage(InputArray _src, Mat* _dst);
	
	float myGetAvgImage1f(InputArray _src);
	
	float myGetAvgImage1u(InputArray _src);
	
	void myGrayNormalize(InputArray _src, OutputArray _dst);
	
	void myDepthNormalize(InputArray _src, OutputArray _dst);
	
	void myDepthNormalizeRGB(InputArray _src, OutputArray _dst);
	
	void myMergeFrames(InputArray _src1, InputArray _src2, OutputArray _mask);
	
	////must be more robust
    int myGetHistogram(InputArray _src, OutputArray _hist = noArray() );
	
	void myDepthROI(InputArray _src, OutputArray _dst, int min=0, int max=255);
	
private:
	//mode control (pre_ready -> ready -> run)	
	void gaugeControlMode(int fingerCount);
	
	bool readyMode_flag = false;
	bool pre_readyMode_flag = false;
	
	void pre_readyMode(int fingerCount);
	
	void readyMode(int fingerCount);
	
	void runMode(int fingerCount);
	
	void ctrIlluminance(int numLED);
	
	void displayMode(int message);

	string soundDisplay();
	
	string batteryDisplay();
	
	void sendingData();
	
private:	// range warning
	bool outOfRangeBit(Point range, int handNum);

	//outofRangeMessage
	void outOfRangeMessage(int message);
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

