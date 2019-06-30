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
#include <omp.h>		//for serial programming
#include "MyListener.h"

enum HANDMODE{ RIGHT_HANDED, LEFT_HANDED, BOTH_HANDED }MODE;

int led[LED_NUMBER];
int numLED;
int move_bit;
int readyGauge;

bool displayOn = false;

bool start_flag = false;
bool run_flag = false;

vector<Point> vHandPos;
vector<float> vRadius;

bool KEY_LEFT_flag, KEY_RIGHT_flag, KEY_DOWN_flag;

void MyListener::onNewData (const DepthData *data)
{
	
		
clock_t startTime = clock(); 
	// this callback function will be called for every new
	// depth frame
	
	//std::lock_guard<std::mutex> lock (flagMutex);

	// create two images which will be filled afterwards
	// each image containing one 32Bit channel
	zImagef.create (Size (data->width, data->height), CV_32FC1);
	grayImagef.create (Size (data->width, data->height), CV_32FC1);

	// set the image to zero
	zImagef = Scalar::all (0);
	grayImagef = Scalar::all (0);
  
	//if=>for (int x,y = 0; x,y < zImagef.rows; x--,y--) (original)
	// get gray and depth information from picoflexx camera
	int k = 0;
	for (int y = zImagef.rows-1; y >=0 ; y--)	// rotated image (180degree)
	{
		float *zRowPtr = zImagef.ptr<float> (y);
		float *grayRowPtr = grayImagef.ptr<float> (y);
		for (int x = 0; x < zImagef.cols; x++, k++)
		{
			auto curPoint = data->points.at (k);
			if (curPoint.depthConfidence > 0)
			{
				if(curPoint.z < MAX_DEPTH){
					zRowPtr[x] = curPoint.z;
					grayRowPtr[x] = curPoint.grayValue;
				}
			}
		}
	}
	
	preProcessing();
	
	//trim only hand ROI by depth value
	myGetHandROI(maskDepthGray, handROI);
	
	//get only contour to get rid of noise inside of the hand
	Mat contour;
	myGetContour(handROI, contour, handROI);		

	//get a center point of the hand and draw a circle
	getHandCenter(handROI);  			
	setHandNum();				// set (determine) the number of hands
	
	//imshow("handROI", handROI);
	
	int fingers = countingNumbers();	
				
	//gaugeControlMode(message);
	detectionControlMode(fingers);
	keyMessageTX();
		
	infoOnDisplay(fingers);
	
	sendingData();
	//BeepSound::beepMode();	
	
cout<<1<<endl;	//success
clock_t endTime = clock();
double codeExcuteTime = ((double)(endTime-startTime))/CLOCKS_PER_SEC;	
//cout<<codeExcuteTime*1000.0<<endl;
}
	
void MyListener::infoOnDisplay(int fingers)
{
	display.clearDisplay();
	
	string strDirectionMsg;
	switch(directionMsg){
	case MSG_LEFT:
		strDirectionMsg = "LEFT";
		break;
	case MSG_RIGHT:
		strDirectionMsg = "RIGHT";
		break;
	case MSG_DOWN:
		strDirectionMsg = "DOWN";
		break;
	}
	string text = "Counting : "+to_string(fingers)+" "+strDirectionMsg+"\n"; 		
	
	if( !displayOn )
	{
		display.setTextSize(1);
		display.setTextColor(WHITE);
		display.printf("%s", text.c_str());
		
		if(readyGauge==READY)
			imageOnDisplay(trackMat);
		else
			imageOnDisplay(handROI);
		
		display.display();
	} 
	else
	{
		display.setTextSize(1);
		display.setTextColor(WHITE);
		
		string pos = "";			
		for(int i=0; i<numOfHand; i++)
		{
			string strhandNum = to_string(i+1) + ">";
			string strPosX = " x:" + to_string(vHand.at(i).pos.x);
			string strPosY = " y:" + to_string(vHand.at(i).pos.y);
			string strPosZ = " z:" + to_string(vHand.at(i).distance);
			pos += (strhandNum + strPosX + strPosY + strPosZ + "\n");	
		}
		
		display.printf("%s", (text+pos).c_str());
		display.display();
	}
}

//display is 8bit register, so that binary image (128x56) 
// should be converted into 8bit register sets (128x7)
void MyListener::imageOnDisplay(InputArray _src)	//without display : 0.3ms
{
	Mat src = _src.getMat();
	
	//display size is 128x64 oled, but first line is empty on display
	resize(src, src, Size(128, 56));	
	
	unsigned int buffer[src.cols] = {0x00};
	for(int y = 0; y < src.rows; y++){		
		uchar *yPtr = src.ptr<uchar> (y);		
		if(y % 8 == 0) memset(buffer, 0x00, sizeof buffer );
		
		for(int x = 0; x < src.cols; x++)
		{	
			if(yPtr[x] > 0)	buffer[x] |= (1 << (y % 8));
			//first line is empty on display, so y/8 + 1
			if(y % 8 == 7) display.setBuffer(x, y/8 + 1, buffer[x]);
		}	
	}
}
	
	
	
void MyListener::preProcessing()	// grayNormalize+depthNormalize+mergedFrames
{
	
	float avg_src = myGetAvgImage1f(grayImagef);
	
	//formulate a correlation between the average value of gray float Image and the value of object 
	float coefficient;
	if(avg_src<0.3)
		coefficient=48.0;
	else if(avg_src<1)
		coefficient = 59.29f-39.29f*avg_src;
	else if(avg_src<2.5)
		coefficient = 30.33f-8.33f*avg_src;
	else if(avg_src<7.5)
		coefficient = 11.135f-0.654f*avg_src;
	else if(avg_src<30)
		coefficient = 7.003f-0.102f*avg_src;
	else if(avg_src<120)
		coefficient = 4.441f-0.0166f*avg_src;
	else
		coefficient = 2.5f;
		
	//cout<<avg_src<<" "<<coefficient<<" "<<avg_src*coefficient<<endl;	////
		
	maskDepthGray.create(grayImagef.size(), CV_8UC1);
		
	//through the formular, the maximum value in the image is approximated to 255
	for (int y = 0; y < zImagef.rows; y++)
	{
		float *fDepthRowPtr = zImagef.ptr<float> (y);
		float *fGrayRowPtr = grayImagef.ptr<float> (y);
		uchar *maskPtr = maskDepthGray.ptr<uchar> (y);
		
		for (int x = 0; x < zImagef.cols; x++)
		{
			float tempGray = fGrayRowPtr[x]/(avg_src*coefficient)/0.8f*255.0f;
			if(tempGray>127.0f){
				uint tempDepth = (uint)(255.0f/2.0f*(fDepthRowPtr[x]));
				if(tempDepth ==0 || tempDepth>255)
					maskPtr[x] = 255;
				else
					maskPtr[x] = tempDepth;
			}else{
				maskPtr[x] = 255;
			}
		}
	}

	/*for (int y = 0; y < zImagef.rows; y++)
	{
		float *fDepthRowPtr = zImagef.ptr<float> (y);
		float *fGrayRowPtr = grayImagef.ptr<float> (y);
		uchar *maskPtr = maskDepthGray.ptr<uchar> (y);
		
		for (int x = 0; x < zImagef.cols; x++)
		{
			if(fGrayRowPtr[x] == 0)
				maskPtr[x] = 255;
			else
				maskPtr[x] = (uchar)(64.0f/300.0f*(fGrayRowPtr[x]-50.0f));
		}
	}*/
}

void MyListener::midPointCircle(vector<Point>& v_circle, Size size, int handNum)
{
	if(vHand.empty()) return;
//Midpoint circle algorithm
//0.55ms -> 0.35ms 
//take shorter time than with the algorithm findContours of circle
	vector<Point> v_circle_octa[8];
	
	Point center(vHand.at(handNum).pos);
	
	int radius = cvRound(vHand.at(handNum).palmRadius * circleScale);
	
	int x = radius-1, y = 0;
	int dx = 1, dy = 1;
	int err = dx - radius*2;
	
	int width = size.width;
	int height = size.height;
	
	while(x >= y)
	{
		if(center.x + x < width && center.y - y > 0)
			v_circle_octa[0].push_back(Point(center.x + x, center.y - y));
		if(center.x + y < width && center.y - x > 0)
			v_circle_octa[1].push_back(Point(center.x + y, center.y - x));
		if(center.x - y > 0 && center.y - x > 0)
			v_circle_octa[2].push_back(Point(center.x - y, center.y - x));
		if(center.x - x > 0 && center.y - y > 0)
			v_circle_octa[3].push_back(Point(center.x - x, center.y - y));
		if(center.x - x > 0 && center.y + y < height)
			v_circle_octa[4].push_back(Point(center.x - x, center.y + y));
		if(center.x - y > 0 && center.y + x < height)
			v_circle_octa[5].push_back(Point(center.x - y, center.y + x));
		if(center.x + y < width && center.y + x < height)
			v_circle_octa[6].push_back(Point(center.x + y, center.y + x));	
		if(center.x + x < width && center.y + y < height)
			v_circle_octa[7].push_back(Point(center.x + x, center.y + y));

		if(err<=0){
			y++;
			err+=dy;
			dy+=2;
		}else{
			x--;
			dx+=2;
			err+=dx-radius*2;
		}	
	}
	// save all 8 different parts into one v_circle
	for(int octa = 0; octa < 8; octa++)
	{
		if(octa % 2 != 0){	//odd number parts : were saved in stack clockwise
			for(int elem = v_circle_octa[octa].size()-1; elem >=0 ; elem--)
				v_circle.push_back(v_circle_octa[octa][elem]);
		}else{				//even number parts : were saved in stack unclockwise
			for(int elem = 0; elem<v_circle_octa[octa].size() ; elem++)
				v_circle.push_back(v_circle_octa[octa][elem]);
		}
	}
}
	
int MyListener::countingFingers(InputArray _src, OutputArray _dst, int handNum)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), src.type());
	Mat dst = _dst.getMat();
	
	Point center(vHand.at(handNum).pos);
	int distance = vHand.at(handNum).distance; 

	//Mat cImg(src.size(), CV_8U, Scalar(0));		//
	//circle(cImg, center, cvRound(vHand.at(handNum).palmRadius * circleScale), Scalar(255));	//
	
	vector<Point> v_circle;
	midPointCircle(v_circle, Size(src.cols, src.rows), handNum );	
	 
	vector<Point> v_point;		//finger position
	vector<double> v_deg;		//finger angle between finger position and palm center

	//cImg = src&cImg;	//

	int fingerWidth = 0;
	Point start_pt(0,0);	//rising point
	
	// save all crossing points through finger contour
	// v_circle[0] -> v_circle[end] : UNCLOCKWISE from degree=0 (3 o'clock)
	for(int i=0; i<v_circle.size(); i++){
		Point pre_pt = v_circle[i];
		Point pt = v_circle[(i+1)%v_circle.size()];
		
		// all points on circle must be continuous, unless circle invades boundary	
		// pre_pt.x-pt.x>1 => only in case of upside invasion
		// abs(pre_pt.y-pt.y)>1 => in case of left/right invasion
	//	if( pre_pt.x - pt.x > 1 || abs(pre_pt.y - pt.y) > 1)
	//	{
			// out of range function
	//		return -1;
	//	}
						
		//detect points where the value changes 0 -> 255
		if(src.at<uchar>(pre_pt.y, pre_pt.x)==0 && src.at<uchar>(pt.y, pt.x)==255){		
			start_pt = pt;	
		}
		else if(src.at<uchar>(pre_pt.y, pre_pt.x)==255 && src.at<uchar>(pt.y, pt.x)==255){
			fingerWidth++;
		}		
		else if(src.at<uchar>(pre_pt.y, pre_pt.x)==255 && src.at<uchar>(pt.y, pt.x)==0){
		/*
			int fWidthByDis;
			if(distance<45)
				fWidthByDis = cvRound(((double)distance-4.0)/24.0*fingerWidth);
			else
				fWidthByDis = cvRound(((double)distance+36.0)/48.0*fingerWidth);
	
			if(fWidthByDis > 15 && fWidthByDis < 28){
				Point midPt = (v_pt_rEdge[0]+v_pt_fEdge[0])/2;
				double rad = -atan2f((double)(midPt.y-center.y),(double)(midPt.x-center.x) );
				v_deg.push_back(radianToDegree(rad));
				circle(cImg, midPt, 3, Scalar(255), 1);	//
			}
		*/
			if(fingerWidth > 5 && fingerWidth < 35){
				Point midPt;
				if(start_pt == Point(0,0)) 	midPt = pre_pt;
				else 						midPt = (start_pt + pre_pt)/2;
				
				v_point.push_back(midPt);
				double rad = -atan2f((double)(midPt.y-center.y),(double)(midPt.x-center.x) );
				v_deg.push_back(radianToDegree(rad));
			}
			
			fingerWidth = 0;
		}		
	}

	//number of fingers
	int fingerCount = (int)v_deg.size();			
	
	// counting points on arm
	// if the points are below (negative angle - quadrant 3 and 4), it means arm
	double armAngle = 0.0;
	int armCt = 0;
	if(fingerCount>0){		
		for(int i=0; i<fingerCount; i++){
			if( v_deg.at(i) < 0){
				armCt++;
				armAngle = ( armAngle*(armCt-1) + v_deg.at(i) ) / (double)armCt;
			}	
		}
	}
	
	//imshow("cImg", cImg);	//	
	return fingerCount - armCt;
}
	
int MyListener::countingNumbers()
{
	int count[2] = {-1,-1};
	
	int message = -1;

	for(int i=0; i<numOfHand; i++)
	{
		Mat onlyHandROI;	//without wrist
		count[i] = countingFingers(handROI, onlyHandROI, i);	
		//imshow("onlyHandROI", onlyHandROI);
	}
	
	if(numOfHand == 1){
		message = count[0];
	}else if(numOfHand == 2){	
		// upto 10 -> just counting for fingers of two hands
		// over 10 -> one fist with fingers in another hand
		if(count[0] > 0 && count[1] == 0){
			message = 10 + count[0];
		}else if(count[0] == 0 && count[1] > 0){
			message = 10 + count[1];
		}else{
			message = count[0] + count[1];
		}
	}else
		message = -1;
		
	return message;
} 
	

void MyListener::drawCircle(InputArray _src, OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), src.type());
	Mat dst = _dst.getMat();
	
	src.copyTo(dst);

	for(int i=0; i<numOfHand; i++){
		double palmRadius = vHand.at(i).palmRadius;
		circle(dst, vHand.at(i).pos, 2, Scalar(0,255,0), -1);
		circle(dst, vHand.at(i).pos, cvRound(palmRadius*circleScale), Scalar(255,0,0), 1);
		circle(dst, vHand.at(i).pos, cvRound(palmRadius*circleScale), Scalar(255,0,0), 1);
	}	
}
	
////must be more robust
void MyListener::getHandCenter(InputArray _mask)
{
	Mat mask = _mask.getMat();
	
	//거리 변환 행렬을 저장할 변수
	Mat dst;
	distanceTransform(mask, dst, CV_DIST_L2, 3);  //결과는 CV_32SC1 타입
		
	int numMarking = 10;
	int minIdx[2];    //idx[0] : y, idx[1] : x
	
	vHand.clear();
	
	//if arms are too big, then arms could be detected => to avoid this, use only the uppermost point
	for(int i=0; i<numMarking; i++){
		double radius;
		minMaxIdx(dst, NULL, &radius, NULL, minIdx, mask);   //get maximum point and size by 32 float value
		
		Point handPos = Point(minIdx[1],minIdx[0]);
		int distance = maskDepthGray.at<uchar>(minIdx[0],minIdx[1]);
		float fdistance = zImagef.at<float>(minIdx[0],minIdx[1]);

		double palmSize = distance*radius;	
		double erase;
		
		if(800<palmSize && palmSize<1250){			//hand size
			erase = radius*1.0;			
			vHand.push_back(Hand(handPos, radius, fdistance));	
		}else{
			erase = radius/10.0;
		}
		if(minIdx[1]!=-1){
			circle(dst, Point(minIdx[1], minIdx[0]), cvRound(erase*1.1), Scalar(0), -1);
		}	
	}
	
	imshow("dst",dst);
	
	sort(vHand.begin(), vHand.end());
	
	if(MODE == BOTH_HANDED)
		handSegment();
} 
	
	
void MyListener::handSegment()
{	
	Point firstHandPos;
	int difHandPosY = 30;	
	 
	if(vHand.size() > 1){
		auto it = vHand.begin();
		firstHandPos = it->pos;
		it++;
		//there's another hand unless if it's in the near x position(such as wrist)
		//and there must be another hand within a similar y position
		//otherwise, erase it
		for( ; it != vHand.end() ;){
			if( abs(firstHandPos.x - it->pos.x) < it->palmRadius*circleScale*2
				|| abs(firstHandPos.y - it->pos.y) > difHandPosY )
			{
				it = vHand.erase(it);
			}
			else
			{
				++it;
			}
		}
	}
}
	
void MyListener::setHandNum()
{
	numOfHand = vHand.size();
		
	if(MODE == BOTH_HANDED && numOfHand > 2) numOfHand = 2;
	
	if(MODE != BOTH_HANDED && numOfHand > 1) numOfHand = 1;
}



double MyListener::radianToDegree(double angleValue)
{	
	return ( (angleValue*180.0f) / (double)M_PI );	
	
}

void MyListener::myMorphology(InputArray _src, OutputArray _dst ) 
{
	Mat src = _src.getMat();
	_dst.create(src.size(), src.type());
	Mat dst = _dst.getMat();
	
	src.copyTo(dst);
	
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(dst, dst, MORPH_OPEN, element, Point(-1,-1), 1);	
}

void MyListener::myGetHandROI(InputArray _src, OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), src.type());
	Mat dst = _dst.getMat();
	
	//set ROI gray Image from maskDepthGray  
	Mat maskDepthGrayHist;
	int roi_x1 = myGetHistogram(src, maskDepthGrayHist);
	
	myDepthROI(src, dst, roi_x1-15, roi_x1+8);	//trim the image if it's closer than user's body 
	
	//to get rid of noise
	threshold(dst, dst, 127, 255, THRESH_BINARY_INV);
}

void MyListener::myGetContour(InputArray _src, OutputArray _dst, OutputArray _fill)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), src.type());
	Mat dst = _dst.getMat();
	dst = Scalar(0);
	
	//extract only the external contour of hand  
	vector<vector<Point> > contours;
	findContours(src, contours, noArray(), RETR_EXTERNAL, CHAIN_APPROX_NONE);  
	drawContours(dst, contours, -1, Scalar(255), 1);
	
	Mat fill;	//if colored contour is needed
	if(_fill.needed()){
		_fill.create(src.size(), src.type());
		fill = _fill.getMat();
		fill = Scalar(0);
		
		drawContours(fill, contours, -1, Scalar(255), -1);
	}
}

void MyListener::mySplitRGBImage(InputArray _src, Mat* _dst)
{
	Mat src = _src.getMat();
	
	vector<Mat> planesRGB;
	split(src, planesRGB);

	planesRGB[0].copyTo(_dst[0]);
	planesRGB[1].copyTo(_dst[1]);
	planesRGB[2].copyTo(_dst[2]);
}
	
float MyListener::myGetAvgImage1f(InputArray _src)
{
	if (_src.channels() != 1) return 0;       //channel should be 1-channel
	
	Mat src = _src.getMat();
	
	int xSampleGap = 4;
	int ySampleGap = 4;
	int xSampleNum = src.cols/xSampleGap;
	int ySampleNum = src.rows/ySampleGap;
	float sum = 0;
	for(int y=0; y<ySampleNum; y++){
		float *fGrayPtr = src.ptr<float> (y*ySampleGap);
		for (int x = 0; x < xSampleNum; x ++) 
			sum += fGrayPtr[x*xSampleGap];    
	}	

	return sum*(float)xSampleGap*(float)ySampleGap / (float)src.total();
}

float MyListener::myGetAvgImage1u(InputArray _src)
{
	if (_src.channels() != 1) return 0;       //channel should be 1-channel
	
	Mat src = _src.getMat();
	
	int xSampleGap = 4;
	int ySampleGap = 4;
	int xSampleNum = src.cols/xSampleGap;
	int ySampleNum = src.rows/ySampleGap;
	float sum = 0;
	for(int y=0; y<ySampleNum; y++){
		uchar *fGrayPtr = src.ptr<uchar> (y*ySampleGap);
		for (int x = 0; x < xSampleNum; x ++) 
			sum += fGrayPtr[x*xSampleGap];    
	}
	return sum*(float)xSampleGap*(float)ySampleGap / (float)src.total(); 
}

void MyListener::myGrayNormalize(InputArray _src, OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), CV_8UC1);
	Mat dst = _dst.getMat();
	
	float avg_src = myGetAvgImage1f(src);
	
	//formulate a correlation between the average value of gray float Image and the value of object 
	float coefficient;
	if(avg_src<0.3)
		coefficient=48.0;
	else if(avg_src<1)
		coefficient = 59.29f-39.29f*avg_src;
	else if(avg_src<2.5)
		coefficient = 30.33f-8.33f*avg_src;
	else if(avg_src<7.5)
		coefficient = 11.135f-0.654f*avg_src;
	else if(avg_src<30)
		coefficient = 7.003f-0.102f*avg_src;
	else if(avg_src<120)
		coefficient = 4.441f-0.0166f*avg_src;
	else
		coefficient = 2.5f;
		
	//through the formular, the maximum value in the image is approximated to 255
	for (int y = 0; y < src.rows; y++)
	{
		float *fGrayRowPtr = src.ptr<float> (y);
		uchar *uGrayRowPtr = dst.ptr<uchar> (y);
		for (int x = 0; x < src.cols; x++)
		{
			float temp = fGrayRowPtr[x]/(avg_src*coefficient)/0.8f*255.0f;
			if(temp>255) temp = 255;
			uGrayRowPtr[x] = (uchar)temp;
		}
	}	
}

void MyListener::myDepthNormalize(InputArray _src, OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), CV_8UC1);
	Mat dst = _dst.getMat();
	
	for (int y = 0; y < src.rows; y++)
	{
		float* zRowPtr = src.ptr<float> (y);
		uchar* grayRowPtr = dst.ptr<uchar> (y);	
		for (int x = 0; x < src.cols; x++)
		{
			uint temp = (uint)(255.0/2.0*(zRowPtr[x]));
			if(temp == 0 || temp>255)
				temp = 255;
			grayRowPtr[x] = (uchar)temp;
		}
	}
}

void MyListener::myDepthNormalizeRGB(InputArray _src, OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), CV_8UC3);
	Mat dst = _dst.getMat();

	vector<Mat> planesRGB;
	split(dst, planesRGB);

	for (int y = 0; y < src.rows; y++)
	{
		float *zRowPtr = src.ptr<float> (y);
		uchar *bRowPtr = planesRGB[0].ptr<uchar> (y);
		uchar *gRowPtr = planesRGB[1].ptr<uchar> (y);
		uchar *rRowPtr = planesRGB[2].ptr<uchar> (y);
		for (int x = 0; x < src.cols; x++)
		{
			uint temp = (uint)(768/2*zRowPtr[x]);
			uchar bTemp, gTemp, rTemp;
			if(temp == 0){
				bTemp = 255;
				gTemp = 255;
				rTemp = 255;
			}else if(temp<256){	//0~254
				bTemp = (uchar)temp;
				gTemp = 0;
				rTemp = 0;
			}else if(temp<512){	//255~509
				bTemp = 255;
				gTemp = (uchar)(temp-256);
				rTemp = 0;
			}else if(temp<768){	//510~764
				bTemp = 0;
				gTemp = 255;
				rTemp = (uchar)(temp-512);
			}else{
				bTemp = 255;
				gTemp = 255;
				rTemp = 255;
			}
			bRowPtr[x] = bTemp;
			gRowPtr[x] = gTemp;
			rRowPtr[x] = rTemp;
		}
	}
	merge(planesRGB, dst);
}

void MyListener::myMergeFrames(InputArray _src1, InputArray _src2, OutputArray _mask)
{
	Mat src1 = _src1.getMat();		//RGB Image or Gray Image
	Mat src2 = _src2.getMat();		//binary Image
	_mask.create(src1.size(), src1.type());
	Mat mask = _mask.getMat();

	vector<Mat> _planesRGB;
	split(src1, _planesRGB);
	
	vector<Mat> planesRGB;
	split(mask, planesRGB);

	for (int y = 0; y < src1.rows; y++)
	{	
		int numCh = src1.channels();	//num of channels
		uchar* tempSrc1[numCh];
		uchar* tempMask[numCh];
		for(int ch=0; ch<numCh; ch++){
			tempSrc1[ch] = _planesRGB[ch].ptr<uchar> (y);
			tempMask[ch] = planesRGB[ch].ptr<uchar> (y);
		}
		uchar *grayRowPtr = src2.ptr<uchar> (y);
		for (int x = 0; x < src1.cols; x++)
		{
			if(grayRowPtr[x]==255){
				for(int ch=0; ch<numCh; ch++)
					tempMask[ch][x] = tempSrc1[ch][x];
			}else{
				for(int ch=0; ch<numCh; ch++)
					tempMask[ch][x] = 255;
			}
		}
	}
	merge(planesRGB, mask);
}
	

////must be more robust
int MyListener::myGetHistogram(InputArray _src, OutputArray _hist)
{
	Mat src = _src.getMat();
	
	Mat histImage;
	if(_hist.needed()){
		_hist.create(512,512,CV_8U);
		histImage = _hist.getMat();
	}else{
		histImage = Mat(512,512,CV_8U);
	}
		
	Mat hist;
	int channels = 0;
	int histSize = 256;
	float valueRange[] = {0.0,255.0};
	const float* ranges[] = {valueRange};
	calcHist(&src, 1, &channels, Mat(), hist, 1, &histSize, ranges);   
	//normalize(hist,hist,0,histImage.rows,NORM_MINMAX,CV_32F);
	
	histImage = Scalar(255);
	int binW = histImage.cols/histSize;
	Point max_pt1, max_pt2;
	int peak_y = -1;
	//int peak_x = -1;
	for(int i=0; i<histSize/2; i++){
		float area = hist.at<float>(i);
		
		float formular;
		if(i<50) formular = -425*((float)i/2-59.41f);
		else if(i<histSize/2) formular = -100*((float)i/2-80);
		float calib1 = area/formular;
		float coeff = 100.0;

		float calib = calib1*coeff;
		int y2 = histImage.rows-cvRound(calib);
		
		Point pt1(i*binW, histImage.rows);
		Point pt2((i+1)*binW, y2);		
		rectangle(histImage, pt1, pt2, Scalar(127), -1);
		
		//handsize + maximum value of hist + hand region range
		if((pt1.y-pt2.y)<200/2 && (pt1.y-pt2.y)>peak_y ){
			if(i<30) temp_move_bit = CLOSE;
		//	else if(i>55) temp_move_bit = FAR;
			//peak_x = i;
			peak_y = pt1.y-pt2.y;
			max_pt1 = Point(pt1);
			max_pt2 = Point(pt2);
		}
	}
	rectangle(histImage, max_pt1, max_pt2, Scalar(0), -1);
	return max_pt1.x/(histImage.cols/256);
}
	
void MyListener::myDepthROI(InputArray _src, OutputArray _dst, int min, int max)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), CV_8UC1);
	Mat dst = _dst.getMat();
	
	for (int y = 0; y < src.rows; y++)
	{
		uchar* grayRowPtr = src.ptr<uchar> (y);
		uchar* maskRowPtr = dst.ptr<uchar> (y);	
		for (int x = 0; x < src.cols; x++)
		{
			uchar temp = grayRowPtr[x];
			if(temp>min && temp<max)
				maskRowPtr[x] = temp;
			else 
				maskRowPtr[x] = 255;
		}
	}
}




void MyListener::detectionControlMode(int fingerCount)
{
	if(fingerCount >= 0 && fingerCount <= 5)	//detected
		handDetected();
	else 										//non detected
		handNonDetected();
}

void MyListener::handDetected()
{
	static int readyCount;
	if(!run_flag)
	{	
		if(readyCount < 5)
		{
			if(vHand.empty()) return;
			
			static int pre_distance = 255;
			
			if(pre_distance+2 < vHand.at(0).distance)
				readyCount++;
			else
				readyCount = 0;
			
			pre_distance = vHand.at(0).distance;
		}
		else
		{	
			run_flag = true;
			readyCount = 0;
		}
	}
	else
	{
		runMode();
	}
}

void MyListener::handNonDetected()
{
	if(run_flag)
	{
		if(readyGauge > 0)
		{
			if(readyGauge == READY)
				if(vHandPos.size() > 5)
					direction();
			readyGauge--;
		}
		else
		{
			run_flag = false;
			readyGauge = 0;
		}
	}
}

void MyListener::runMode()
{
	trackMat.create(zImagef.size(), CV_8UC1);
	trackMat = Scalar(0);
	
	if(vHand.empty()) return;
	
	if(readyGauge != READY)
	{
		readyGauge = READY;
		
		//make vector emptied
		vHandPos.clear();
		vRadius.clear();
	}
	
	//vector push_back
	if(vHand.empty()) return;
	float radius = (65.0f-vHand.at(0).distance)/2.5f;	//distance

	int x = vHand.at(0).pos.x;
	int y = vHand.at(0).pos.y;
	
	//avoid closed at boundaries
	int boundTol = 10; //boundary tolerance
	if( (x>boundTol && x<zImagef.cols-boundTol) && (y>boundTol && zImagef.rows-boundTol) ){
		if( vHandPos.size() < 10 ){	// when HandPos vector has less than 10 arrays 
			vHandPos.push_back(vHand.at(0).pos);
			vRadius.push_back(radius);
		}else{						// when HandPos vector has enough 10 arrays 
			vHandPos.erase(vHandPos.begin());	//update while deleting an oldest value 
			vHandPos.push_back(vHand.at(0).pos);// ,adding a newest value 
			
			vRadius.erase(vRadius.begin());
			vRadius.push_back(radius);
		}
	}

	for(int i=0; i<vHandPos.size(); i++)
	{	
		circle(trackMat, vHandPos[i], vRadius[i], Scalar(64+i*19), -1);
		if(i!=0)
			arrowedLine(trackMat, vHandPos[i-1], vHandPos[i], Scalar(255), 1);
	}
	//imshow("trackMat", trackMat);
}

void MyListener::direction()
{	
	int xCt = 0, yCt = 0;
	Point difPos = vHandPos[vHandPos.size()-1] - vHandPos[vHandPos.size()-6];
	for(int i=vHandPos.size()-1; i>vHandPos.size()-6; i--)
	{
		if(vHandPos[i].x - vHandPos[i-1].x > 0) xCt++;	//moving leftward
		else if(vHandPos[i].x - vHandPos[i-1].x < 0) xCt--;	  //rightward
		
		if(vHandPos[i].y - vHandPos[i-1].y > 0) yCt++;	//moving upward
		else if(vHandPos[i].x - vHandPos[i-1].x < 0) yCt--;	  // downward
	}
		
	if(abs(difPos.x) > 40 && abs(difPos.y) < 30)
	{
		if(xCt>=5){
			directionMsg = MSG_RIGHT;
			KEY_RIGHT_flag = true;
			//cout<<"Right"<<endl;
		}
		else if(xCt<=-5){
			directionMsg = MSG_LEFT;
			KEY_LEFT_flag = true;
			//cout<<"Left"<<endl;
		}
	}
	else if(abs(difPos.x) < 30 && abs(difPos.y) > 30)
	{
		if(yCt>=3){
			directionMsg = MSG_DOWN;
			KEY_DOWN_flag = true;
			//cout<<"Down"<<endl;
		}
		//else if(yCt<=-3) cout<<"Up"<<endl;
	}	
}

void MyListener::keyMessageTX()
{
	digitalWrite(KEY_LEFT, HIGH);	
	digitalWrite(KEY_RIGHT, HIGH);	
	digitalWrite(KEY_DOWN, HIGH);	
	
	softToneWrite(BEEP,0);			//// turn off after one frame
	
	static bool pre_run_flag;
	if(!pre_run_flag && run_flag) softToneWrite(BEEP, 5120);
	pre_run_flag = run_flag;
	
	if(KEY_RIGHT_flag){
		digitalWrite(KEY_RIGHT, LOW);
		softToneWrite(BEEP,5120);	//// without using BeepSound class
		KEY_RIGHT_flag = false;
	}
	if(KEY_LEFT_flag){
		digitalWrite(KEY_LEFT, LOW);
		softToneWrite(BEEP,5120);	////
		KEY_LEFT_flag = false;
	}
	if(KEY_DOWN_flag){
		digitalWrite(KEY_DOWN, LOW);
		softToneWrite(BEEP,5120);	////
		KEY_DOWN_flag = false;
	}
}

	
// range warning
bool MyListener::outOfRangeBit(Point range, int handNum)
{	
	Point center = vHand.at(handNum).pos;
	double radius = vHand.at(handNum).palmRadius;
	int centerDistance = vHand.at(handNum).distance;
	
	if(centerDistance<33){
		temp_move_bit = CLOSE;
		return false;
	}else if(centerDistance>58){
		temp_move_bit = FAR;
		return false;
	}

	if(center.x-radius*circleScale<0){
		temp_move_bit = RIGHT;
		return false;
	}else if(center.x+radius*circleScale>range.x){
		temp_move_bit = LEFT;
		return false;
	}else if(center.y-radius*circleScale<0){
		temp_move_bit = DOWN;
		return false;
	}else if(center.y+radius*circleScale>range.y){
		temp_move_bit = UP;
		return false;
	}else{
		temp_move_bit = DETECTION;
		return true;
	}
}

//outofRangeMessage
void MyListener::outOfRangeMessage(int message)
{//R=2, D, L, U, C, F, NO_DETECTION, DETECTION
	move_bit = message;
	
	switch(move_bit){
	case RIGHT:
		cout<<"Please move right"<<endl;
		break;
	case LEFT:
		cout<<"Please move left"<<endl;
		break;
	case DOWN:
		cout<<"Please move down"<<endl;
		break;
	case UP:
		cout<<"Please move up"<<endl;
		break;
	case CLOSE:
		cout<<"Too close!"<<endl;
		break;
	case FAR:
		cout<<"Too far!"<<endl;
		break;
	}
}


void MyListener::ctrIlluminance(int numLED)
{
	// fdistance 0.25 ~ 0.4 => 0 ~ 100 %
	if(!vHand.empty()){		
		float fdistance = vHand.at(0).fdistance;
		int illuminance = cvRound( 100.0/0.15*(fdistance-0.25) );
		
		if(illuminance > 100) illuminance = 100;
		else if(illuminance < 0) illuminance = 0;
		
		if(abs(led[numLED] - illuminance) < 15)
			led[numLED] = illuminance;
	}
}

	
void MyListener::sendingData()	//sending LED illuminance
{	//brightness of led + number of led + readyGauge + move_bit 
	if(!start_flag)
	{
		unsigned char data[4];
		for(int i=0; i<4; i++){
			data[0] = (unsigned char)led[i];
			data[1] = (unsigned char)i;
			data[2] = 0;
			data[3] = NO_DETECTION;

			//Bluetooth::transmitting(data, 4);
		}
		start_flag = true;
	}
	
	static unsigned char data[4];
	bool flag = false;
	
	//see if the led brightness is changed
	if(data[0]!=(unsigned char)led[numLED]) flag = true;
	data[0] = (unsigned char)led[numLED];
	
	//see if the led num is changed
	if(data[1]!=(unsigned char)numLED) flag = true;
	data[1] = (unsigned char)numLED;
	
	//see if the readyGauge is changed
	if(data[2]!=(unsigned char)readyGauge) flag= true;
	data[2] = (unsigned char)readyGauge;
	
	//see if the move bit is changed
	if(data[3]!=(unsigned char)move_bit) flag= true;
	data[3] = (unsigned char)move_bit;
		
	//sending data only if the data is changed
	if(flag){
		//Bluetooth::transmitting(data, 4);
		//for(int i=0; i<4; i++) cout<<+data[i]<<" ";
	}
	//cout<<endl;
}


