/****************************************************************************\
 * Copyright (C) 2017 Infineon Technologies & pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include <royale.hpp>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>

#include <sample_utils/PlatformResources.hpp>
#include "Bluetooth.h"
#include "BeepSound.h"

#include <wiringPi.h>
#include <softTone.h>
#include <time.h>

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
#define BEEP 29 //GPIO 21(PIN 40)

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

bool start_flag;

class MyListener : public IDepthDataListener
{
public:
	MyListener() :
        undistortImage (false)
    {
    }
    
    void onNewData (const DepthData *data)
    {
	clock_t startTime = clock();
	    // this callback function will be called for every new
        // depth frame
        
        std::lock_guard<std::mutex> lock (flagMutex);

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
			for (int x = zImagef.cols-1; x >=0 ; x--, k++)
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
		/*
        //create images to store the 8Bit version (some OpenCV functions may only work on 8Bit images)
        //normalize zImage(float) to gray space with 0~255(uchar) 
        myDepthNormalize(zImagef, depthGray); 
      //imshow("DepthGray", depthGray);         
    
		//normalize grayImage(float) to 8Bit gray scale Image in avg scale 
        myGrayNormalize(grayImagef, grayImage);	//regulate the amount of light (gray)   
        //imshow ("Gray", grayImage);
       
        //set a threshold of the amount gray Image value(light)
        threshold(grayImage, binaryImage, 127, 255, THRESH_BINARY); //set the amount of light (gray)
        //imshow("BinaryImage", binaryImage);
      
        //8bit DepthGray + binary image        
        myMergeFrames(depthGray, binaryImage, maskDepthGray);
        //imshow("MaskDepthGray", maskDepthGray);
		*/
		//trim only hand ROI by depth value
		Mat handROI;		//with wrist
		myGetHandROI(maskDepthGray, handROI);			
	
		//get only contour to get rid of noise inside of the hand
		Mat contour;
		myGetContour(handROI, contour, handROI);		
		//imshow("Contour", contour);
		//imshow("handROI", handROI);

        //get a center point of the hand and draw a circle
		getHandCenter(handROI);  			////
		
		Mat onlyHandROI;	//without wrist
		int count = eliminateArm(handROI, onlyHandROI);	////
		imshow("onlyHandROI", onlyHandROI);
  	
		gaugeControlMode(count);
		sendingData();
	
		BeepSound::beepMode();
		
	clock_t endTime = clock();
	double codeExcuteTime = ((double)(endTime-startTime))/CLOCKS_PER_SEC;	
	cout<<codeExcuteTime*1000.0<<endl;  	
	}
	
	void preProcessing()	// grayNormalize+depthNormalize+mergedFrames
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
				if(tempGray>127){
					uint tempDepth = (uint)(255.0/2.0*(fDepthRowPtr[x]));
					if(tempDepth ==0 || tempDepth>255)
						maskPtr[x] = 255;
					else
						maskPtr[x] = tempDepth;
				}else{
					maskPtr[x] = 255;
				}
			}
		}	
	}

	int eliminateArm(InputArray _src, OutputArray _dst, double scale=1.8)
	{
		Mat src = _src.getMat();
		_dst.create(src.size(), src.type());
		Mat dst = _dst.getMat();
		src.copyTo(dst);

		//draw a circle 
		Mat cImg(src.size(), CV_8U, Scalar(0));
		circle(cImg, center, cvRound(radius * scale), Scalar(255));
		
		//save the contour of circle to vector 
		vector<vector<Point>> contours;
		findContours(cImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		
		//no hand contour
		if(contours.size()==0){  
			move_bit = temp_move_bit = NO_DETECTION;
			return -1;
		}
		
		//hand is out of range
		if(!outOfRangeBit(Point(src.cols,src.rows), scale)) ////
			return -1;
		
		//외곽선을 따라 돌며 mask의 값이 0에서 1로 바뀌는 지점 확인	
		vector<Point> v_pt;		//finger position
		vector<float> v_deg;	//finger angle between finger position and palm center
		vector<pair<float,int> > v_difDeg;	//difference of angle and number
		
		cImg = src&cImg;
		bool rightArmFlag = false;
		for(int i=1; i<contours[0].size(); i++){
			Point p1=contours[0][i-1];
			Point p2=contours[0][i];
			//detect points where the value changes 0 -> 255
			if(src.at<uchar>(p1.y, p1.x)==0 && src.at<uchar>(p2.y, p2.x)>1){
				float rad = -atan2f((float)(p2.y-center.y),(float)(p2.x-center.x) );
				float degree = radianToDegree(rad);	
				
				circle(cImg, p1, 5, Scalar(255), 1);
				v_pt.push_back(p1);
				v_deg.push_back(degree);
			
				if(degree>-90 && degree<20) rightArmFlag = true;
			}		
		}
		if(!rightArmFlag) return -1;	//return if it's not a right arm 
		
	//imshow("cImg", cImg);
		
		//number of fingers
		int fingerCount = (int)v_deg.size();	
		
		//if it's right-hand, the thumb angle must be around 70~170 degree depending on the angle of hand
		//but contours starts from 90 degree and save into v_deg vector, 
		//so if the thumb angle is lower than 90 degree, it's saved the last vector not first
		//this makes the thumb angle saved at the first of v_deg vector,
		//because the thumb must be saved at the first
		for(int i=0; i<fingerCount; i++){  	
			if(v_deg[i]<90 && v_deg[i]>45){
				float deg_tmp = v_deg[i];
				v_deg.erase(v_deg.begin()+i);
				v_deg.insert(v_deg.begin(),deg_tmp);
				
				Point pt_tmp = v_pt[i];	//pair with v_deg
				v_pt.erase(v_pt.begin()+i);
				v_pt.insert(v_pt.begin(),pt_tmp);
			}
		}
		
		//set illuminance by calculating an angle between thumb and second finger
		if(v_deg.size()>1) indexFinger_deg = v_deg[1];
		
		//sort v_deg and v_difDeg vectors and convert the angle -180~180 into 0~360
		sort(v_deg.begin(), v_deg.end()); 
		for(int i=0; i<fingerCount; i++){
			float tmp;	
			if(i+1==fingerCount) tmp = v_deg[0]-v_deg[i];
			else tmp = v_deg[i+1]-v_deg[i];
			if(tmp<0) tmp+=360;
			v_difDeg.push_back(make_pair(tmp, i));
		}	
		sort(v_difDeg.begin(), v_difDeg.end());  
	
	//v_difDeg가 가장 큰 값과 두번째 큰 양 끝점 좌표들을 나열할 경우, 한점이 겹치게 되면 그 점이 손목
	//if the maximum and second maximum v_difDeg value share the same point from each end points
	//it means that the same point is only away from the other points => wrist		
		float armAngle = 0.0f;
		if(fingerCount>2){		//only when more than 2 fingers
			int scoreArray[fingerCount]={0};
			int score1 = v_difDeg[fingerCount-1].second;
			int score2 = v_difDeg[fingerCount-2].second;
			
			if(v_difDeg[fingerCount-1].first>70){
				scoreArray[score1]++;
				int temp1 = score1+1;
				if(temp1==fingerCount) temp1 = 0;
				scoreArray[temp1]++;
			}
			if(v_difDeg[fingerCount-2].first>70){ 
				scoreArray[score2]++;
				int temp2 = score2+1;
				if(temp2==fingerCount) temp2 = 0;
				scoreArray[temp2]++;
			}
			for(int i=0; i<fingerCount; i++){
				if(scoreArray[i]==2)
					armAngle = v_deg.at(i);
			}
		}else if(fingerCount==2){	//in case of just 1 finger (for right hand user)
			if(v_deg[0]<90 && v_deg[0]>-90)
				armAngle = v_deg.at(0);
			else
				armAngle = v_deg.at(1);
		}else{						//in case of a fist
			armAngle = v_deg.at(0);
		}
		
		//eliminate arm
		Mat erase(src.size(), CV_8UC1, Scalar(255));
		if(armAngle!=0.0){
			ellipse(erase, center, Size(radius*8,radius*8), -armAngle, -90, 50, Scalar(0), -1, CV_AA);
			ellipse(erase, center, Size(radius*1.05,radius*1.05), -armAngle, -90, 50, Scalar(255), -1, CV_AA);
		}
		dst = dst&erase;
		threshold(dst, dst, 3, 255, THRESH_BINARY);
		
		return fingerCount - 1;
	}
	
	void drawLines(InputArray _src, OutputArray _dst)
	{
		Mat src = _src.getMat();
		_dst.create(src.size(), src.type());
		Mat dst = _dst.getMat();

		dst = Scalar(0);
		
		vector<Vec4i> lines;  
		HoughLinesP( src, lines, 1, CV_PI/180,  30, 30, 3);  
	  
		//검출한 직선을 영상에 그려줍니다.  
		for(uint i=0; i<lines.size(); i++ )  
		{  
			Vec4i L = lines[i];  
			line(dst, Point(L[0],L[1]), Point(L[2],L[3]),  
				 Scalar(255), 1, LINE_AA );  
		}  
	}

	void drawCircle(InputArray _src, OutputArray _dst)
	{
		Mat src = _src.getMat();
		_dst.create(src.size(), src.type());
		Mat dst = _dst.getMat();
		
		src.copyTo(dst);
   
		circle(dst, center, 2, Scalar(0,255,0), -1);
		circle(dst, center, cvRound(radius), Scalar(255,0,0), 1);
		circle(dst, center, cvRound(radius*1.8), Scalar(255,0,0), 1);
	}
	////must be more robust
	void getHandCenter(InputArray _mask)
	{
		Mat mask = _mask.getMat();
		
		//거리 변환 행렬을 저장할 변수
		Mat dst;
		distanceTransform(mask, dst, CV_DIST_L2, 3);  //결과는 CV_32SC1 타입
		
		//거리 변환 행렬에서 값(거리)이 가장 큰 픽셀의 좌표와, 값을 얻어온다.
		//cout<<endl;
		int numMarking = 10;
		Point leftmostIdx(dst.cols, dst.rows);
		int minIdx[2];    //좌표 값을 얻어올 배열(행, 열 순으로 저장됨)
		//if arms are too big, then arms could be detected => to avoid this, use only the leftmost point
		for(int i=0; i<numMarking; i++){
			double temp_radius;
			minMaxIdx(dst, NULL, &temp_radius, NULL, minIdx, mask);   //최댓값만 얻어옴
			
			int grayValue = maskDepthGray.at<uchar>(minIdx[0],minIdx[1]);
			float size = grayValue*temp_radius;	
			
			double erase;	//int scale
			//cout<<size<<endl;
			if(1700/2<size && size<2700/2){			//hand size
				erase = temp_radius;
				
				if(minIdx[1] < leftmostIdx.x){	//the leftmost point
					leftmostIdx = Point(minIdx[1],minIdx[0]);
					radius = temp_radius;
				}
			}else{
				erase = temp_radius/10;
			}
			if(minIdx[1]!=-1){
				circle(dst, Point(minIdx[1],minIdx[0]), cvRound(erase*1.1), Scalar(0), -1);
			}	
		}
	//imshow("dst",dst);
		if(leftmostIdx==Point(dst.cols, dst.rows)) radius = 0;
		
		center = leftmostIdx;	//get center of palm
		centerDistance = maskDepthGray.at<uchar>(center.y,center.x);
	} 
	////noise must not show up
	void mySkeleton(InputArray _src, OutputArray _dst)
	{
	////skeleton	
		Mat mask = _src.getMat();
		Mat src = mask.clone();
		_dst.create(src.size(), CV_8UC1);
		Mat dst = _dst.getMat();
		dst = Scalar(0);
		
		Mat temp;
		Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
		bool done;
		do{
			cv::morphologyEx(src, temp, cv::MORPH_OPEN, element);
			cv::bitwise_not(temp, temp);
			cv::bitwise_and(src, temp, temp);
			cv::bitwise_or(dst, temp, dst);
			cv::erode(src, src, element);

			double max;
			cv::minMaxLoc(src, 0, &max);
			done = (max == 0);
		} while (!done);
		morphologyEx(dst, dst, cv::MORPH_CLOSE, element);
	}
    void findAndDrawContours(Mat & thresholdedImage)
	{
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		double maxArea = 0;
		double tempArea = -1;
		int tempIndex = 0;
		vector<Point> maxContour;
		vector<Point> hull;
		vector<vector<int> >hull_1;
		
	//	cout << "Before 1" << endl;
	//	imshow("TH functions", thresholdedImage);
		findContours(thresholdedImage, contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
	//	cout << "Before 2" << endl;	
		
		Mat drawing = Mat::zeros(thresholdedImage.size(), CV_8UC3);
		int size = contours.size();
		
		if(size > 0)
		{
	//		cout << "Size > 0" << endl;
			for(int i = 0; i < size; i++)
			{
	//			cout << "Petla" << endl;
				vector<Point> tempContour = contours[i];
				tempArea = contourArea(tempContour);
				if(tempArea > maxArea)
				{
					maxArea = tempArea;
					tempIndex = i;
				}
			}
			maxContour = contours[tempIndex];
			
			vector<vector<Point>> tmp_max;
			tmp_max.push_back(maxContour);

			vector<vector<int> >hull_1(tmp_max.size());
			for(uint i = 0; i < tmp_max.size(); i++)
			{
				convexHull(tmp_max[i], hull_1[i], false );
			}
			
			convexHull(contours[tempIndex], hull, CV_CLOCKWISE, false);

			vector<vector<Point>> tmp;
			tmp.push_back(hull);

			vector<vector<Point>> contours_play(1);
			Rect boundRect;
			
			approxPolyDP(Mat(tmp_max[0]), contours_play[0], 3, true);
			boundRect = boundingRect(Mat(contours_play[0]));
			
			//contours
			drawContours(drawing, tmp_max, 0, Scalar(0, 255, 0), 2);
			//bounding rectangle
			rectangle(drawing, boundRect.tl(), boundRect.br(), Scalar(255, 0, 0), 2, 8, 0);
			////draw hull of the biggest object
			drawContours(drawing, tmp, 0, Scalar(0, 0, 255), 3);
			
			//vector< <vector<Vec4i> > defects(1);
			std::vector< std::vector<Vec4i> > defects( tmp_max.size() );
			for(uint j = 0; j < tmp_max.size(); j++)
				if(tmp_max[j].size() > 3)
				{
	//				cout << "convexityDefects" << endl;
					convexityDefects(tmp_max[j], hull_1[j], defects[j]);
	//				cout << "defects.size() "<< to_string(defects.size()) << "\n";
	//				cout << "defects.size() "<< to_string(defects[0].size()) << "\n";
				}
					
			vector<Vec4i>::iterator d =defects[0].begin();
			
			vector<Point> pinPoint_temp;

			while( d!=defects[0].end() ) {
				Vec4i& v=(*d);

				int startidx=v[0]; 
				Point ptStart( tmp_max[0][startidx] ); // point of the contour where the defect begins
				int endidx=v[1]; 
				Point ptEnd( tmp_max[0][endidx] ); // point of the contour where the defect ends
				int faridx=v[2]; 
				Point ptFar( tmp_max[0][faridx] );// the farthest from the convex hull point within the defect
				float depth = (float)v[3] / 256; // distance between the farthest point and the convex hull

				if(depth > 10 && depth < 80)
				{
					line( drawing, ptStart, ptFar, CV_RGB(0,255,0), 2 );
					line( drawing, ptEnd, ptFar, CV_RGB(0,255,0), 2 );
					circle( drawing, ptStart,   4, Scalar(100,0,255), 2 );
				}
				
				d++;
			}

			imshow("Contours and convex hull", drawing);
		}    
    }
public:    
    void setLensParameters (LensParameters lensParameters)
    {
        // Construct the camera matrix
        // (fx   0    cx)
        // (0    fy   cy)
        // (0    0    1 )
        cameraMatrix = (Mat1d (3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
                        0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
                        0, 0, 1);

        // Construct the distortion coefficients
        // k1 k2 p1 p2 k3
        distortionCoefficients = (Mat1d (1, 5) << lensParameters.distortionRadial[0],
                                  lensParameters.distortionRadial[1],
                                  lensParameters.distortionTangential.first,
                                  lensParameters.distortionTangential.second,
                                  lensParameters.distortionRadial[2]);
    }
    void toggleUndistort()
    {
        std::lock_guard<std::mutex> lock (flagMutex);
        undistortImage = !undistortImage;
    }

private:
    // define images for depth and gray
    // and for their 8Bit and scaled versions   
    Mat zImagef, grayImagef;
    Mat depthRGB, depthGray, grayImage, binaryImage; 
    
    Mat maskDepthRGB, maskDepthGray;
    
    Point center;	//palm
    uchar centerDistance;	//distance from camera to the center of palm
    int temp_move_bit;
    double radius;	//palm
    double indexFinger_deg = 0.0;
   
    // lens matrices used for the undistortion of
    // the image
    Mat cameraMatrix;
    Mat distortionCoefficients;

    std::mutex flagMutex;
    bool undistortImage;
    
	float radianToDegree(float angleValue)
	{	
		float rad = (angleValue*180.0f) / (float)M_PI;
		return rad;
	}
	void myMorphology(InputArray _src, OutputArray _dst ) 
	{
		Mat src = _src.getMat();
		_dst.create(src.size(), src.type());
		Mat dst = _dst.getMat();
		
		src.copyTo(dst);
		
		Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(dst, dst, MORPH_OPEN, element, Point(-1,-1), 1);	
	}
	void myGetHandROI(InputArray _src, OutputArray _dst)
	{
		Mat src = _src.getMat();
		_dst.create(src.size(), src.type());
		Mat dst = _dst.getMat();
		
		//set ROI gray Image from maskDepthGray  
		Mat maskDepthGrayHist;
		int roi_x1 = myGetHistogram(src, maskDepthGrayHist);
	//imshow("MaskDepthGrayHist", maskDepthGrayHist);
	
        myDepthROI(src, dst, roi_x1-15, roi_x1+8);	//trim the image if it's closer than user's body 
	 	
		//to get rid of noise
		threshold(dst, dst, 127, 255, THRESH_BINARY_INV);
	//	myMorphology(dst, dst);
	}
	void myGetContour(InputArray _src, OutputArray _dst, OutputArray _fill = noArray())
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
	void mySplitRGBImage(InputArray _src, Mat* _dst)
	{
		Mat src = _src.getMat();
		
		vector<Mat> planesRGB;
		split(src, planesRGB);

		planesRGB[0].copyTo(_dst[0]);
		planesRGB[1].copyTo(_dst[1]);
		planesRGB[2].copyTo(_dst[2]);
	}
	
	float myGetAvgImage1f(InputArray _src)
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
	float myGetAvgImage1u(InputArray _src)
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
	void myGrayNormalize(InputArray _src, OutputArray _dst)
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
	void myDepthNormalize(InputArray _src, OutputArray _dst)
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
	void myDepthNormalizeRGB(InputArray _src, OutputArray _dst)
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
	void myMergeFrames(InputArray _src1, InputArray _src2, OutputArray _mask)
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
    int myGetHistogram(InputArray _src, OutputArray _hist = noArray())
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
        for(int i=0; i<histSize/4; i++){
			float area = hist.at<float>(i);
			
			float formular;
			if(i<50) formular = -425*((float)i-59.41f);
			else if(i<histSize/4) formular = -100*((float)i-80);
			float calib1 = area/formular;
			float coeff = 100.0;

			float calib = calib1*coeff;
			int y2 = histImage.rows-cvRound(calib);
			
			Point pt1(i*binW, histImage.rows);
			Point pt2((i+1)*binW, y2);		
			rectangle(histImage, pt1, pt2, Scalar(127), -1);
			
			//handsize + maximum value of hist + hand region range
			if((pt1.y-pt2.y)<200/2 && (pt1.y-pt2.y)>peak_y && (i>25 && i<60) ){
				if(i<30) temp_move_bit = CLOSE;
				else if(i>55) temp_move_bit = FAR;
				//peak_x = i;
				peak_y = pt1.y-pt2.y;
				max_pt1 = Point(pt1);
				max_pt2 = Point(pt2);
			}	
		}
		rectangle(histImage, max_pt1, max_pt2, Scalar(0), -1);
		return max_pt1.x/(histImage.cols/256);
	}
	void myDepthROI(InputArray _src, OutputArray _dst, int min=0, int max=255)
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
	
private:
	//mode control (pre_ready -> ready -> run)
	void gaugeControlMode(int fingerCount)
	{
		if(pre_readyMode_flag){
			if(readyMode_flag){
				runMode(fingerCount);
			}else{
				readyMode(fingerCount);
			}
		}else{
			pre_readyMode(fingerCount);
		}
	}
	bool pre_readyMode_flag = false;
	void pre_readyMode(int fingerCount)
	{	//before getting the message or sending the message
		//if the finger number is all the same during 5 times(5 frames)
		//it is a message from User
		static int ct=0;
		static int pre_fingerCount;
		
		if(fingerCount>=0 && fingerCount<=5){	//detect
			if(pre_fingerCount==fingerCount){
				if(ct==5){
					ct=0;
					pre_readyMode_flag = true;
				}else{
					ct++;
					pre_readyMode_flag = false;
				}
			}else{
				ct=0;
				pre_readyMode_flag = false;
			}
		}else{
			ct=0;
			pre_readyMode_flag = false;
		}
		pre_fingerCount = fingerCount;
	}
	bool readyMode_flag = false;
	void readyMode(int fingerCount)
	{	//before getting the exact message from user
		//delay and clear the message from the user 
		//while keeping the same pose or gesture (message robustness)
		outOfRangeMessage(temp_move_bit);
		
		static int pre_fingerCount;
		if(fingerCount>0 && fingerCount<5){		//detect
			numLED = fingerCount-1;
			if(pre_fingerCount==fingerCount){
				if(readyGauge==READY){
				//	readyGauge=0;
					readyMode_flag = true;
				}else{
					readyGauge++;
					readyMode_flag = false;
				}
			}else{
				readyGauge=0;
				readyMode_flag = false;
			}
		}else{
			readyGauge=0;
			pre_readyMode_flag = false;
			readyMode_flag = false;
		}
		pre_fingerCount = fingerCount;
	}
	void runMode(int fingerCount)
	{	//after the two steps before, finally sending the message to device
		outOfRangeMessage(temp_move_bit);
		if(fingerCount>0 && fingerCount<5){		//detect
			readyGauge = READY;
			ctrIlluminance(numLED, indexFinger_deg);
		//	cout<<numLED<<" "<<led[numLED]<<endl;
		}else{
			if(readyGauge==0){
				pre_readyMode_flag = false;
				readyMode_flag = false;
			}else{
				readyGauge--;
			}
		}
	}
	
	bool outOfRangeBit(Point range, double scale)
	{	
		if(centerDistance<35){
			temp_move_bit = CLOSE;
			return false;
		}else if(centerDistance>55 && centerDistance<255){
			temp_move_bit = FAR;
			return false;
		}
		if(center.x-radius*scale*2<0){
			temp_move_bit = RIGHT;
			return false;
		}else if(center.x+radius*scale>range.x){
			temp_move_bit = LEFT;
			return false;
		}else if(center.y-radius*scale<0){
			temp_move_bit = DOWN;
			return false;
		}else if(center.y+radius*scale>range.y){
			temp_move_bit = UP;
			return false;
		}else{
			temp_move_bit = DETECTION;
			return true;
		}
	}
	//outofRangeMessage
	void outOfRangeMessage(int message)
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
	
	void ctrIlluminance(int led_num, double illum)
	{
		if(illum<0) illum+=360;
		
		//percentage of led illminance (index finger angle => 160 ~ 195)	
		int illum_percent = cvRound(-100.0/35.0*(illum-195.0));
		if(illum_percent<0) illum_percent=0;
		if(illum_percent>100) illum_percent=100;
		
		if(abs(led[led_num]-illum_percent)<20){
			led[led_num] = illum_percent;
		}
	}
	
private:
	void initializeData()
	{
		unsigned char data[4];
		for(int i=0; i<4; i++){
			data[0] = (unsigned char)led[i];
			data[1] = (unsigned char)i;
			data[2] = 0;
			data[3] = NO_DETECTION;
			Bluetooth::transmitting(data, 4);
		}
		start_flag = true;
	}	
	void sendingData()	//sending LED illuminance
	{	//brightness of led + number of led + readyGauge + move_bit 
		if(!start_flag) initializeData();	
		
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
			Bluetooth::transmitting(data, 4);
			for(int i=0; i<4; i++) cout<<+data[i]<<" ";
		}
		cout<<endl;
	}
};


int main (int argc, char *argv[])
{
    // Windows requires that the application allocate these, not the DLL.
    PlatformResources resources;

    // This is the data listener which will receive callbacks.  It's declared
    // before the cameraDevice so that, if this function exits with a 'return'
    // statement while the camera is still capturing, it will still be in scope
    // until the cameraDevice's destructor implicitly de-registers the listener.
    MyListener listener;

	// this represents the main camera device object
    std::unique_ptr<ICameraDevice> cameraDevice;
    
    // the camera manager will query for a connected camera
    {
        CameraManager manager;

        // check the number of arguments
        if (argc > 1)
        {
            // if the program was called with an argument try to open this as a file
            cout << "Trying to open : " << argv[1] << endl;
            cameraDevice = manager.createCamera (argv[1]);
        }
        else
        {
            // if no argument was given try to open the first connected camera
            royale::Vector<royale::String> camlist (manager.getConnectedCameraList());
            cout << "Detected " << camlist.size() << " camera(s)." << endl;

            if (!camlist.empty())
            {
                cameraDevice = manager.createCamera (camlist[0]);
            }
            else
            {
                cerr << "No suitable camera device detected." << endl
                     << "Please make sure that a supported camera is plugged in, all drivers are "
                     << "installed, and you have proper USB permission" << endl;
                return 1;
            }

            camlist.clear();
        }
    }
    // the camera device is now available and CameraManager can be deallocated here

    if (cameraDevice == nullptr)
    {
        // no cameraDevice available
        if (argc > 1)
        {
            cerr << "Could not open " << argv[1] << endl;
            return 1;
        }
        else
        {
            cerr << "Cannot create the camera device" << endl;
            return 1;
        }
	}

    // IMPORTANT: call the initialize method before working with the camera device
    auto status = cameraDevice->initialize();
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device, error string : " << getErrorString (status) << endl;
        return 1;
    }
    
    // retrieve the lens parameters from Royale
    LensParameters lensParameters;
    status = cameraDevice->getLensParameters (lensParameters);
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Can't read out the lens parameters" << endl;
        return 1;
    }

    listener.setLensParameters (lensParameters);

    // register a data listener
    if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener" << endl;
        return 1;
    }
	
	// set fps "MODE_9_15FPS_700" or MODE_9_25FPS_450
    if (cameraDevice->setUseCase ("MODE_9_15FPS_700") != CameraStatus::SUCCESS)
    {
        cerr << "Error setting use case" << endl;
        return 1;
    }

    // create two windows
    namedWindow ("onlyHandROI", WINDOW_AUTOSIZE);

    // start capture mode
    if (cameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing" << endl;
        return 1;
    }
     
	Bluetooth::setBluetooth();	
	wiringPiSetup();	
	softToneCreate(BEEP);
	
	// wait until a key is pressed
	waitKey (0);
	digitalWrite(BEEP,0);
	Bluetooth::closeBluetooth();

    // stop capture mode
    if (cameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error stopping the capturing" << endl;
        return 1;
    }
  
    return 0;
}


