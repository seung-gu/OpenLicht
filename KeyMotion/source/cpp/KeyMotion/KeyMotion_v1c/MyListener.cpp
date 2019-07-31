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
 * 				in KeyMotion version 1c ( July, 2019 )
 \****************************************************************************/

#include "MyListener.h"

enum HANDMODE{ RIGHT_HANDED, LEFT_HANDED, BOTH_HANDED }MODE;


int move_bit;
int readyGauge;

bool displayOn = false;

bool start_flag = false;
bool run_flag = false;



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
	
	imshow("handROI", handROI);
	
	int fingers = countingNumbers();	
				
	detectionControlMode(fingers);
	keyMessageTX();
		
	infoOnDisplay(fingers);
	
//cout<<1<<endl;	//success
clock_t endTime = clock();
double codeExcuteTime = ((double)(endTime-startTime))/CLOCKS_PER_SEC;	
//cout<<codeExcuteTime*1000.0<<endl;
}
	
	
void MyListener::preProcessing()	// grayNormalize+depthNormalize+mergedFrames
{
	maskDepthGray.create(grayImagef.size(), CV_8UC1);
	maskDepthGray = Scalar::all (255);
	
	float pow_coef = 1.9f;
	float average_gray = 26.0f;
	float margin_plus = 35.0f;
	float margin_minus = 15.0f;
	
	for (int y = 0; y < grayImagef.rows; y++)
	{
		float *fGrayRowPtr = grayImagef.ptr<float> (y);
		float *fDepthRowPtr = zImagef.ptr<float> (y);
		uchar *maskPtr = maskDepthGray.ptr<uchar> (y);
		
		for (int x = 0; x < grayImagef.cols; x++)
		{
			if(fGrayRowPtr[x]>0.0f){
				float threshold = pow(fDepthRowPtr[x], pow_coef)*fGrayRowPtr[x];
				if(threshold > average_gray-margin_minus && threshold < average_gray+margin_plus)
					maskPtr[x] = (uchar)(255.0f/float(MAX_DEPTH)*(fDepthRowPtr[x]));
				else
					fGrayRowPtr[x] = 0.0f;
			}
		}
	}
	//imshow("grayImagef", grayImagef);
	//imshow("maskDepthGray", maskDepthGray);
}


void MyListener::myGetHandROI(InputArray _src, OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), src.type());
	Mat dst = _dst.getMat();
	
	//set ROI gray Image from maskDepthGray  
	Mat maskDepthGrayHist;
	int zROI = myGetHistogram(src, maskDepthGrayHist);
	int zROI_tol = 15;		//tolerance of z ROI
	
	//sort only near hand position by depth data
	myDepthROI(src, dst, zROI - zROI_tol*2, zROI + zROI_tol);	
	
	//to make binary
	threshold(dst, dst, 254, 255, THRESH_BINARY_INV);
}
	
int MyListener::myGetHistogram(InputArray _src, OutputArray _hist)
{
	Mat src = _src.getMat();
	
	Mat histImage;
	if(_hist.needed()){
		_hist.create(256,256,CV_8U);
		histImage = _hist.getMat();
	}else{
		histImage = Mat(256,256,CV_8U);
	}
	histImage = Scalar::all(255);
		
	Mat hist;
	int channels = 0;
	int histSize = 64;
	float valueRange[] = {0.0,254.9};	//255 is background, so sorted out
	const float* ranges[] = {valueRange};
	calcHist(&src, 1, &channels, Mat(), hist, 1, &histSize, ranges);   
	
	int binW = histImage.cols/histSize;
	
	// these things are customized values
	// it fits when MAX_DEPTH is 1.0, binW = 256/64
	// but these ranges could be modified depending on the size of hand 
	int max_x = 255, max_y = -1;	
	int num_sample = 5;
	int hand_size_range[] = {160, 360};	
	int std_dev_range[] = {15, 60};

	for(int x = histSize*MAX_DISTANCE-num_sample/2 - 1 ; x >= num_sample/2; x--){
		// hand size is in inverse proportion to depth value 
		// y is the number of pixels of hand area, so it is the size of hand
		int y = cvRound(pow(hist.at<float>(x), 0.5)*float(x)/10.0f);
			
		//drawing rectangles
		Point pt1(x*binW, histImage.rows);
		Point pt2((x+1)*binW, histImage.rows - y);		
		rectangle(histImage, pt1, pt2, Scalar(127), -1);
		
		//hand shape + hand size
		if( y > max_y ){
			
			// if current y value is bigger than max_y, hand area and normalized y arrays are calculated
			int y_norm[num_sample] = {0};	// normalized y
			int hand_area = 0;				// sum of normalized y
			
			// range of sample : ex) sample : 5 => x-2, x-1, x, x+1, x+2
			for(int i = -num_sample/2; i<=num_sample/2; i++){
				//normalized
				y_norm[i+num_sample/2] = cvRound(pow(hist.at<float>(x+i), 0.5)*float(x+i)/10.0f);	
				hand_area += y_norm[i+num_sample/2];
			}
			float avg_y_norm = (float)hand_area/(float)num_sample;
				
			if( hand_area > hand_size_range[0] && hand_area < hand_size_range[1]){	// size range
				
				// if hand area is within the cusomized range, standard dev is calculated
				float sum_dev = 0.0f;
				for(int i=0; i<num_sample; i++)
					sum_dev += pow(y_norm[i]-avg_y_norm, 2);
				float std_dev = pow(sum_dev/(float)num_sample, 0.5);	//standard deviation
		
				if( std_dev > std_dev_range[0] && std_dev < std_dev_range[1]){		// shape -> standard deviation
					
					max_x = x;
					max_y = y_norm[num_sample/2];
					
				//	cout<<"sum : "<<sum_y_norm<<endl;
				//	cout<<"avg : "<<avg_y_norm<<endl;
				//	cout<<"std dev : "<<pow(sum_dev/(float)num_sample, 0.5)<<endl;
				}
			}
		}
	}
	rectangle(histImage, Point(max_x*binW, histImage.rows), 
		Point((max_x+1)*binW, histImage.rows - max_y), Scalar(0), -1);
	imshow("histImage", histImage);
	
	return (max_x*255)/histSize;
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


void MyListener::getHandCenter(InputArray _mask)
{
	Mat mask = _mask.getMat();
	
	Mat dst;
	distanceTransform(mask, dst, CV_DIST_L2, 3); 
		
	int numMarking = 7;
	int minIdx[2];    //idx[0] : y, idx[1] : x
	int palmSizeRange[] = {1500, 2600};
	
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
		
		if(palmSizeRange[0]<palmSize && palmSize<palmSizeRange[1]){	
			erase = radius*2.0;			
			vHand.push_back(Hand(handPos, radius, fdistance));	
		}else{
			erase = radius/10.0;
		}
		if(minIdx[1]!=-1){
			circle(dst, Point(minIdx[1], minIdx[0]), cvRound(erase*1.1), Scalar(0), -1);
		}	
	}
	
	//imshow("dst",dst);
	
	sort(vHand.begin(), vHand.end());
	
	//if(MODE == BOTH_HANDED)
	//	handSegment();
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
						
		//detect points where the value changes 0 -> 255
		if(src.at<uchar>(pre_pt.y, pre_pt.x)==0 && src.at<uchar>(pt.y, pt.x)==255){		
			start_pt = pt;	
		}
		else if(src.at<uchar>(pre_pt.y, pre_pt.x)==255 && src.at<uchar>(pt.y, pt.x)==255){
			fingerWidth++;
		}		
		else if(src.at<uchar>(pre_pt.y, pre_pt.x)==255 && src.at<uchar>(pt.y, pt.x)==0){
		
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
	

	
	




double MyListener::radianToDegree(double angleValue)
{	
	return ( (angleValue*180.0f) / (double)M_PI );	
}












void MyListener::detectionControlMode(int fingerCount)
{

	if(fingerCount >= 0 && fingerCount <= 5 && !outOfRange())	//detected
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
		// when the hand is moving away from camera,
		// it starts counting and being ready to run
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
			if(readyGauge == READY && vHandPos.size() > 5)
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

bool MyListener::outOfRange()
{
	if(vHand.empty()) return true;
	int dist = vHand.at(0).distance;
	int x = vHand.at(0).pos.x;
	int y = vHand.at(0).pos.y;
	int x_tol = (int)((float)zImagef.cols * (float)dist/255.0f );
	int y_tol = (int)((float)zImagef.rows * (float)dist/255.0f / 2.0);
	
	if( (x>x_tol && x<zImagef.cols-x_tol) && (y>y_tol && y<(zImagef.rows-y_tol)) )
		return false;
	else
		return true;
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
		vHandPos_z.clear();
		vRadius.clear();
	}
	
	//vector push_back
	if(vHand.empty()) return;
	int dist = vHand.at(0).distance;
	float radius = (float)(255-dist)/10.0f;	//distance
	
	int x_tol = (int)((float)zImagef.cols * (float)dist/255.0f );
	int y_tol = (int)((float)zImagef.rows * (float)dist/255.0f / 2.0);
	
	
	//avoid closed at boundaries
	
	if( !outOfRange() ){
		if( vHandPos.size() < 10 ){	// when HandPos vector has less than 10 arrays 
			vHandPos.push_back(vHand.at(0).pos);
			vRadius.push_back(radius);
			vHandPos_z.push_back(dist);
		}else{						// when HandPos vector has enough 10 arrays 
			vHandPos.erase(vHandPos.begin());	//update while deleting an oldest value 
			vHandPos.push_back(vHand.at(0).pos);// ,adding a newest value 
			
			vHandPos_z.erase(vHandPos_z.begin());
			vHandPos_z.push_back(dist);
			
			vRadius.erase(vRadius.begin());
			vRadius.push_back(radius);
			
		}
	}

	//drawing circle tracks
	for(int i=0; i<vHandPos.size(); i++)
	{	
		circle(trackMat, vHandPos[i], vRadius[i], Scalar(64+i*19), -1);
		if(i!=0)
			arrowedLine(trackMat, vHandPos[i-1], vHandPos[i], Scalar(255), 1);
	}
	//rectangle
	rectangle(trackMat, Point(x_tol, y_tol), Point(zImagef.cols-x_tol, zImagef.rows-y_tol), Scalar(255), 2);
	
	//imshow("trackMat", trackMat);
}

void MyListener::direction()
{	
	//movement direction
	int xCt = 0, yCt = 0;
	int start_pt = 1, last_pt = 4;
	Point difPos = vHandPos[vHandPos.size()-start_pt] - vHandPos[vHandPos.size()-last_pt];
	for(int i=vHandPos.size()-start_pt; i>vHandPos.size()-last_pt; i--)
	{
		if(vHandPos[i].x - vHandPos[i-1].x > 0) xCt++;	//moving leftward
		else if(vHandPos[i].x - vHandPos[i-1].x < 0) xCt--;	  //rightward
		
		if(vHandPos[i].y - vHandPos[i-1].y > 0) yCt++;	//moving upward
		else if(vHandPos[i].x - vHandPos[i-1].x < 0) yCt--;	  // downward
	}
	
	//movement condition
	int sum_dist = 0;
	for(int i=0; i<vHandPos_z.size(); i++)
		sum_dist += vHandPos_z[i];
	int dist = sum_dist/vHandPos_z.size();
	
	int x_tol = (int)((float)zImagef.cols * (float)dist/255.0f );
	int y_tol = (int)((float)zImagef.rows * (float)dist/255.0f / 2.0);
	
	int x_dif_range = zImagef.cols/2 - x_tol;
	int y_dif_range = zImagef.rows/2 - y_tol;
	
	if(abs(difPos.x) > x_dif_range/2 && abs(difPos.y) < y_dif_range)
	{
		if(xCt>=3){
			directionMsg = MSG_RIGHT;
			cout<<"Right"<<endl;
		}
		else if(xCt<=-3){
			directionMsg = MSG_LEFT;
			cout<<"Left"<<endl;
		}
	}
	else if(abs(difPos.x) < x_dif_range/2 && abs(difPos.y) > y_dif_range/2)
	{
		if(yCt>=3){
			directionMsg = MSG_DOWN;
			cout<<"Down"<<endl;
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
	
	switch(directionMsg)
	{
	case MSG_LEFT:
		digitalWrite(KEY_LEFT, LOW);
		softToneWrite(BEEP,5120);	////
		break;
	case MSG_RIGHT:
		digitalWrite(KEY_RIGHT, LOW);
		softToneWrite(BEEP,5120);
		break;
	case MSG_DOWN:
		digitalWrite(KEY_DOWN, LOW);
		softToneWrite(BEEP,5120);
		break;
	}
}


void MyListener::infoOnDisplay(int fingers)
{
	display.clearDisplay();
	
	string strDirectionMsg;
	
	static int pre_directionMsg;
	static int msgGauge;
	
	if(directionMsg)
	{
		switch(directionMsg)
		{
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
		pre_directionMsg = directionMsg;
		directionMsg = 0;
		msgGauge = 20;	//turn on message "LEFT", "RIGHT", "DOWN" for 20 frames
	}
	else if(pre_directionMsg)
	{
		if(msgGauge>0)
		{
			switch(pre_directionMsg)
			{
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
			msgGauge--;
		}else{
			pre_directionMsg = 0;
		}
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
