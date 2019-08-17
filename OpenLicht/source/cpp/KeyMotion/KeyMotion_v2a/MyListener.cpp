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

#include "MyListener.h"


int readyGauge;
bool run_flag = false;

bool displayOn = false;		//extern -> main.cpp
		

void MyListener::onNewData (const DepthData *data)
{
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
	//setHandNum();				// set (determine) the number of hands
	
	imshow("handROI", handROI);
	
	Mat onlyHandROI;	//without wrist
	int fingers = countingFingers(handROI, onlyHandROI);
			
	detectionControlMode(fingers);
	keyMessageTX();
		
	infoOnDisplay(fingers);
	
	cout<<1<<endl;	//success
}
	
	
void MyListener::preProcessing()	// grayNormalize+depthNormalize+mergedFrames
{
	maskDepthGray.create(grayImagef.size(), CV_8UC1);
	maskDepthGray = Scalar::all (255);
	
	float pow_coef = 1.9f;
	float average_gray = 26.0f;	//average value of gray
	float margin_plus = 35.0f;	//tolerance
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
	int hand_size_range[] = {160, 360};			// <------ could be customized
	int std_dev_range[] = {15, 70};				// <------ could be customized

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
					
				//	cout<<"hand area : "<<hand_area<<endl;
				//	cout<<"std dev : "<<pow(sum_dev/(float)num_sample, 0.5)<<endl;
				}
			}
		}
	}
//	rectangle(histImage, Point(max_x*binW, histImage.rows), 
//		Point((max_x+1)*binW, histImage.rows - max_y), Scalar(0), -1);
//	imshow("histImage", histImage);
	
	return (max_x*255)/histSize;
}

// set the range by z distance (min ~ max)
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

//get contour of hand and fill in 
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

//get hand center
void MyListener::getHandCenter(InputArray _mask)
{
	Mat mask = _mask.getMat();
	
	Mat dst;
	distanceTransform(mask, dst, CV_DIST_L2, 3); //distance transform
		
	int numMarking = 7;	// 
	int minIdx[2];    	// idx[0] : y, idx[1] : x
	int palmSizeRange[] = {1500, 2600};		// <------ could be customized (hand size)
	
	vHand.clear();
	
	//if arms are too big, then arm could be also detected 
	// => to avoid this, use only the uppermost point
	for(int i=0; i<numMarking; i++){
		double radius;
		//get maximum point and size by 32 float value
		minMaxIdx(dst, NULL, &radius, NULL, minIdx, mask);   
		
		Point handPos = Point(minIdx[1],minIdx[0]);
		int distance = maskDepthGray.at<uchar>(minIdx[0],minIdx[1]);
		float fdistance = zImagef.at<float>(minIdx[0],minIdx[1]);

		double palmSize = distance*radius;	
		double erase;
		
		if(palmSizeRange[0]<palmSize && palmSize<palmSizeRange[1]){	
			// to avoid to pick a second greatest value just near the greatest value
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
} 



int MyListener::countingFingers(InputArray _src, OutputArray _dst)
{
	if(vHand.empty()) return -1;
	
	Mat src = _src.getMat();
	_dst.create(src.size(), src.type());
	Mat dst = _dst.getMat();

	Point center(vHand.at(0).pos);
	
	//Mat cImg(src.size(), CV_8U, Scalar(0));		
	//circle(cImg, center, cvRound(vHand.at(handNum).palmRadius * circleScale), Scalar(255));	//
	
	vector<Point> v_circle;
	midPointCircle(v_circle, Size(src.cols, src.rows));	
	 
	vector<Point> v_point;		//finger position
	vector<double> v_deg;		//finger angle between finger position and palm center

	//cImg = src&cImg;	

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
			
			// finger width (5 ~ 35) <------ could be customized
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

	//counting number of fingers
	int fingerCount = (int)v_deg.size();			
	
	// counting points on arm
	// if the points are below (negative angle => quadrant 3 and 4), it means arm
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
	
void MyListener::midPointCircle(vector<Point>& v_circle, Size size)
{
	if(vHand.empty()) return;
//Midpoint circle algorithm
//0.55ms -> 0.35ms 
//take shorter time than with the algorithm findContours of circle
	vector<Point> v_circle_octa[8];
	
	Point center(vHand.at(0).pos);
	
	double circleScale = 2.0;
	
	int radius = cvRound(vHand.at(0).palmRadius * circleScale);
	
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

	

void MyListener::detectionControlMode(int fingerCount)
{

	if(fingerCount >= 0 && fingerCount <= 5 && !outOfRange())	//detected
		handDetected();
	else 														//non detected
		handNonDetected();
}

bool MyListener::outOfRange()
{
	if(vHand.empty()) return true;
	
	// x_tol, y_tol are the range of boundary
	// this value is changing depending on the distance
	int dist = vHand.at(0).distance;
	int x = vHand.at(0).pos.x;
	int y = vHand.at(0).pos.y;
	int x_tol = (int)((float)zImagef.cols * (float)dist/255.0f );	//tolerance
	int y_tol = (int)((float)zImagef.rows * (float)dist/255.0f / 2.0);
	
	if( (x>x_tol && x<zImagef.cols-x_tol) && (y>y_tol && y<(zImagef.rows-y_tol)) )
		return false;	//not out of range
	else
		return true;	//out of range
}

void MyListener::handDetected()
{
	static int readyCount;
	if(!run_flag)
	{	
		if(readyCount < 5)
		{
			
		// when the hand is getting further from camera,
		// it starts counting and being ready to run 
			if(vHand.empty()) return;
			
			int moving_speed = +2;
			
			static int pre_distance = 255;
			
			if(pre_distance+moving_speed < vHand.at(0).distance)
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

void MyListener::runMode()
{
	trackMat.create(zImagef.size(), CV_8UC1);
	trackMat = Scalar(0);
	
	if(vHand.empty()) return;
	
	if(readyGauge != READY)
	{
		// when the hand comes back into the range during runMode,
		// readyGauge is quickly updated full
		readyGauge = READY;	
		
		//make vector emptied
		vHandPos.clear();
		vHandPos_z.clear();
		vRadius.clear();
	}
	
	// x_tol, y_tol are the range of boundary
	// this value is changing depending on the distance
	int dist = vHand.at(0).distance;
	float radius = (float)(255-dist)/10.0f;	//distance
	
	int x_tol = (int)((float)zImagef.cols * (float)dist/255.0f );
	int y_tol = (int)((float)zImagef.rows * (float)dist/255.0f / 2.0);
	
	
	//vector push_back
	if( !outOfRange() ){
		if( vHandPos.size() < max_vHand_arr ){	// when HandPos vector has less than 10 arrays 
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
	// compare start_pt of vHandPos array with last_pt of it 
	int test_size = 5;	
	// calculate how many pixels the position moved between start_pt and last_pt
	Point difPos = vHandPos[vHandPos.size()-1] - vHandPos[vHandPos.size()-test_size-1];
	for(int i=vHandPos.size()-1; i>vHandPos.size()-test_size-1; i--)
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
	
	float accuracy = 0.6; // 60% accuracy rate 				<------ could be customized
	int success_point = cvRound((float)test_size * accuracy);
	// e.g.) regarded as successful, if xCt or yCt is over than test_size*accuracy
	if(abs(difPos.x) > x_dif_range/2 && abs(difPos.y) < y_dif_range)
	{
		if(xCt >= success_point){
			directionMsg = MSG_RIGHT;
			//cout<<"Right"<<endl;
		}
		else if(xCt <= -success_point){
			directionMsg = MSG_LEFT;
			//cout<<"Left"<<endl;
		}
	}
	else if(abs(difPos.x) < x_dif_range/2 && abs(difPos.y) > y_dif_range/2)
	{
		if(yCt >= success_point-1){
			directionMsg = MSG_DOWN;
			//cout<<"Down"<<endl;
		}
	}	
}

void MyListener::keyMessageTX()
{
	digitalWrite(KEY_LEFT, HIGH);	
	digitalWrite(KEY_RIGHT, HIGH);	
	digitalWrite(KEY_DOWN, HIGH);	
	
	softToneWrite(BEEP,0);			// turn off after one frame
	
	static bool pre_run_flag;
	if(!pre_run_flag && run_flag) softToneWrite(BEEP, 5120);
	pre_run_flag = run_flag;
	
	switch(directionMsg)
	{
	case MSG_LEFT:
		digitalWrite(KEY_LEFT, LOW);
		softToneWrite(BEEP,5120);	
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
		msgGauge = 20;	//to display message "LEFT", "RIGHT", "DOWN" for 20 frames
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
		if(!vHand.empty())
		{
			string strhandNum = to_string(1) + ">";
			string strPosX = " x:" + to_string(vHand.at(0).pos.x);
			string strPosY = " y:" + to_string(vHand.at(0).pos.y);
			string strPosZ = " z:" + to_string(vHand.at(0).distance);
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
