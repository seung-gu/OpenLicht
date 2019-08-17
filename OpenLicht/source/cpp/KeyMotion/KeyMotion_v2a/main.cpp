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
 * Overwritten by Seung-Gu Kang (main class) 
 * 				in KeyMotion version 2a ( July, 2019 )
 \****************************************************************************/


#include "MyListener.h"


void initPINPRESS();
void PINPRESS_handler();
void initPINKEY2();
void PINKEY2_handler();

ArduiPi_OLED display;	

extern bool displayOn;			//myListener.cpp

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
			//cout << "Trying to open : " << argv[1] << endl; 
            
            cameraDevice = manager.createCamera (argv[1]);
        }
        else
        {
            // if no argument was given try to open the first connected camera
            royale::Vector<royale::String> camlist (manager.getConnectedCameraList());
			//cout << "Detected " << camlist.size() << " camera(s)." << endl;
            
            if (!camlist.empty())
            {
                cameraDevice = manager.createCamera (camlist[0]);
            }
            else
            {
               // cerr << "No suitable camera device detected." << endl
               //      << "Please make sure that a supported camera is plugged in, all drivers are "
               //      << "installed, and you have proper USB permission" << endl;
                cout<<-1<<endl;	//error
                return -1;
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
          //cerr << "Could not open " << argv[1] << endl;
            cout<<-1<<endl;	//error
            return -1;
        }
        else
        {
          //cerr << "Cannot create the camera device" << endl;
            cout<<-1<<endl;	//error
            return -1;
        }
	}

    // IMPORTANT: call the initialize method before working with the camera device
    auto status = cameraDevice->initialize();
    if (status != CameraStatus::SUCCESS)
    {
      //cerr << "Cannot initialize the camera device, error string : " << getErrorString (status) << endl;
        cout<<-1<<endl;	//error
        return -1;
    }
    
    // retrieve the lens parameters from Royale
    LensParameters lensParameters;
    status = cameraDevice->getLensParameters (lensParameters);
    if (status != CameraStatus::SUCCESS)
    {
      //cerr << "Can't read out the lens parameters" << endl;
        cout<<-1<<endl;	//error
        return -1;
    }

    //listener.setLensParameters (lensParameters);

    // register a data listener
    if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    {
      //cerr << "Error registering data listener" << endl;
        cout<<-1<<endl;	//error
        return -1;
    }
	
	// set fps "MODE_9_15FPS_700" or MODE_9_25FPS_450
    if (cameraDevice->setUseCase ("MODE_9_25FPS_450") != CameraStatus::SUCCESS)
    {
      //cerr << "Error setting use case" << endl;
        cout<<-1<<endl;	//error
        return -1;
    }

    // create two windows
    namedWindow ("handROI", WINDOW_AUTOSIZE);

    // start capture mode
    if (cameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
      //cerr << "Error starting the capturing" << endl;
        cout<<-1<<endl;	//error
        return -1;
    }

	// Oled supported display in ArduiPi_SSD1306.h , SPI
	if ( !display.init(OLED_SPI_DC, OLED_SPI_RESET, OLED_SPI_CS, OLED_ADAFRUIT_SPI_128x64) )
		exit(EXIT_FAILURE);		
	display.begin();			// init done
	
		
	wiringPiSetup();
	softToneCreate(BEEP);
	pinMode(KEY2, INPUT);
	
	initPINKEY2();
	initPINPRESS();
	
	// raspberry TX pins setting
	pinMode(KEY_LEFT, OUTPUT);
	pullUpDnControl(KEY_LEFT, PUD_UP);
	pinMode(KEY_RIGHT, OUTPUT);
	pullUpDnControl(KEY_RIGHT, PUD_UP);
	pinMode(KEY_DOWN, OUTPUT);
	pullUpDnControl(KEY_DOWN, PUD_UP);

	// wait until a key is pressed
	waitKey (0);

    // stop capture mode
	destroyAllWindows();
	//Bluetooth::closeBluetooth();
	softToneStop(BEEP);	
    
    if (cameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
      //cerr << "Error stopping the capturing" << endl;
        cout<<-1<<endl;	//error
        return -1;
    }
    
    display.clearDisplay();
    display.display();
    
    exit(EXIT_SUCCESS);
}




void initPINPRESS(void)
{
	pinMode(PINPRESS, INPUT);
	pullUpDnControl(PINPRESS, PUD_UP);
	wiringPiISR(PINPRESS, INT_EDGE_RISING, PINPRESS_handler);
}
void PINPRESS_handler(void)
{
	cout<<0<<endl;
			
//end of process pressing a fake key 
// it's the only way to end the program so far
// with waitKey(0) and callback function
	Display *disp = XOpenDisplay(NULL);
	if(!disp) return;
	XTestFakeKeyEvent(disp, 9, True, 0);
	XTestFakeKeyEvent(disp, 9, False, 0);
	XCloseDisplay(disp);	
}

void initPINKEY2(void)
{
	pinMode(KEY2, INPUT);
	pullUpDnControl(KEY2, PUD_UP);
	wiringPiISR(KEY2, INT_EDGE_RISING, PINKEY2_handler);
}
void PINKEY2_handler(void)
{
	displayOn = !displayOn; 	// toggle
}
