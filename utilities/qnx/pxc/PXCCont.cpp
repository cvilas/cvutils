//======================================================================== 
// Project: OCTOR   
// ---------------------------------------------------------------------- 
// Package: PXC Series Continuous Display
// Authors: Vilas Kumar Chitrakaran
// Start Date: Tue Sep 30 09:30:10 EDT 2003
// Compiler: GCC 2.95.3
// Operating System: QNX 6.2.1 
// ----------------------------------------------------------------------  
// File: PXCCont.cpp
// Implementation of the class PXCCont.
// See PXCCont.hpp for more details.
//========================================================================  

// ----- Standard Libaries -----

// ----- QRTS Libraries -----

// ----- Project Includes -----
#include "PXCCont.hpp"

unsigned long PXCCont::s_framegrabberHandle;
int PXCCont::s_videoType;
int PXCCont::s_pixelType;
int PXCCont::s_modelNumber;
int PXCCont::s_inputChannel = 0;
int PXCCont::s_inputMode = e_composite;
bool PXCCont::s_doGrab = false;

static COnOffButton *s_selectCameraModeSVideo;
static COnOffButton *s_selectCameraModeComposite;
static COnOffButton *s_selectCamera01;
static COnOffButton *s_selectCamera02;	
static COnOffButton *s_selectCamera03;
static COnOffButton *s_selectCamera04;
static CText *d_fileBaseText;
	
static PXC200 *s_pxc200Lib;
		// function pointer to pxc200 library
		
static FRAMELIB *s_frameLib;
		// function pointer to frame library


//======================================================================== 
// min: A simple function to limit pixel value range
//========================================================================  
int min(int a, int b)
{
	if (b < 0)
		b = 0;
	if(a <= b)
		return a;
	return b;
}

//======================================================================== 
// PXCCont::PXCCont: Constructor of the PXCCont class
//========================================================================  
 
PXCCont::PXCCont()
	: CMainWindow("PXC Series Continuous Display"),
	d_liveImage(0),
	d_height(480),
	d_width(640)
{
	s_doGrab = false;
	
	// Initialize the framegrabber
	if(initPXC() == -1)
		return;
	
	// start the GUI
	initMainWindow();
	PtFlush();

	// setup a photon cahnnel for communication
	// between GUI and framegrabber
	d_channelId = ChannelCreate(0);
	if (d_channelId == -1)
	{
		d_status.setStatusError()
			<< "Error creating channel for pulses" << endl;
		return;
	}
	
	d_connectionId = ConnectAttach(0, 0, d_channelId, _NTO_SIDE_CHANNEL, 0 );
	if (d_connectionId == -1)
	{
		d_status.setStatusError()
			<< "Error creating connection to the channel for pulses" << endl;
		return;
	}

	PhChannelAttach(d_channelId, d_connectionId, NULL);
	PtAppAddInput(NULL, 0, s_eventCallback, this);

	// start grabbing image frames
	s_doGrab = true;
	grab();
}


//======================================================================== 
// PXCCont::~PXCCont: Destructor of the PXCCont class
//========================================================================  
 
PXCCont::~PXCCont ()
{
	ConnectDetach(d_connectionId);
	
	if(d_liveImage)
	{
		if(d_liveImage->image) delete d_liveImage->image;
		delete d_liveImage;
	}
	
	if(d_frameBuffer)
		s_frameLib->FreeFrame(d_frameBuffer);

	if(s_framegrabberHandle)
		s_pxc200Lib->FreeFG(s_framegrabberHandle);

	FRAMELIB_CloseLibrary(s_frameLib);
	PXC200_CloseLibrary(s_pxc200Lib);
}


//======================================================================== 
// PXCCont::initPXC
//========================================================================  
int PXCCont::initPXC()
{
	if(geteuid() != 0){
 		d_status.setStatusError() 
			<< "[PXCCont::initPXC] ERROR. Program must be executed as root." << endl;
 		return(-1);
	}
	d_frameBuffer = NULL;
	d_status.setStatusOk();

	s_frameLib = new FRAMELIB;
	s_pxc200Lib = new PXC200;
	
	// Open the frame library first
	if(!FRAMELIB_OpenLibrary(s_frameLib,sizeof(FRAMELIB)))
	{
		d_status.setStatusError()
			<< "[PXCCont::PXCCont] Couldn't allocate FRAMELIB." << endl; 
		return -1;
	}
	
	// Open the PXC200 library next
	if(!PXC200_OpenLibrary(s_pxc200Lib,sizeof(PXC200)))
	{ 
		d_status.setStatusError() 
			<< "[PXCCont::PXCCont] Couldn't open PXC200 library."
			<< endl;
		return -1;
	}
	
	// allocate any PXC200
	s_framegrabberHandle = s_pxc200Lib->AllocateFG(-1);

	// Determine the video type - this is important for 
	// determining the frame buffer size
	s_videoType = s_pxc200Lib->VideoType(s_framegrabberHandle);
	s_modelNumber = s_pxc200Lib->GetModelNumber(s_framegrabberHandle);
	switch(s_videoType)
	{
		case 0:     // no video
		case 1:     // NTSC/RS-170
			d_width = 640;
			d_height = 486;
			break;
		case 2:     // CCIR/PAL
			d_width = 768;
			d_height = 576;
			break;
		default:
			cout << "[PXCCont::PXCCont] Video: None, or unknown  " << endl;
	}

	//The following values control scaling and decimation. For normal frames
	//they can be set to the height and width of the frame buffer.
	s_pxc200Lib->SetWidth(s_framegrabberHandle, d_width);
	s_pxc200Lib->SetHeight(s_framegrabberHandle, d_height);
	s_pxc200Lib->SetXResolution(s_framegrabberHandle, d_width);
	s_pxc200Lib->SetYResolution(s_framegrabberHandle, d_height);

	// Allocate a frame buffer
	// Use PBITS_RGB24 for color and PBITS_Y8 for monochrome
	s_pixelType = PBITS_RGB24;
	d_frameBuffer = s_pxc200Lib->AllocateBuffer(d_width, d_height, s_pixelType);
	if(d_frameBuffer == NULL)
	{
		d_status.setStatusError()
			<< "[PXCCont::PXCCont] Framebuffer allocation failed" << endl;
		return -1;
	}
	d_bytesPerPixel = 3;
	
	// Get a pointer to the frame buffer
	d_imageData = (unsigned char *)s_frameLib->FrameBuffer(d_frameBuffer);
	
	return 0;
}


//======================================================================== 
// PXCCont::initMainWindow
//========================================================================  
void PXCCont::initMainWindow()
{
	CLabel *label;
	setResizeXYAlways();
	
	d_group = new CGroup(this);
	d_group->setVerticalOrientation();

	//-------------------------------------------
	// MENU
	//-------------------------------------------
	d_menuBar = new CMenuBar(d_group);
		
	// ----- File menu
	CMenuButton *fileMenuButton = new CMenuButton(d_menuBar, "File");
	CMenu *fileMenu = new CMenu(fileMenuButton);
	d_fileExit = new CMenuButton(fileMenu, "Exit", s_menuCallback, this);

	// ----- Help menu
	CMenuButton *helpMenuButton = new CMenuButton(d_menuBar, "Help");
	CMenu *helpMenu = new CMenu(helpMenuButton);
	d_about = new CMenuButton(helpMenu, "About", s_menuCallback, this);

	//----------------------------------------------
	// Live image window
	//----------------------------------------------
	d_imagesGroup = new CGroup(d_group);
	d_imagesGroup->setVerticalAlignCenter();
	createLiveImage(d_imagesGroup);

	//----------------------------------------------
	// Input mode, camera input channel and image control
	//----------------------------------------------
	CGroup *controlsGroup = new CGroup(d_group);
	controlsGroup->setHorizontalOrientation();
	controlsGroup->setMargin(8, 8);
	controlsGroup->setResizeXAsRequired();

	//-------- input channel
	CGroup *cameraInputGroup = new CGroup(controlsGroup);
	cameraInputGroup->setVerticalOrientation();
	cameraInputGroup->setMargin(8, 8);

	label = new CLabel(cameraInputGroup, "Input Channel");
	label->setFontBoldOn();
	
	cameraInputGroup->setExclusiveSelectOn();
	s_selectCamera01 = new COnOffButton(cameraInputGroup, 
						"Camera 1", s_inputSelectCallback);
	s_selectCamera02 = new COnOffButton(cameraInputGroup, 
						"Camera 2", s_inputSelectCallback);
	s_selectCamera03 = new COnOffButton(cameraInputGroup, 
						"Camera 3", s_inputSelectCallback);
	s_selectCamera04 = new COnOffButton(cameraInputGroup, 
						"Camera 4", s_inputSelectCallback);
	s_selectCamera01->setState(1);

	//-------- input mode
	CGroup *cameraModeGroup = new CGroup(controlsGroup);
	cameraModeGroup->setVerticalOrientation();
	cameraModeGroup->setMargin(8, 8);

	label = new CLabel(cameraModeGroup, "Input Mode");
	label->setFontBoldOn();

	cameraModeGroup->setExclusiveSelectOn();
	s_selectCameraModeComposite = new COnOffButton(cameraModeGroup, 
						"Composite", s_inputSelectCallback);
	s_selectCameraModeSVideo = new COnOffButton(cameraModeGroup, 
						"S-Video", s_inputSelectCallback);
	s_selectCameraModeComposite->setState(1);
	s_selectCameraModeSVideo->setWidth(90);
	s_selectCameraModeComposite->setWidth(90);
	
	//--------- image control
	CGroup *sliderGroup = new CGroup(controlsGroup);
	sliderGroup->setVerticalOrientation();
	sliderGroup->setMargin(8, 8);
	
	label = new CLabel(sliderGroup, "Image Controls");
	label->setFontBoldOn();
	CGroup *brightnessGroup = new CGroup(sliderGroup);
	brightnessGroup->setHorizontalOrientation();
	
	new CLabel(brightnessGroup, "Brightness:");
	d_imageBrightness = new CSlider(brightnessGroup);
	d_imageBrightness->setRange(-100, 100);
	d_imageBrightness->setHandleWidth(10);
	d_imageBrightness->setWidth(150);
	
	CGroup *contrastGroup = new CGroup(sliderGroup);
	contrastGroup->setHorizontalOrientation();
	
	new CLabel(contrastGroup, "Contrast   :");
	d_imageContrast = new CSlider(contrastGroup);
	d_imageContrast->setRange(1, 100);
	d_imageContrast->setHandleWidth(10);
	d_imageContrast->setWidth(150);

	d_timestampButton = new COnOffButton(sliderGroup, "Stamp Time/Framerate");
	d_crosshairButton = new COnOffButton(sliderGroup, "Show Crosshair");
	d_timestampButton->setTypeCheck();
	d_crosshairButton->setTypeCheck();
	d_timestampButton->setWidth(165);
	d_crosshairButton->setWidth(165);
	d_crosshairButton->setState(1);
	d_timestampButton->setState(1);

	//-----------------------------------------------------
	// acquire
	//-----------------------------------------------------
	CGroup *acquireButtonGroup = new CGroup(controlsGroup);
	acquireButtonGroup->setVerticalOrientation();
	acquireButtonGroup->setHorizontalAlignCenter();
	acquireButtonGroup->setMargin(8, 8);
	acquireButtonGroup->highlight();
	
	label = new CLabel(acquireButtonGroup, "Acquire Image");
	label->setFontBoldOn();
	
	d_acquireButton = new CButton(acquireButtonGroup, "Grab", 
											s_imageAcquireCallback, this);
	d_acquireButton->setSize(100, 20);
	d_acquireButton->setArmFill();
	d_acquireButton->setArmColor(0xFF0000);

	new CLabel(acquireButtonGroup, "File base-name: ");
	d_fileBaseText = new CText(acquireButtonGroup, "./images/frame_", 10);

	controlsGroup->setSize(d_imagesGroup->getWidth(), controlsGroup->getHeight());
	d_group->calculateExtent();
}



//======================================================================== 
// PXCCont::createLiveImage
//========================================================================  
void PXCCont::createLiveImage(CWidget *parent)
{
	CGroup *group;
	char headerText[100];
	
	switch(s_modelNumber)
	{
		case PXC200_LC:
			sprintf(headerText, "Live video on: PXC200L");
			break;
		case PXC200_LC_2:
			sprintf(headerText, "Live video on: PXC200AL");
			break;
		case PXC200_F:
			sprintf(headerText, "Live video on: PXC200F");
			break;
		case PXC200_F_2:
			sprintf(headerText, "Live video on: PXC200AF");
			break;
		default:
			sprintf(headerText, "Live video on: unknown device");
			break;
	}
	

	// Allocate and initialize the image object
	d_liveImage = new PhImage_t;
	d_liveImage->type = Pg_IMAGE_DIRECT_8888;
	d_liveImage->size.w = (short)d_width;
	d_liveImage->size.h = (short)d_height;
	d_liveImage->bpl = 4 * d_width;
	
	// Allocate the image buffer
	d_liveImageBufferSize = d_liveImage->bpl * d_height;
	d_liveImage->image = new char[d_liveImageBufferSize];

	// Create the widget
	group = new CGroup(parent);
	group->setVerticalOrientation();
	group->setHorizontalAlignCenter();
	CLabel *label = new CLabel(group, headerText);
	label->setFontBoldOn();
	
	// In QNX6.2.1, there is some flickering for some reason (was not in QNX 4)
	// Hence, we put the image in a double buffer widget to avoid the flickering
	// Need to do some extra work, since the PtOSContainer is not part of QWidgets yet
	PtArg_t  argt[5];
	PtWidget_t *osContainer = PtCreateWidget( PtOSContainer, 
							group->getWidgetPointer(), 0, argt );
	CWidget *qwidgetsOsContainer = new CWidget(osContainer);
	qwidgetsOsContainer->setSize(d_width, d_height);
	d_liveImageWidget = new CLabel(qwidgetsOsContainer, d_liveImage);
}


//======================================================================== 
// PXCCont::grab
//========================================================================  
void PXCCont::grab()
{
	// grab -> updateLiveImage() -> grab

	// Take a snapshot
/*	if(s_doGrab)
		s_pxc200Lib->Grab(s_framegrabberHandle, d_frameBuffer, 0); // no queueing
*/

	// Set it up for continuous grab
	static int startGrab;
	if(startGrab == 0)
	{
		s_pxc200Lib->GrabContinuous(s_framegrabberHandle, d_frameBuffer, -1, 0); // no queueing
		startGrab = 1;
	}

	// send pulse to update image
	MsgSendPulse(d_connectionId, 
		sched_get_priority_min(SCHED_FIFO), 1, 200);
}


//======================================================================== 
// PXCCont::updateLiveImage
//========================================================================  
void PXCCont::updateLiveImage()
{
	unsigned char *source;
	unsigned char *destination;

	char value;
	double brightness;
	double contrast;

	source = d_imageData;
	destination = (unsigned char *)d_liveImage->image;

	brightness = 0.01 * d_imageBrightness->getValue();
	contrast = 0.01 * d_imageContrast->getValue();
	//contrast = exp(contrast/32.0);

	s_pxc200Lib->SetBrightness(s_framegrabberHandle, brightness, 0);
	s_pxc200Lib->SetContrast(s_framegrabberHandle, contrast, 0);

	if(s_doGrab)
	{
		int imageBPL = d_width * (((s_pixelType & 0xFF)+7) >> 3);
		PhPoint_t imagePos = {2,53};
		PhDim_t imageSize = {d_width, d_height};
		
		PgDrawImagemx(d_imageData, Pg_IMAGE_DIRECT_888, &imagePos, &imageSize,
						imageBPL, 0l);
	}

	// Draw time stamp/framerate
	if( d_timestampButton->getState() )
	{
		time_t timeOfDay;
		static clock_t prevClock;
		clock_t currentClock;
		clock_t diffClock;
		char timeNow[100];
  		char frameRateText[100];
		
		// time stamp
		timeOfDay = time( NULL );
		sprintf(timeNow, "%s", ctime(&timeOfDay));
		PgSetFont("TextFont10");
		PhPoint_t pos = { 6, d_height + 20 };
		PgSetTextColor(Pg_YELLOW);
		PgDrawText( timeNow, strlen(timeNow), &pos, 0);

		// frame rate
		currentClock = clock();
		diffClock = currentClock - prevClock;
		prevClock = currentClock;
		static int fr[20];
		static int count = 0;
		fr[count % 20] = (int) ((float) CLOCKS_PER_SEC / diffClock);;
		count++;
		int frameRate = 0;
		for (int i = 0; i < 20; i++)
			frameRate += fr[i];
		frameRate /= 20;
	
		pos.x = 6;
		pos.y = d_height + 35;
		sprintf(frameRateText, "Frame Display Rate %d", frameRate);
		PgSetTextColor(Pg_YELLOW);
		PgDrawText(frameRateText, strlen(frameRateText), &pos, 0);

	}
	
	// Draw crosshair
	if( d_crosshairButton->getState() )
	{
		PhPoint_t center = {d_width/2, 53+d_height/2};
		PhPoint_t radius = {30,30};
		PgSetStrokeColor(Pg_RED);
		PgDrawILine(d_width/2, 53+(d_height/2-50), d_width/2, 53+(d_height/2+50));
		PgDrawILine((d_width/2-50), 53+d_height/2, d_width/2+50, 53+d_height/2);
		PgDrawArc(&center, &radius, 0x0000, 0xFF00, Pg_DRAW_STROKE|Pg_ARC);
	}

	PgFlush();

	// start next grab
	grab();

}


//======================================================================== 
// PXCCont::s_eventCallback
//========================================================================  
int PXCCont::s_eventCallback(void *object, pid_t pid, void *message,
		size_t size)
{
	return ((PXCCont *)object)->eventCallback(pid, message, size);
}

int PXCCont::eventCallback(pid_t pid, void *message, size_t size)
{
	updateLiveImage();
	return Pt_CONTINUE;
}


//======================================================================== 
// PXCCont::s_menuCallback
//========================================================================  
int PXCCont::s_menuCallback(PtWidget_t *widget,
				void *objPtr, PtCallbackInfo_t *cbinfo)
{
	return ((PXCCont *) objPtr) -> menuCallback(widget, cbinfo);
}


int PXCCont::menuCallback(PtWidget_t *widget, PtCallbackInfo_t *cbinfo)
{
	if( d_fileExit->isWidget(widget) )
		exitApplication();

	else if( d_about->isWidget(widget) )
		showAboutWindow();

	return Pt_CONTINUE;
}


//======================================================================== 
// PXCCont::s_imageAcquireCallback
//========================================================================  
int PXCCont::s_imageAcquireCallback(PtWidget_t *widget,
				void *objPtr, PtCallbackInfo_t *cbinfo)
{
	return ((PXCCont *) objPtr) -> imageAcquireCallback(widget, cbinfo);
}


int PXCCont::imageAcquireCallback(PtWidget_t *widget, PtCallbackInfo_t *cbinfo)
{
	static int count;
	char filename[100];
	PhRegion_t imageRegion;
	PhRect_t imageWindow;
	int imageLength;
	char *imageSharedMemory;

	s_doGrab = false;

	// Get application window region
	if ( PhRegionQuery( PgGetRegion(), &imageRegion, &imageWindow,NULL,0) == -1)
		cout << "ERROR finding image region" << endl;

	// translate the regions rect to its parent until you hit the bottom
	while (imageRegion.rid)
	{
		PhTranslateRect( &imageWindow, &imageRegion.origin );
		if (PhRegionQuery(imageRegion.parent,&imageRegion,NULL,NULL,0) == -1)
			break;
	}
	
	// Resize our window to the video feed region. We dont need the rest
	imageWindow.lr.x = imageWindow.ul.x + d_width;
	imageWindow.ul.y += 53;
	imageWindow.lr.y = imageWindow.ul.y + d_height;
	
	// get the size of the shared memory needed for the image
	if ( (imageLength = PgReadScreenSize(&imageWindow)) <= 0 )
		cout << "ERROR reading image from screen" << endl;

	// allocate shared memory to contain the image
	if (!(imageSharedMemory = (char*)PgShmemCreate(imageLength,NULL))) 
		cout << "ERROR allocating shared memory for image" << endl;

	// Read the image region on the screen and write to file.
	if ( d_acquiredImage = PgReadScreen(&imageWindow, imageSharedMemory) )
	{
		sprintf(filename, "%s%02d%s", d_fileBaseText->getText(), count, ".jpg");
	    PxWriteImage(filename, d_acquiredImage, NULL, PX_IMAGE_JPG, 0);
	}

	PgShmemDestroy(imageSharedMemory);

	s_doGrab = true;

	count++;

	return Pt_CONTINUE;
}


//======================================================================== 
// PXCCont::s_inputSelectCallback
//========================================================================  
int PXCCont::s_inputSelectCallback(PtWidget_t *widget,
				void *objPtr, PtCallbackInfo_t *cbinfo)
{
	return ((PXCCont *) objPtr) -> inputSelectCallback(widget, cbinfo);
}


int PXCCont::inputSelectCallback(PtWidget_t *widget, PtCallbackInfo_t *cbinfo)
{
	s_doGrab = false;

	// Select mode
	// Note: PXC200L and PXC200AL have S-video only on channel 1
	if ( s_selectCameraModeSVideo->getState() )
	{
		if( s_modelNumber == PXC200_LC || s_modelNumber == PXC200_LC_2 )
		{
			s_selectCamera02->setState(1);
			s_inputChannel = 1;
			s_pxc200Lib->SetCamera(s_framegrabberHandle, s_inputChannel,0);
		} 
		
		s_inputMode = e_sVideo;
		s_pxc200Lib->SetChromaControl(s_framegrabberHandle,SVIDEO);	
	}
	else if ( s_selectCameraModeComposite->getState() )
	{
		s_inputMode = e_composite;
		s_pxc200Lib->SetChromaControl(s_framegrabberHandle,NOTCH_FILTER);
	}

	// Select Camera
	// Note: PXC200L and PXC200AL have S-video only on channel 1
	if ( s_selectCamera01->getState() )
	{
		if( (s_modelNumber == PXC200_LC || s_modelNumber == PXC200_LC_2) 
			&& (s_inputMode == e_sVideo) )
		{
			// set to composite
			s_inputMode = e_composite;
			s_selectCameraModeComposite->setState(1);
		}
		s_inputChannel = 0;
	}
		
	else if ( s_selectCamera02->getState() )
		s_inputChannel = 1;

	else if ( s_selectCamera03->getState() )
	{
		if( (s_modelNumber == PXC200_LC || s_modelNumber == PXC200_LC_2) 
			&& (s_inputMode == e_sVideo) )
		{
			// set to composite
			s_inputMode = e_composite;
			s_selectCameraModeComposite->setState(1);
		}
		s_inputChannel = 2;
	}
	
	else if ( s_selectCamera04->getState() )
	{
		if( (s_modelNumber == PXC200_LC || s_modelNumber == PXC200_LC_2) 
			&& (s_inputMode == e_sVideo) )
		{
			// set to composite
			s_inputMode = e_composite;
			s_selectCameraModeComposite->setState(1);
		}
		s_inputChannel = 3;
	}
	s_pxc200Lib->SetCamera(s_framegrabberHandle, s_inputChannel,0);

	s_doGrab = true;

	return Pt_CONTINUE;
}


//======================================================================== 
// PXCCont::exitApplication
//========================================================================  
void PXCCont::exitApplication()
{
	exit(0);
}


//======================================================================== 
// PXCCont::showAboutWindow
//========================================================================  
void PXCCont::showAboutWindow()
{
	CAlert alert("PXC200 Series Continuous Display",
	 "GUI by Vilas Chitrakaran\n""PXC library from Imagenation\n\n"
		"Mechatronics Laboratory, Clemson University");
	int result = alert.doModalDialog();
}
