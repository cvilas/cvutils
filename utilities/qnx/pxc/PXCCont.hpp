//======================================================================== 
// Project: OCTOR   
// ---------------------------------------------------------------------- 
// Package: PXC Series Continuous Display
// Authors: Vilas Kumar Chitrakaran
// Start Date: Tue Sep 30 09:30:10 EDT 2003
// Compiler: GCC 2.95.3
// Operating System: QNX 6.2.1 
// ----------------------------------------------------------------------  
// File: PXCCont.hpp
// Interface of the class PXCCont.
//========================================================================  
 

#ifndef INCLUDED_PXCCont_hpp
#define INCLUDED_PXCCont_hpp
#include <stdio.h>
#include <iostream.h>
#include <photon/PxImage.h>
#include <sys/neutrino.h>
#include <math.h>

extern "C" {		// jpeglib is not C++ safe already
#include <jpeglib.h>
}
#include "QWidgets.hpp"
#include "Status.hpp"

#include "pxc200/pxc.h"
//#include "frame.h"


//======================================================================== 
// class PXCCont
// ----------------------------------------------------------------------
// \brief
// A continuous display program for the PXC series framegrabber.
//
// This class uses the PXC200 library from Imagenation.
//
//========================================================================  

class PXCCont : public CMainWindow
{
 public:
 
	PXCCont();
	~PXCCont ();
			
 protected:
		 
	// ---------- init functions ----------
	int initPXC();
		// Initialize framegrabber
		
	void initMainWindow();
		// start the GUI
		
	void createLiveImage(CWidget *parent);
		// Set up the image window


	// ----------- Grab and update -------
	void grab();
		// Grab a frame.
		
	void updateLiveImage();
		// Update image on the screen


	// ---------- GUI callbacks ----------
	static int s_eventCallback(void *object, pid_t pid, void *message,
			size_t size);
		// Calls handler for internel events

	int eventCallback(pid_t pid, void *message, size_t size);
		// Handler for internel events
	
	static int s_menuCallback(PtWidget_t *widget, void *, PtCallbackInfo_t *);
		// Calls handler for user interaction with GUI.

	int menuCallback(PtWidget_t *widget, PtCallbackInfo_t *cbinfo);
		// Handles user interaction with GUI.

	static int s_inputSelectCallback(PtWidget_t *widget, void *, PtCallbackInfo_t *);
		// Calls handler for user interaction with camera selection buttons.

	int inputSelectCallback(PtWidget_t *widget, PtCallbackInfo_t *cbinfo);
		// Handles user interaction with camera selection buttons.

	static int s_imageAcquireCallback(PtWidget_t *widget, void *, PtCallbackInfo_t *);
		// Calls handler for user interaction with image control widgets.

	int imageAcquireCallback(PtWidget_t *widget, PtCallbackInfo_t *cbinfo);
		// Handles user interaction with image control widgets.


	// --------- Helper functions --------
	void showAboutWindow();
		// Help window

	void exitApplication();
		// Exit the application
				
 public:
 	Status d_status;
 		// Operational status information

 protected:
	static unsigned long s_framegrabberHandle;
		// framegrabber handle
	
	static int s_videoType;
		// Video format
		
	static int s_pixelType;
		// image data type (RGB24, YUV444, etc)
		
	static int s_modelNumber;
		// PXC model number
			
	static int s_inputChannel;
		// Get the input channel number
	
	static int s_inputMode;
		// Input mode (S-Video/composite)
		
	enum inputMode
	{
		e_sVideo,
		e_composite
	};
	
	static bool s_doGrab;
		// flag - grab image
	
	int d_width;
	int d_height;
		// Frame size

	FRAME __PX_FAR *d_frameBuffer;
		// framebuffer pointers
	
	unsigned char *d_imageData;
		// Pointer to image data in framebuffer
	
	PhImage_t *d_liveImage;
		// photon image data pointer
	
	PhImage_t *d_acquiredImage;
		// image acquired for saving to disk
			
	int d_liveImageBufferSize;
		// Photon display image buffer size
		
	int d_bytesPerPixel;
		// photon image bytes per pixel
			
	// ----- Widgets -----
	CGroup *d_group;       
	CGroup *d_imagesGroup;
	CLabel *d_liveImageWidget;
	CMenuBar *d_menuBar;
	CMenuButton *d_fileExit;
	CMenuButton *d_about;
	CButton *d_acquireButton;
	CButton *d_saveButton;
	CSlider *d_imageBrightness;
	CSlider *d_imageContrast;
	COnOffButton *d_timestampButton;
	COnOffButton *d_crosshairButton;
	COnOffButton *d_fastUpdateButton;

	// ========== END OF INTERFACE ==========
  private:
  	int d_channelId;
  	int d_connectionId;

};

#endif

