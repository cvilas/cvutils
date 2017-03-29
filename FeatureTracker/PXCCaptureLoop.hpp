//==============================================================================
// PXCCaptureLoop.hpp - Image capture loop using PXC-200AF
// Vilas Chitrakaran, May 2006
//==============================================================================

#ifndef INCLUDED_PXCCAPTURELOOP_HPP
#define INCLUDED_PXCCAPTURELOOP_HPP

#include "pxc200/pxc.h"
#include "pxc200/frame.h"
#include "putils/Thread.hpp"
#include <stdio.h>


//==============================================================================
/*! \struct _PXCContext
    \brief Parameters for the PXC200AF framegrabber. Use with class 
    PXCCaptureLoop. */
//==============================================================================
typedef struct _PXCContext
{
 int board_number;     //!< -1 to request any available
 int video_channel;    //!< Video channel number, see pxc.h.
 int pixel_format;     //!< see frame.h for image data types. Use PBITS_Y8 or PBITS_RGB24
 int video_format;     //!< see pxc.h for video detect types. Use NTSC_FORMAT
 int trigger_channel;  //!< -1 for no external triggering; else triggers on rising edge. 
 int thread_priority;  //!< Priority of image capturing thread.
}PXCContext_t;


//==============================================================================
// class PXCCaptureLoop
//------------------------------------------------------------------------------
// \brief
// A QNX specific interface to capture images using the Imagenation PXC200AF 
// frame grabber.
//
// An object of this class interfaces with a PXC200 AF framegrabber through 
// its device driver. The object initiates a separate thread for image
// capturing and transfers image data to a user specified memory buffer through
// a user implemented function. This class is specifically written to work
// with the QNX 6.2.1 device driver for PXC200AF. More information on PXC series
// framegrabbers are available here: http://www.imagenation.com/pxcfamily.html.
//
// <b>Example Program:</b>
// \include PXCCaptureLoop.t.cpp
//==============================================================================

class PXCCaptureLoop : public Thread
{ 
 public:
  PXCCaptureLoop();
   // Default constructor. 
   
  virtual ~PXCCaptureLoop();
   // Destructor. Frees any allocated resources, shuts down the framegrabber.
   
  int initialize(PXCContext_t &cxt);
   // Initializes the frame grabber.
   //  cxt     settings specific to framegrabber.
   //  return  0 on success, -1 on error (error message redirected to stderr).
    
  int startCaptureLoop();
   // start image capture loop. Method processImage() is called 
   // everytime a new image is acquired.
   //  return  0 on success, -1 on error (error message redirected to stderr).
  
  int getImageProperties(int &w, int &h, int &bpp);
   // Get properties of the images being captured by the camera
   //  w, h    Width and height of the image.
   //  bpp     Image bytes per pixel (1 = 8 bit grayscale, 3 = 24 bit RGB).
   //  return  0 on success, -1 on error (error message redirected to stderr).
   
 protected:
  virtual int processImage(const unsigned char *fbr, int w, int h, int bpp);
   // Reimplement this function in a derived class. This method is automatically
   // called upon every successful acquisition of an image. It is upto the 
   // implementation to ensure that processing delays do not cause dropped 
   // frames. A suggested implementation would do nothing more than memcpy() 
   // the framebuffer to a user specified buffer and return immediately. A 
   // separate thread/process can then process the contents of the copied buffer.
   //  fbr     Pointer to frame buffer containing current image update.
   //  w, h    Width and height of the image.
   //  bpp     Image bytes per pixel (1 = 8 bit grayscale, 3 = 24 bit RGB).
   //  return  Implementation must return 0 on success, non-zero on error.

  virtual void enterThread(void *arg);
   // Reimplemented from Thread class
   
  virtual int executeInThread(void *arg);
   // Reimplemented from Thread class

  virtual void exitThread(void *arg);
   // Reimplemented from Thread class

 protected:
  int d_imgWidth;
  int d_imgHeight;
  int d_bpp;
 private:
  int d_triggerChannel;
  PXC200 d_pxcLib;
  FRAMELIB d_frameLib;
  int d_fgHandle;
  FRAME __PX_FAR *d_frHandle[2];
  int d_priority;
  bool d_isInit;
  bool d_libOpenError;
};

#endif // INCLUDED_PXCCAPTURELOOP_HPP
