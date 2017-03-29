//==============================================================================
// PXCCaptureLoop.cpp - Image capture loop using PXC-200AF
// UAV follower experiment
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "PXCCaptureLoop.hpp"

//#define DEBUG

//==============================================================================
PXCCaptureLoop::PXCCaptureLoop()
//==============================================================================
{
 d_frHandle[0] = NULL;
 d_frHandle[1] = NULL;
 d_imgWidth = 0;
 d_imgHeight = 0;
 d_fgHandle = 0;
 d_isInit = false;
 d_libOpenError = true;
 d_triggerChannel = -1;
 d_bpp = 0;
 d_priority = 0;
#ifdef DEBUG 
 fprintf(stderr, "DEBUG [PXCCaptureLoop::PXCCaptureLoop] leaving\n");
#endif
}


//==============================================================================
PXCCaptureLoop::~PXCCaptureLoop()
//==============================================================================
{
 Thread::cancel();
 Thread::join();
 if(d_fgHandle) d_pxcLib.Reset(d_fgHandle);
 if(d_fgHandle) d_pxcLib.FreeFG(d_fgHandle);
 if(d_frHandle[0]) d_frameLib.FreeFrame(d_frHandle[0]);
 if(d_frHandle[1]) d_frameLib.FreeFrame(d_frHandle[1]);
 if(!d_libOpenError) {
  PXC200_CloseLibrary(&d_pxcLib);
  FRAMELIB_CloseLibrary(&d_frameLib);
 }
#ifdef DEBUG 
 fprintf(stderr, "DEBUG [PXCCaptureLoop::~PXCCaptureLoop] leaving\n");
#endif
}


//==============================================================================
int PXCCaptureLoop::initialize(PXCContext_t &cxt)
//==============================================================================
{ 
 d_isInit = false;

 if(geteuid() != 0){
  fprintf(stderr, "[PXCCaptureLoop::initialize]: ERROR. Program must be executed as root.\n");
  return(-1);
 }

 // open libraries
 if( !PXC200_OpenLibrary(&d_pxcLib, sizeof(d_pxcLib)) ) {
  fprintf(stderr, "[PXCCaptureLoop::initialize]: ERROR opening pxc200 library\n");
  d_libOpenError = true;
  return(-1);
 }
 if( !FRAMELIB_OpenLibrary(&d_frameLib, sizeof(d_frameLib)) ) {
  fprintf(stderr, "[PXCCaptureLoop::initialize]: ERROR opening frame library\n");
  d_libOpenError = true;
  PXC200_CloseLibrary(&d_pxcLib);
  return(-1);
 }
 d_libOpenError = false;
 
 // allocate framegrabber
 if( (d_fgHandle = d_pxcLib.AllocateFG(cxt.board_number)) == 0 ) {
  fprintf(stderr, "[PXCCaptureLoop::initialize]: ERROR finding frame grabber\n");
  return(-1);
 }

 d_pxcLib.Reset(d_fgHandle);
 
 // Setup video capture parameters
 if( d_pxcLib.SetCamera(d_fgHandle, cxt.video_channel, 0) == 0) {        
  fprintf(stderr, "[PXCCaptureLoop::initialize]: ERROR setting camera\n");
  return(-1);
 }
 fprintf(stdout, "[PXCCaptureLoop::initialize]: Video source: %d\n", 
                 d_pxcLib.GetCamera(d_fgHandle));
 
 if( d_pxcLib.SetPixelFormat(d_fgHandle, cxt.pixel_format) == 0 ) {
  fprintf(stderr, "[PXCCaptureLoop::initialize]: ERROR setting pixel format\n");
  return(-1);
 }
 if (cxt.pixel_format == PBITS_Y8)
  d_bpp = 1;
 if (cxt.pixel_format == PBITS_RGB24)
  d_bpp = 3;
 
 
 if( d_pxcLib.SetVideoDetect(d_fgHandle, cxt.video_format) == 0 ) {
  fprintf(stderr, "[PXCCaptureLoop::initialize]: ERROR setting video format\n");
  return(-1);
 }
 d_imgWidth = d_pxcLib.GetWidth(d_fgHandle);
 d_imgHeight = d_pxcLib.GetHeight(d_fgHandle);
 fprintf(stdout, "[PXCCaptureLoop::initialize]: Image size: %d x %d, %d Bpp\n", 
         d_imgWidth, d_imgHeight, d_bpp);
 
 // Allocate frames
 if ( cxt.pixel_format != PBITS_Y8 && cxt.pixel_format != PBITS_RGB24 ) {
  fprintf(stderr, "[PXCCaptureLoop::initialize]: ERROR unsupported pixel format\n");
  return(-1);
 }
 for(int i = 0; i < 2; ++i) {
  if( (d_frHandle[i] = d_frameLib.AllocateMemoryFrame(d_imgWidth, d_imgHeight, cxt.pixel_format)) == 0) {
   fprintf(stderr, "[PXCCaptureLoop::initialize]: ERROR allocating frame buffer %d\n", i);
   d_frHandle[i] = NULL;
   return(-1);
  }
 }

 // set latch triggered input
 d_triggerChannel = cxt.trigger_channel;
 if( d_triggerChannel < 0 ) {
  fprintf(stdout, "[PXCCaptureLoop::initialize]: No external triggering\n");
 } else {
  if( d_pxcLib.SetIOType(d_fgHandle, d_triggerChannel, LATCH_RISING) == 0) {
   fprintf(stderr, "[PXCCaptureLoop::initialize]: ERROR configuring input trigger type\n");
   return(-1);
  }
  fprintf(stdout, "[PXCCaptureLoop::initialize]: Trigger channel: %d\n", d_triggerChannel);
 }
 d_priority = cxt.thread_priority;
 d_isInit = true;
 
#ifdef DEBUG 
 fprintf(stderr, "DEBUG [PXCCaptureLoop::initialize] success\n");
#endif

 return 0;
}


//==============================================================================
int PXCCaptureLoop::getImageProperties(int &w, int &h, int &bpp)
//==============================================================================
{
 if( !d_isInit ) {
  fprintf(stderr, "[PXCCaptureLoop::getImageProperties]: ERROR PXC not initialized yet\n");
  return(-1);
 }
 w = d_imgWidth;
 h = d_imgHeight;
 bpp = d_bpp;
 
 return 0;
}


//==============================================================================
int PXCCaptureLoop::startCaptureLoop()
//==============================================================================
{ 
 if( !d_isInit ) {
  fprintf(stderr, "[PXCCaptureLoop::startCaptureLoop]: ERROR PXC not initialized yet\n");
  return(-1);
 }
 
 if( Thread::run((void *)this) != 0) {
  fprintf(stderr, "[PXCCaptureLoop::startCaptureLoop]: ERROR starting image capture thread\n");
  return(-1);
 }
 sched_yield();

#ifdef DEBUG 
 fprintf(stderr, "DEBUG [PXCCaptureLoop::startCaptureLoop] success\n");
#endif

 return 0;
}


//==============================================================================
int PXCCaptureLoop::processImage(const unsigned char *fbr, int w, int h, int bpp)
//==============================================================================
{ 
 w = w; h = h; bpp = bpp; fbr = fbr;
#ifdef DEBUG 
 fprintf(stderr, "DEBUG [PXCCaptureLoop::processImage] success\n");
#endif
 return 0;
}


//==============================================================================
void PXCCaptureLoop::enterThread(void *arg)
//==============================================================================
{
#ifdef DEBUG 
 fprintf(stderr, "DEBUG [PXCCaptureLoop::enterThread] entering\n");
#endif
 struct sched_param param;
 int policy;

 pthread_getschedparam(pthread_self(), &policy, &param);
 policy = SCHED_FIFO;
 param.sched_priority = ((PXCCaptureLoop *)arg)->d_priority;
 pthread_setschedparam(pthread_self(), policy, &param);
 pthread_getschedparam(pthread_self(), &policy, &param);
 fprintf(stdout, "[PXCCaptureLoop::enterThread]: Thread priority set to %d\n", 
         param.sched_priority);
#ifdef DEBUG 
 fprintf(stderr, "DEBUG [PXCCaptureLoop::enterThread] success\n");
#endif
}


//==============================================================================
int PXCCaptureLoop::executeInThread(void *arg)
//==============================================================================
{
#ifdef DEBUG 
 fprintf(stderr, "DEBUG [PXCCaptureLoop::executeInThread] entering\n");
#endif
 struct timespec t;
 t.tv_sec = 0;
 t.tv_nsec = 1;
 PXCCaptureLoop *classPtr = (PXCCaptureLoop *)arg;
 int trigChannel = classPtr->d_triggerChannel;
 int mask = 1 << trigChannel;
 int gh[2];
 int w = classPtr->d_imgWidth;
 int h = classPtr->d_imgHeight;
 int bpp = classPtr->d_bpp;
 unsigned char *fbAddr[2]; 

 fbAddr[0] = (unsigned char *)classPtr->d_frameLib.FrameBuffer(classPtr->d_frHandle[0]);
 fbAddr[1] = (unsigned char *)classPtr->d_frameLib.FrameBuffer(classPtr->d_frHandle[1]);

 ////////////////////// untriggered capture - use double buffering /////////////
 if( trigChannel < 0 ) {
  gh[0] = classPtr->d_pxcLib.Grab(classPtr->d_fgHandle, d_frHandle[0], QUEUED);
  gh[1] = classPtr->d_pxcLib.Grab(classPtr->d_fgHandle, d_frHandle[1], QUEUED);
  classPtr->d_pxcLib.WaitFinished(classPtr->d_fgHandle, gh[0]);
  
  for(;;) {
   nanosleep(&t, NULL);
   if( classPtr->processImage(fbAddr[0], w, h, bpp) != 0 ) break;
   gh[0] = classPtr->d_pxcLib.Grab(classPtr->d_fgHandle, d_frHandle[0], QUEUED);
   nanosleep(&t, NULL);
   classPtr->d_pxcLib.WaitFinished(classPtr->d_fgHandle, gh[1]);
   if( classPtr->processImage(fbAddr[1], w, h, bpp) != 0 ) break;
   gh[1] = classPtr->d_pxcLib.Grab(classPtr->d_fgHandle, d_frHandle[1], QUEUED);
   classPtr->d_pxcLib.WaitFinished(classPtr->d_fgHandle, gh[0]);
   pthread_testcancel();
  }
 }
 
 /////////////////////// triggered capture - use single buffering //////////////
 else {
  for(;;) {
   classPtr->d_pxcLib.WaitAnyEvent(classPtr->d_fgHandle, classPtr->d_fgHandle, mask, 1, QUEUED);
   gh[0] = classPtr->d_pxcLib.Grab(classPtr->d_fgHandle, d_frHandle[0], QUEUED);
   classPtr->d_pxcLib.WaitFinished(classPtr->d_fgHandle, gh[0]);
   if( classPtr->processImage(fbAddr[0], w, h, bpp) != 0 )
    break;
   pthread_testcancel();
   nanosleep(&t, NULL);
  }
 }

#ifdef DEBUG 
 fprintf(stderr, "DEBUG [PXCCaptureLoop::executeInThread] leaving\n");
#endif
  
 return 0;
}


//==============================================================================
void PXCCaptureLoop::exitThread(void *arg)
//==============================================================================
{
 arg = arg;
#ifdef DEBUG 
 fprintf(stderr, "DEBUG [PXCCaptureLoop::exitThread] success\n");
#endif
}


