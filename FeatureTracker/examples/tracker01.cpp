//==============================================================================
// tracker01.cpp - A complete example for online feature tracking system.
// UAV follower experiment
// Vilas Chitrakaran, May 2006
//
// This test program continuously acquires images and tracks user selected 
// features in a sequential capture->track->display loop (no ext. triggering).
//==============================================================================

#include "PXCCaptureLoop.hpp"
#include "FeatureTrackerOCV.hpp"
#include "FeatureClientServer.hpp"


//==============================================================================
class PXCTrackLoop : public PXCCaptureLoop
//==============================================================================
{
 public:
  PXCTrackLoop();
  ~PXCTrackLoop();
  int initSystem(PXCContext_t &cam_cxt, FeatureTrackerContext_t &ft_cxt,
                 OCVTrackingContext_t &ocvtc, FeatureServerContext_t &fs_cxt);
  int processCycle();
 protected:
  virtual int processImage(const unsigned char *fbr, int w, int h, int bpp);
 private:
  RWLock d_rwLock;
  int d_srcImgNum;
  unsigned char *d_copyBuf;
  FeatureTrackerOCV d_fTracker;
  FeatureServer d_fServer;
  bool d_sysIsInit;
  bool d_fgIsInit;
  feature_list_t d_featureList;
};


//==============================================================================
PXCTrackLoop::PXCTrackLoop()
//==============================================================================
{
 d_sysIsInit = false;
 d_fgIsInit = false;
 d_srcImgNum = 0;
 d_copyBuf = NULL;
}


//==============================================================================
PXCTrackLoop::~PXCTrackLoop()
//==============================================================================
{
 if(d_copyBuf)
  free(d_copyBuf);
 freeFeatureList(d_featureList);
}


//==============================================================================
int PXCTrackLoop::initSystem(PXCContext_t &cam_cxt, 
                             FeatureTrackerContext_t &ft_cxt,
                             OCVTrackingContext_t &ocvtc, 
                             FeatureServerContext_t &fs_cxt)
//==============================================================================
{
 d_sysIsInit = false;
 d_fgIsInit = false;
 d_srcImgNum = 0;
 
 if( allocateFeatureList(d_featureList, ft_cxt.num_features) < 0 )
  return -1;

 // init capture source
 if( PXCCaptureLoop::initialize(cam_cxt) != 0)
  return -1;
 
 // create copy buffer
 if( (d_copyBuf = (unsigned char *)realloc(d_copyBuf, d_imgWidth * d_imgHeight * d_bpp + sizeof(int))) == NULL) {
  fprintf(stderr, "[PXCTrackLoop::initSystem] ERROR creating buffer.\n");
  return -1;
 }

 // init tracker
 if( d_fTracker.initialize(ft_cxt, ocvtc) != 0 )
  return -1;

 d_fgIsInit = true; 

 // start capture thread
 if( PXCCaptureLoop::startCaptureLoop() != 0)
  return -1;
 
 // init and start server thread
 if( d_fServer.initialize(fs_cxt) != 0 )
  return -1;

 d_sysIsInit = true;

 return 0;
}


//==============================================================================
int PXCTrackLoop::processCycle()
//==============================================================================
{
 if( !d_sysIsInit ) {
  fprintf(stderr, "[PXCTrackLoop::processCycle]: ERROR - system not initialized yet\n");
  return(-1);
 }

 unsigned char *buf = d_copyBuf;
 int frPrNum = 0;
  
 d_rwLock.readLock();
 int fSrcNum = *(int *)buf;
 buf += sizeof(int);
 if( ( frPrNum = d_fTracker.processImage(buf, d_imgWidth, d_imgHeight, d_featureList)) < 0 ) {
  d_rwLock.unlock();
  return frPrNum;
 }
 d_rwLock.unlock();
 
 if( d_fServer.updateFeatures(d_featureList, fSrcNum) != 0)
  return -1;

 
 return frPrNum;
}


//==============================================================================
int PXCTrackLoop::processImage(const unsigned char *fbr, int w, int h, int bpp)
//==============================================================================
{
 if( !d_fgIsInit ) {
  fprintf(stderr, "[PXCTrackLoop::processImage]: ERROR PXC not initialized yet\n");
  return(-1);
 }

 unsigned char *buf = d_copyBuf;

 d_rwLock.writeLock();
 * (int *)buf = d_srcImgNum;
 buf += sizeof(int);
 memcpy(buf, fbr, w * h * bpp);
 d_rwLock.unlock();
 
 ++d_srcImgNum;

 return 0;
}



//==============================================================================
int main()
//==============================================================================
{
 PXCTrackLoop cv;
 PXCContext_t cam_cxt;
 FeatureTrackerContext_t ft_cxt;
 OCVTrackingContext_t ocvtc;
 FeatureServerContext_t fs_cxt;

 //--------------------------------------------------------------------
 // Thread priority: this program > feature server > capture thread seems
 // to work best.
 //--------------------------------------------------------------------
 struct sched_param param;
 int policy;
 pthread_getschedparam(pthread_self(), &policy, &param);
 policy = SCHED_FIFO;
 param.sched_priority = 40;
 pthread_setschedparam(pthread_self(), policy, &param);
 pthread_getschedparam(pthread_self(), &policy, &param);
 fprintf(stdout, "[main]: Thread priority set to %d\n", param.sched_priority);

 // camera capture board settings
 cam_cxt.board_number = -1;
 cam_cxt.video_channel = 0;
 cam_cxt.pixel_format = PBITS_Y8;
 cam_cxt.video_format = NTSC_FORMAT;
 cam_cxt.trigger_channel = -1; // Free running framegrabber, no ext. trigger
 cam_cxt.thread_priority = 10;
 
 // general settings for feature tracker
 ft_cxt.num_features = 10;
 ft_cxt.num_frames = 100;
 ft_cxt.auto_select_features = false;
 ft_cxt.display_tracked_features = true;
 
 // settings specific to tracking algorithm
 ocvtc.min_dist = 10;
 ocvtc.quality = 0.01;
 ocvtc.block_size = 3;
 ocvtc.max_iter = 20;
 ocvtc.epsilon = 0.01;
 ocvtc.window_size = 5; 
 ocvtc.max_error = 300;

 // feature server settings
 fs_cxt.port = 8000;
 fs_cxt.thread_priority = 20;
 fs_cxt.num_features = ft_cxt.num_features;

 // system init 
 if( cv.initSystem(cam_cxt, ft_cxt, ocvtc, fs_cxt) != 0)
  return -1;

 sleep(1);
 
 // capture->process->serve loop
 int fr;
 while(1) {
  usleep(1000);
  if( (fr = cv.processCycle()) < 0 )
   break;
 }
 
 if(fr == -1) {
  fprintf(stderr, "ERROR occurred.\n");
 }
 
 return 0;
}

