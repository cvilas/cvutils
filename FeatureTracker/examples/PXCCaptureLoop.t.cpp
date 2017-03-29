//==============================================================================
// PXCCaptureLoop.t.cpp - Examples program for PXCCaptureLoop class
// UAV follower experiment
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "PXCCaptureLoop.hpp"


int main()
{
 PXCContext_t settings;
 PXCCaptureLoop camera;

 settings.board_number = -1;
 settings.video_channel = 0;    
 settings.pixel_format = PBITS_Y8;
 settings.video_format = NTSC_FORMAT;
 settings.trigger_channel = -1; 
 settings.thread_priority = 10;
 
 if( camera.initialize(settings) != 0 )
  return -1;
  
 if( camera.startCaptureLoop() != 0 )
  return -1;
 
 fprintf(stdout, "thread started\n");
 sleep(10);
 
 return 0;
}