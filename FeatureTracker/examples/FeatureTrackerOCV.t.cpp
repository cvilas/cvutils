//==============================================================================
// FeatureTrackerKLT.t.cpp - Example program for FeatureTrackerOCV class
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "FeatureTrackerOCV.hpp"
#include "Pixmap.hpp"
#include <unistd.h>

int main()
{ 
 OCVTrackingContext_t ocvContext;
 FeatureTrackerContext_t tracContext;
 PixmapGray img[2];
 feature_list_t features;
 
 ocvContext.min_dist = 20;
 ocvContext.quality = 0.001;
 ocvContext.block_size = 5;
 ocvContext.max_iter = 100;
 ocvContext.epsilon = 0.01;
 ocvContext.window_size = 3; 
 ocvContext.max_error = 300;
 
 tracContext.num_features = 48;
 tracContext.num_frames = 2;
 tracContext.auto_select_features = false;
 tracContext.display_tracked_features = true;
 
 // create feature list
 if( allocateFeatureList(features, tracContext.num_features) < 0 )
  return -1;

 FeatureTrackerOCV tracker;

 img[0].loadPixmap("images/cam01-0000.ppm");
 img[1].loadPixmap("images/cam01-0000.ppm");

 // initialize system
 if( tracker.initialize(tracContext, ocvContext) != 0 ) {
  fprintf(stderr, "ERROR initializing tracker.\n");
  return -1;
 }

 // track features between frames
 for(int i = 0; i < 2; ++i) {
  if( tracker.processImage(img[i].getPointer(0), img[i].getWidth(), 
                           img[i].getHeight(), features) < 0 ) {
   fprintf(stderr, "ERROR processing image.\n");
   return -1;
  }
  
  // print features
  fprintf(stdout, "== frame %2d ==\n", i);
  for(int j = 0; j < tracContext.num_features; ++j) {
   fprintf(stdout, "%2d (%3.1f, %3.1f)\n", features.features[j].val, 
           features.features[j].x, features.features[j].y);
  }
 }

 // SDL events won't be caught outside processImage(), unless
 // you do this...
 SDL_Event event;
 while( SDL_PollEvent(&event)  ) {
  if(event.type == SDL_QUIT) {
   fprintf(stdout, "\nI was asked to quit!\n");
   return -2;
  }
 }

 freeFeatureList(features);

 return 0;
}
