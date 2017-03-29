//==============================================================================
// FeatureTrackerKLT.t.cpp - Example program for FeatureTrackerKLT class
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "FeatureTrackerKLT.hpp"
#include "Pixmap.hpp"
#include <unistd.h>

int main()
{ 
 KLT_TrackingContext kltContext;
 FeatureTrackerContext_t tracContext;
 PixmapGray img[2];
 feature_list_t features;
 
 kltContext = KLTCreateTrackingContext();
 kltContext->lighting_insensitive = true;
 kltContext->writeInternalImages = false;
 kltContext->affineConsistencyCheck = -1;
 kltContext->window_width = 9;
 kltContext->window_height = 9;
 kltContext->max_iterations = 100;
 kltContext->mindist = 10;
 kltContext->smoothBeforeSelecting = true;
 
 tracContext.num_features = 4;
 tracContext.num_frames = 2;
 tracContext.auto_select_features = false;
 tracContext.display_tracked_features = true;
 
 // create feature list
 if( allocateFeatureList(features, tracContext.num_features) < 0 )
  return -1;

 FeatureTrackerKLT tracker;

 img[0].loadPixmap("images/box0.pgm");
 img[1].loadPixmap("images/box1.pgm");

 // initialize system
 if( tracker.initialize(tracContext, kltContext) != 0 ) {
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

 // Write results to text.
 if( tracker.writeFeatureTable("trackedFeatures") != 0 ) {
  fprintf(stderr, "ERROR writing feature tables.\n");
  return -1;
 }
 
 freeFeatureList(features);
 KLTFreeTrackingContext(kltContext);

 return 0;
}
