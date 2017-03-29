//==============================================================================
// FeatureTrackerKLT.cpp
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "FeatureTrackerKLT.hpp"

//#define DEBUG

//==============================================================================
FeatureTrackerKLT::FeatureTrackerKLT()
//==============================================================================
{
 d_kltc = NULL;
 d_numFeatures = 0;
 d_numFrames = 0;
 d_frameNumber = 0;
 d_autoSelect = true;
 d_displayOn = true;
 d_featureList = NULL;
 d_featureTable = NULL;
 d_screen = NULL;
}


//==============================================================================
FeatureTrackerKLT::~FeatureTrackerKLT()
//==============================================================================
{
 if(d_featureList) KLTFreeFeatureList(d_featureList);
 if(d_featureTable) KLTFreeFeatureTable(d_featureTable);
#ifdef DEBUG
 fprintf(stderr, "[FeatureTrackerKLT::~FeatureTrackerKLT] Leaving.\n");
#endif
}


//==============================================================================
int FeatureTrackerKLT::initialize(FeatureTrackerContext_t &ftc, KLT_TrackingContext kltc)
//==============================================================================
{
 d_frameNumber = 0;
 d_kltc = kltc;
 
 // override some user specified parameters
 d_kltc->sequentialMode = true;
 
 d_numFeatures = ftc.num_features;
 d_numFrames = ftc.num_frames;
 d_autoSelect = ftc.auto_select_features;
 d_displayOn = ftc.display_tracked_features;
 d_featureList = KLTCreateFeatureList(d_numFeatures);
 d_featureTable = KLTCreateFeatureTable(d_numFrames, d_numFeatures);
 KLTSetVerbosity(0);
 
 return 0;
}


//==============================================================================
int FeatureTrackerKLT::processImage(unsigned char *buf, int w, int h, feature_list_t &features)
//==============================================================================
{
 SDL_Event event;
 float x = 0;
 float y = 0;

 if( features.num_features != d_numFeatures ) {
  fprintf(stderr, "[FeatureTrackerKLT::processImage] ERROR. Buffer size mismatch ->\n"); 
  fprintf(stderr, "-> User buffer (arg 4) holds %d features, but we are tracking %d features.\n", 
  features.num_features, d_numFeatures);
  return -1;
 }

 // check if initialize() was called
 if( d_featureList == NULL ) {
  fprintf(stderr, "%s\n", "[FeatureTrackerKLT::processImage] ERROR. Must call initialize first.");
  return -1;
 }
 
 if( d_frameNumber == d_numFrames) {
  fprintf(stderr, "\n%s\n%s\n", 
          "[FeatureTrackerKLT::processImage] WARNING Exceeded specified frame number limit ->",
          "-> Feature table won't update further.");
 }
 
 // first frame - select features
 if(d_frameNumber == 0) {
  
  // automatic or manual feature selection
  if(d_autoSelect) {
   KLTSelectGoodFeatures(d_kltc, buf, w, h, d_featureList);
  } else {  
  
   // initialize display
   if( d_display.init(w, h, "FeatureTrackerKLT") != 0)
    return -1;
   d_screen = d_display.getScreenPointer();
   
   // img->SDL surface
   snprintf(d_message, 80, "ATTENTION: Please select %d feature points\0", d_numFeatures);
   if( d_display.updateScreenBuffer((char *)buf, w, h, 1, d_message) != 0)
    return -1;
   d_display.refresh();
   
   // let user select features
   int nFeatSelected = 0;
   while( (nFeatSelected < d_numFeatures) && SDL_WaitEvent(&event)  ) {
    switch(event.type) {
     case SDL_MOUSEBUTTONDOWN:
      x = event.button.x;
      y = event.button.y;
      fprintf(stdout, "Selected feature %3d at (%6.2f, %6.2f)\n", nFeatSelected, x, y); 
      d_featureList->feature[nFeatSelected]->x = x;
      d_featureList->feature[nFeatSelected]->y = y;
      d_featureList->feature[nFeatSelected]->val = KLT_TRACKED;
      ++nFeatSelected;
      boxColor(d_screen, (short)x-2, (short)y-2, (short)x+2, 
               (short)y+2, 0xff0000ff);
      d_display.refresh();
     break;
     case SDL_QUIT:
      fprintf(stdout, "\n[FeatureTrackerKLT::processImage] I was asked to quit!\n");
      d_displayOn = false;
      return -2;
     break;
     default:
     break;
    }
   }
  }
  KLTStoreFeatureList(d_featureList, d_featureTable, d_frameNumber);
 } else {
  // track features in this frame
  KLTTrackFeatures(d_kltc, buf, buf, w, h, d_featureList);
  if( d_frameNumber < d_numFrames )
   KLTStoreFeatureList(d_featureList, d_featureTable, d_frameNumber);
 }
 
 // prepare screen display 
 if(d_displayOn) {
  if(d_screen == NULL) if( d_display.init(w, h, "FeatureTrackerKLT") != 0) return -1;
  if( d_display.updateScreenBuffer((char *)buf, w, h, 1, NULL) != 0) return -1;
  d_screen = d_display.getScreenPointer();
 }

 // copy features into list, update display
 for(int i = 0; i < d_numFeatures; ++i) {
  if (d_featureList->feature[i]->val == KLT_TRACKED) {
   features.features[i].val = 0;
   features.features[i].x = d_featureList->feature[i]->x;
   features.features[i].y = d_featureList->feature[i]->y;
   if(d_displayOn) 
    boxColor(d_screen, (short)(features.features[i].x)-2, (short)(features.features[i].y)-2, 
             (short)(features.features[i].x)+2, (short)(features.features[i].y)+2, 0xff0000ff);
  } else {
   features.features[i].val = -1;
   features.features[i].x = 0;
   features.features[i].y = 0;
  }
 }
 
 // update display
 if(d_displayOn) {
  snprintf(d_message, 80, "Features tracked: %d/%d\0", KLTCountRemainingFeatures(d_featureList), d_numFeatures);
  stringColor(d_screen, 2, h-10, d_message, 0xfd1b04FF);
  d_display.refresh();
 }
 
 // handle user quit
 while( SDL_PollEvent(&event)  ) {
  if(event.type == SDL_QUIT) {
   fprintf(stdout, "\n[FeatureTrackerKLT::processImage] I was asked to quit!\n");
   return -2;
  }
 }
 ++d_frameNumber;
 return d_frameNumber;
}


//==============================================================================
int FeatureTrackerKLT::writeFeatureTable(const char *fileBaseName)
//==============================================================================
{ 
 char name[80];
 snprintf(name, 80, "%s.txt\0", fileBaseName);
 KLTWriteFeatureTable(d_featureTable, name, "%5.1f");
 snprintf(name, 80, "%s.ft\0", fileBaseName);
 KLTWriteFeatureTable(d_featureTable, name, NULL);
 return 0;
}

