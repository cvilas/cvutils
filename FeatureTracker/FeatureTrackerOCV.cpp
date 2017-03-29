//==============================================================================
// FeatureTrackerOCV.cpp
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "FeatureTrackerOCV.hpp"

//#define DEBUG


//==============================================================================
FeatureTrackerOCV::FeatureTrackerOCV()
//==============================================================================
{
 d_screen = NULL;
 d_message[0] = '\0';
 d_numFeatures = 0;
 d_numFrames = 0;
 d_featureList[0] = 0, d_featureList[1] = 0;
 d_trackStatus = 0;
 d_swapArray = 0;
 d_image = d_prevImage = d_pyramid = d_prevPyramid = d_swapImg = 0;
 d_frameNumber = 0;
 d_autoSelect = true;
 d_displayOn = true;
 d_trackerFlags = 0;
 d_numDetectedFeatures = 0;
 d_trackedFeaturesIndices = 0;
 d_trackingErrors = 0;
}


//==============================================================================
FeatureTrackerOCV::~FeatureTrackerOCV()
//==============================================================================
{
 if(d_image) cvReleaseImage(&d_image);
 if(d_prevImage) cvReleaseImage(&d_prevImage);
 if(d_pyramid) cvReleaseImage(&d_pyramid);
 if(d_prevPyramid) cvReleaseImage(&d_prevPyramid);
 if(d_featureList[1]) cvFree((void**)(&d_featureList[1]));
 if(d_featureList[0]) cvFree((void**)(&d_featureList[0]));
 if(d_trackStatus) cvFree((void**)(&d_trackStatus));
 if(d_trackedFeaturesIndices) free(d_trackedFeaturesIndices);
 if(d_trackingErrors) free(d_trackingErrors);
#ifdef DEBUG
 fprintf(stderr, "[FeatureTrackerOCV::~FeatureTrackerOCV] Leaving.\n");
#endif
}


//==============================================================================
int FeatureTrackerOCV::initialize(FeatureTrackerContext_t &ftc, OCVTrackingContext_t &ocvt)
//==============================================================================
{
 memcpy(&d_trackingContext, &ocvt, sizeof(OCVTrackingContext_t));
 d_frameNumber = 0;
 d_numFeatures = ftc.num_features;
 d_numFrames = ftc.num_frames;
 d_autoSelect = ftc.auto_select_features;
 d_displayOn = ftc.display_tracked_features;
 d_featureList[0] = (CvPoint2D32f*)cvAlloc(d_numFeatures * sizeof(d_featureList[0][0]));
 if(d_featureList[0] == NULL) {
  fprintf(stderr, "[FeatureTrackerOCV::initialize] ERROR in memory allocation.\n");
  return -1;
 }
 d_featureList[1] = (CvPoint2D32f*)cvAlloc(d_numFeatures * sizeof(d_featureList[0][0]));
 if(d_featureList[1] == NULL) {
  fprintf(stderr, "[FeatureTrackerOCV::initialize] ERROR in memory allocation.\n");
  return -1;
 }
 d_trackStatus = (char *)cvAlloc(d_numFeatures);
 if(d_trackStatus == NULL) {
  fprintf(stderr, "[FeatureTrackerOCV::initialize] ERROR in memory allocation.\n");
  return -1;
 }
 d_trackedFeaturesIndices = (int *)malloc(d_numFeatures * sizeof(int));
 if(d_trackedFeaturesIndices == NULL) {
  fprintf(stderr, "[FeatureTrackerOCV::initialize] ERROR in memory allocation.\n");
  return -1;
 }
 d_trackingErrors = (float *)malloc(d_numFeatures * sizeof(float));
 if(d_trackingErrors == NULL) {
  fprintf(stderr, "[FeatureTrackerOCV::initialize] ERROR in memory allocation.\n");
  return -1;
 }
  
 d_trackerFlags = 0;
 d_numDetectedFeatures = 0;
 return 0;
}


//==============================================================================
int FeatureTrackerOCV::processImage(unsigned char *buf, int w, int h, feature_list_t &features)
//==============================================================================
{
 SDL_Event event;
 float x = 0;
 float y = 0;

 if( features.num_features != d_numFeatures ) {
  fprintf(stderr, "[FeatureTrackerOCV::processImage] ERROR. Feature list size mismatch ->\n"); 
  fprintf(stderr, "-> Specified feature list (arg 4) holds %d features, but we are tracking %d features.\n", 
  features.num_features, d_numFeatures);
  return -1;
 }

 // check if initialize() was called
 if( d_featureList[0] == NULL ) {
  fprintf(stderr, "%s\n", "[FeatureTrackerOCV::processImage] ERROR. Must call initialize first.");
  return -1;
 }
 
 // allocate buffers if not already
 if( !d_image) {
  d_image = cvCreateImage(cvSize(w,h), 8, 1);
  d_image->origin = 0;
  d_prevImage = cvCreateImage(cvSize(w,h), 8, 1);
  d_pyramid = cvCreateImage(cvSize(w,h), 8, 1);
  d_prevPyramid = cvCreateImage(cvSize(w,h), 8, 1);
  d_trackerFlags = 0;
 }

 memcpy(d_image->imageData, buf, w * h);
 
 // first frame - select features
 if(d_frameNumber == 0) {
  if(d_autoSelect) { // automatic initialization
   IplImage* eig = cvCreateImage( cvSize(w,h), 32, 1 );
   IplImage* temp = cvCreateImage( cvSize(w,h), 32, 1 );
   d_numDetectedFeatures = d_numFeatures;
   cvGoodFeaturesToTrack( d_image, eig, temp, d_featureList[1], &d_numDetectedFeatures,
                          d_trackingContext.quality, d_trackingContext.min_dist, 0, 
                          d_trackingContext.block_size, 0, 0.04 );
   cvReleaseImage( &eig );
   cvReleaseImage( &temp );
  } else {  // manual initialization
   // initialize display
   if( d_display.init(w, h, "FeatureTrackerOCV") != 0) return -1;
   d_screen = d_display.getScreenPointer();
   
   // img->SDL surface
   snprintf(d_message, 80, "ATTENTION: Please select %d feature points\0", d_numFeatures);
   if( d_display.updateScreenBuffer((char *)buf, w, h, 1, d_message) != 0) return -1;
   d_display.refresh();
   
   // let user select features
   d_numDetectedFeatures = 0;
   while( (d_numDetectedFeatures < d_numFeatures) && SDL_WaitEvent(&event)  ) {
    switch(event.type) {
     case SDL_MOUSEBUTTONDOWN:
      x = event.button.x;
      y = event.button.y;
      fprintf(stdout, "Selected feature %3d at (%6.2f, %6.2f)\n", d_numDetectedFeatures, x, y); 
      d_featureList[1][d_numDetectedFeatures] = cvPointTo32f(cvPoint((int)x,(int)y));
      ++d_numDetectedFeatures;
      boxColor(d_screen, (short)x-2, (short)y-2, (short)x+2, (short)y+2, 0xff0000ff);
      d_display.refresh();
     break;
     case SDL_QUIT:
      fprintf(stdout, "\n[FeatureTrackerOCV::processImage] I was asked to quit!\n");
      d_displayOn = false;
      return -2;
     break;
     default:
     break;
    }
   }
  }
  cvFindCornerSubPix( d_image, d_featureList[1], d_numDetectedFeatures, 
                      cvSize(d_trackingContext.window_size, d_trackingContext.window_size), 
                      cvSize(-1,-1),
                      cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,
                      d_trackingContext.max_iter, d_trackingContext.epsilon));
  for(int i = 0; i < d_numDetectedFeatures; ++i) {
   d_trackedFeaturesIndices[i] = i;
   d_trackingErrors[i] = 0;
  }
 } else if(d_numDetectedFeatures){
  cvCalcOpticalFlowPyrLK( d_prevImage, d_image, d_prevPyramid, d_pyramid,
                 d_featureList[0], d_featureList[1], d_numDetectedFeatures, 
                 cvSize(d_trackingContext.window_size, d_trackingContext.window_size), 
                 3, d_trackStatus, d_trackingErrors,
                 cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 
                 d_trackingContext.max_iter, d_trackingContext.epsilon), d_trackerFlags );
  d_trackerFlags |= CV_LKFLOW_PYR_A_READY;
 }
 
 // prepare screen display 
 if(d_displayOn) {
  if(d_screen == NULL) if( d_display.init(w, h, "FeatureTrackerOCV") != 0) return -1;
  if( d_display.updateScreenBuffer((char *)buf, w, h, 1, NULL) != 0) return -1;
  d_screen = d_display.getScreenPointer();
 }

 // copy features to external list, update internal feature list
 int i, j = 0, k = 0, l = 0;
 for(i = 0; i < d_numDetectedFeatures; ++i) {
  if( (d_frameNumber == 0) || ((d_trackStatus[i] == 1) && (fabs(d_trackingErrors[i]) < d_trackingContext.max_error)) ) {

   d_featureList[1][k] = d_featureList[1][i];
   d_trackedFeaturesIndices[k] = d_trackedFeaturesIndices[i];

   if(d_displayOn) 
    boxColor(d_screen, (short)(d_featureList[1][k].x)-2, (short)(d_featureList[1][k].y)-2, 
             (short)(d_featureList[1][k].x)+2, (short)(d_featureList[1][k].y)+2, 0xff0000ff);
  
   for(j = l; j < d_trackedFeaturesIndices[k]; ++j) {
    features.features[j].x = 0;
    features.features[j].y = 0;
    features.features[j].val = -1;
   }
  
   l = d_trackedFeaturesIndices[k] + 1;
   features.features[d_trackedFeaturesIndices[k]].x = d_featureList[1][k].x;
   features.features[d_trackedFeaturesIndices[k]].y = d_featureList[1][k].y;
   features.features[d_trackedFeaturesIndices[k]].val = 0;
  
   ++k;
  }
 }
 d_numDetectedFeatures = k;
 for (j = l; j < d_numFeatures; ++j) {
  features.features[j].x = 0;
  features.features[j].y = 0;
  features.features[j].val = -1;
 }
 if(d_displayOn) {
  snprintf(d_message, 80, "Features tracked: %d/%d\0", d_numDetectedFeatures, d_numFeatures);
  stringColor(d_screen, 2, h-10, d_message, 0xfd1b04FF);
  d_display.refresh();
 }

 CV_SWAP( d_prevImage, d_image, d_swapImg );
 CV_SWAP( d_prevPyramid, d_pyramid, d_swapImg );
 CV_SWAP( d_featureList[0], d_featureList[1], d_swapArray );
 
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

