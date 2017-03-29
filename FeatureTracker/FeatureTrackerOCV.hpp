//==============================================================================
// FeatureTrackerOCV.hpp - Feature detection and tracking using OpenCV's LK alg.
// Vilas Chitrakaran, May 2006
//==============================================================================

#ifndef INCLUDED_FEATURETRACKEROCV_HPP
#define INCLUDED_FEATURETRACKEROCV_HPP

#include "opencv/cv.h"
#include <stdio.h>
#include <malloc.h>
#include "TrackerUtils.hpp"


//==============================================================================
/*! \struct _OCVTrackingContext 
    \brief Parameters specific to OpenCV tracker (for use with FeatureTrackerOCV class)
    
    Good default values for parameters are given in parenthesis. */
//==============================================================================
typedef struct _OCVTrackingContext
{
 int min_dist;          /*!< Minimum distance between detected corners (10). */
 double quality;        /*!< Multiplier for the maxmin eigenvalue; specifies 
                             minimal accepted quality of image corners. (0.01). */
 int block_size;        /*!< Size of the averaging block, passed to underlying 
                             cvCornerMinEigenVal() (3). */
 int max_iter;          /*!< Maximum number of iterations (20). */
 double epsilon;        /*!< Desired tracking accuracy (0.03). */
 int window_size;       /*!< Size of search window (10). */
 float max_error;       /*!< Difference between patches around the original and 
                             moved points. Should be a large value for scenes
                             with substantial motion (200 for static camera). */
}OCVTrackingContext_t;


//==============================================================================
// class FeatureTrackerOCV
//------------------------------------------------------------------------------
// \brief
// Automatic image feature detection and tracking using the OpenCV library 
// implementation of the Lucas-Kanade tracking algorithm.
//
// OpenCV must be installed in order to use this class. See:
// http://www.intel.com/technology/computing/opencv/index.htm .
//
// Image display and event handling routines use the SDL library. See:
// http://www.libsdl.org . 
//
// <b>Example Program:</b>
// \include FeatureTrackerOCV.t.cpp
//==============================================================================

class FeatureTrackerOCV
{
 public:
  FeatureTrackerOCV();
   // Default constructor.
   
  ~FeatureTrackerOCV();
   // Destructor frees any allocated resources.
   
  int initialize(FeatureTrackerContext_t &ftc, OCVTrackingContext_t &ocvt);
   // Initialize the tracker. Call this method before calling 
   // any other methods of this class.
   //  ftc     settings specific to this class.
   //  ocvt    tracker algorithm specific settings. 
   //  return  0 on success, -1 on error (error message redirected to stderr).
   
  int processImage(unsigned char *img, int w, int h, feature_list_t &list);
   // Track features in the image buffer. Upon calling this method the first time, 
   // features are selected either automatically (if 'auto_select_features' was 
   // turned on during initialization) or by the user. If 'display_tracked_features' 
   // was turned on during initialization, the image display window will be 
   // updated with the current image in the buffer and the location of 
   // tracked features are marked.
   // <hr>
   // NOTE: Calling this function initiates SDL event handling, including for SIGINT
   // (CNTRL+C). Hence, to catch events outside of this method, use SDL functions
   // such as SDL_PollEvent().
   // <hr>
   //  img       Pointer to image buffer. NOTE: image must be 8 bit grayscale.
   //  w,h       Image dimensions in pixels.
   //  list      List of tracked features. This list contains updated (x,y) locations
   //            of features and an integer value indicating whether the feature was 
   //            tracked successfully (0) or not (-1). 
   //  return    current frame number on success (first frame = 1), -1 on error (error  
   //            message redirected to stderr), -2 on user initiated quit.
  
 protected:
 private:
  SDLWindow d_display;
  SDL_Surface *d_screen;
  char d_message[80];
  int d_numFeatures;
  int d_numFrames;
  CvPoint2D32f *d_featureList[2];
  CvPoint2D32f *d_swapArray;
  char *d_trackStatus;
  int *d_trackedFeaturesIndices;
  float *d_trackingErrors;
  IplImage *d_image, *d_prevImage, *d_pyramid, *d_prevPyramid, *d_swapImg;
  int d_frameNumber;
  bool d_autoSelect;
  bool d_displayOn;
  OCVTrackingContext_t d_trackingContext;
  int d_trackerFlags;
  int d_numDetectedFeatures;
};



#endif // INCLUDED_FEATURETRACKEROCV_HPP
