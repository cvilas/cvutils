//==============================================================================
// FeatureTrackerKLT.hpp - Feature detection and tracking using KLT library.
// Vilas Chitrakaran, May 2006
//==============================================================================

#ifndef INCLUDED_FEATURETRACKERKLT_HPP
#define INCLUDED_FEATURETRACKERKLT_HPP

#include "klt/klt.h"
#include <stdio.h>
#include <malloc.h>
#include "TrackerUtils.hpp"


//==============================================================================
// class FeatureTrackerKLT
//------------------------------------------------------------------------------
// \brief
// Automatic image feature detection and tracking using the Lucas-Kanade 
// tracking algorithm implemented in the KLT library.
//
// This class uses the implementation of the KLT algorithm developed and 
// maintained by Stan Birchfield. See: http://www.ces.clemson.edu/~stb/klt .
// Image display and event handling routines use the SDL library . See:
// http://www.libsdl.org. 
//
// <b>Example Program:</b>
// \include FeatureTrackerKLT.t.cpp
//==============================================================================

class FeatureTrackerKLT
{
 public:
  FeatureTrackerKLT();
   // Default constructor.
   
  ~FeatureTrackerKLT();
   // Destructor frees any allocated resources
   
  int initialize(FeatureTrackerContext_t &ftc, KLT_TrackingContext kltc);
   // Initialize the tracker. Call this method before calling 
   // any other methods of this class.
   //  ftc     settings specific to this class.
   //  kltc    KLT algorithm specific settings.
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
  
  int writeFeatureTable(const char *fileBaseName);
   // Write the history of all tracked features into a feature table in ascii (.txt) 
   // and binary format (.ft). (See KLT library documentation for details on 
   // reading from feature table). The number of records in this feature table 
   // will be less than or equal to the 'num_frames' parameter in the  
   // FeatureTrackerContext_t passed to initialize(). 
   //  fileBaseName  Base name of the file.
   //  return    0 on success, -1 on error (error message redirected to stderr).
   
 protected:
 private:
  SDLWindow d_display;
  SDL_Surface *d_screen;
  char d_message[80];
  KLT_TrackingContext d_kltc;
  KLT_FeatureList d_featureList;
  KLT_FeatureTable d_featureTable;
  int d_numFeatures;
  int d_numFrames;
  int d_frameNumber;
  bool d_autoSelect;
  bool d_displayOn;
};


#endif // INCLUDED_FEATURETRACKERKLT_HPP
