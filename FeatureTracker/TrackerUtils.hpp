//==============================================================================
// TrackerUtils.hpp - Utility classes and functions for the feature tracker
// Vilas Chitrakaran, May 2006
//==============================================================================

#ifndef INCLUDED_TRACKERUTILS_HPP
#define INCLUDED_TRACKERUTILS_HPP

#include "SDL/SDL.h"
#include "SDL/SDL_gfxPrimitives.h"
#include "klt/klt.h"

//==============================================================================
/*! \struct _FeatureTrackerContext
    \brief Parameters for feature tracking */
//==============================================================================
typedef struct _FeatureTrackerContext
{
 int num_features;             /*!< Max. number of features to track. */
 int num_frames;               /*!< Number of frames to track features over. */
 int auto_select_features;     /*!< 1: if you want the tracker to select features
                                    automoatically, else 0. */
 int display_tracked_features; /*!< 1: if you want to display tracked features,
                                    else 0. */
}FeatureTrackerContext_t;


//==============================================================================
/*! \struct _feature 
    \brief A feature point */
//==============================================================================
typedef struct _feature
{
 float x;  //!< x coordinate of feature in the image.
 float y;  //!< y coordinate of feature in the image.
 int val;  //!< FeatureTracker classes set this to 0 if feature is tracked, else -1.
}feature_t;


//==============================================================================
/*! \struct _feature_list
    \brief List of features in a single image */
//==============================================================================
typedef struct _feature_list
{
 _feature_list() : frame_number(-1), num_features(-1), features(NULL){};
 int frame_number;     //!< Image frame number correspoding to the list.
 int num_features;     //!< Maximum number of features the list can hold.
 feature_t *features;  //!< Array of features.
}feature_list_t;


int allocateFeatureList(feature_list_t &f, int num_features);
 /*!< Allocate memory for storing features.
      \return  Size (bytes) of the entire buffer on success, -1 on error;*/

 
void freeFeatureList(feature_list_t &f);
 /*!< Free the memory allocated for storing features using allocateFeatureStruct(). */

int copyFeaturesToKLTFeatureList(feature_list_t &f, KLT_FeatureList kl);
 /*!< Copy a feature list into feature list structure used in the KLT library.
      \return  0 on success, -1 on error (error message redirected to stderr). */

//==============================================================================
// class CountFPS
//------------------------------------------------------------------------------
// \brief
// A frames-per-second counter.
//
// This code was obtained from SDL webpage and encapsulated in a class. See:
// http://www.libsdl.org/cgi/docwiki.cgi/SDL_20Average_20FPS_20Measurement
//==============================================================================
class CountFPS
{
 public:
  CountFPS(); 
   // Default constructor. 
   
  ~CountFPS();
   // Default destructor.
   
  int init(int nFrames);
   // Initialize counter. Call this method first before 
   // calling other methods.
   //  nFrames  The number of frames to average over in calculating the frame
   //           rate.
   //  return   0 on success, -1 on error.
   
  void compute();
   // Call this function every time a new frame is captured. 
   
  float report() {return d_fps;}
   // Report the current FPS calculation.
   //  return  Last computed average frame rate.
   
 protected:
 private:
  Uint32 d_numFrames;
  Uint32 *d_frameTimes;
  Uint32 d_frameTimeLast;
  Uint32 d_frameCount;
  float d_fps;
};


//==============================================================================
// class SDLWindow
//------------------------------------------------------------------------------
// \brief
// A window for image display.
//
// The SDLWindow class uses the SDL library to display images. Use SDL 
// event handling routines to catch events such as mouse clicks.
//
// <b>Example Program:</b>
// \include SDLWindow.t.cpp
//==============================================================================
class SDLWindow
{
 public:
  SDLWindow();
   // The default constructor. Does some initializations.

  ~SDLWindow();
   // The destructor cleans up.

  int init(int w, int h, const char *title);
   // Initialize an SDL screen buffer. Window doesn't show up until refresh() is called.
   //  w, h    Window width and height.
   //  title   A title for the window. Should be set to NULL if not desired.
   //  return  0 on success, -1 on error (error message redirected to stderr).
   
  int updateScreenBuffer(char *buf, int w, int h, int bpp, const char *msg=NULL);
   // Update the screen buffer with data from user provided image buffer. Window 
   // doesn't show up on until refresh() is called.
   //  buf     A pointer to the image buffer, provided by the user.
   //  w, h    Image width and height. Provided here to reinitialize SDL screen if
   //          they are different from parameters used for init().
   //  bpp     The bytes per pixel.
   //  msg     An optional message upto 80 characters long. Useful to print helpful
   //          information on the screen.
   //  return  0 on success, -1 on error (error message redirected to stderr).
   
  void refresh();
   // Display the video on screen.
   
  SDL_Surface *getScreenPointer() {return d_screen;};
   //  return  Pointer to internal screen buffer. This is useful if you wish 
   //          to directly manipulate the screen buffer using an external 
   //          library such as SDL_gfx package.

 protected:
 private:
  void printInfo();
  CountFPS d_fps;
  char d_message[20];
  SDL_Surface *d_screen;
  SDL_Color d_8bppPalette[256];
};

#endif // #ifndef INCLUDED_TRACKERUTILS_HPP
