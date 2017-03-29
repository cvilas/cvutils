//==============================================================================
// TrackerUtils.cpp - Utility classes and functions for the feature tracker
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "TrackerUtils.hpp"
#include <stdlib.h>

//==============================================================================
int allocateFeatureList(feature_list_t &f, int num_features)
//==============================================================================
{
 int bytes = num_features * sizeof(feature_t);
 f.features = (feature_t *)realloc(f.features, bytes);
 if(f.features == NULL) {
  fprintf(stderr, "[allocateFeatureStruct] ERROR allocating memory.\n");
  return -1;
 }
 f.num_features = num_features;
 return( bytes + 2 * sizeof(int) );
}

 
//==============================================================================
void freeFeatureList(feature_list_t &f)
//==============================================================================
{
 if(f.features) free(f.features);
}


//==============================================================================
int copyFeaturesToKLTFeatureList(feature_list_t &f, KLT_FeatureList kl)
//==============================================================================
{
 if(f.num_features != kl->nFeatures) {
  fprintf(stderr, "[copyFeaturesToKLTFeatureList]: argument sizes don't match.\n");
  return -1;
 }
 for(int i = 0; i < kl->nFeatures; ++i) {
  kl->feature[i]->x = f.features[i].x;
  kl->feature[i]->y = f.features[i].y;
  if( f.features[i].val == 0)
   kl->feature[i]->val = KLT_TRACKED;
  else
   kl->feature[i]->val = KLT_NOT_FOUND;
 }
 return 0;
}


//==============================================================================
CountFPS::CountFPS()
//==============================================================================
{ 
 d_numFrames = 0;
 d_frameTimes = NULL;
 if( SDL_Init(SDL_INIT_TIMER) < 0 ) {
  fprintf(stderr, "[CountFPS::CountFPS] ERROR SDL timer init failed.\n");
 }
}


//==============================================================================
CountFPS::~CountFPS()
//==============================================================================
{ 
 if(d_frameTimes)
  free(d_frameTimes);
 if( SDL_WasInit(SDL_INIT_TIMER) ) SDL_QuitSubSystem(SDL_INIT_TIMER);
}


//==============================================================================
int CountFPS::init(int n) 
//==============================================================================
{
 d_numFrames = n;
 d_frameTimes = (Uint32 *)realloc(d_frameTimes, sizeof(Uint32) * n);
 if(d_frameTimes == NULL) {
  fprintf(stderr, "[ComputeFPS::init] ERROR allocating memory.\n");
  return -1;
 }
 d_frameCount = 0;
 d_fps = 0;
 d_frameTimeLast = SDL_GetTicks();
 return 0;
}


//==============================================================================
void CountFPS::compute() 
//==============================================================================
{
 if(d_frameTimes == NULL) {
  fprintf(stderr, "[ComputeFPS::compute] ERROR. Did you call init yet?\n");
  return;
 }
 
 Uint32 frameTimesIndex;
 Uint32 getTicks;
 Uint32 count;
 Uint32 i;

 frameTimesIndex = d_frameCount % d_numFrames;
 getTicks = SDL_GetTicks();
 d_frameTimes[frameTimesIndex] = getTicks - d_frameTimeLast;
 d_frameTimeLast = getTicks;
 d_frameCount++;
 if (d_frameCount < d_numFrames) {
  count = d_frameCount;
 } else {
  count = d_numFrames;
 }
 d_fps = 0;
 for (i = 0; i < count; ++i) {
  d_fps += d_frameTimes[i];
 }
 d_fps /= count;

 d_fps = 1000.f / d_fps;
}


//==============================================================================
SDLWindow::SDLWindow()
//==============================================================================
{
 d_screen = NULL;
 d_message[0]='\0';

 if( SDL_Init(SDL_INIT_VIDEO|SDL_INIT_EVENTTHREAD) < 0 ) 
  fprintf(stderr, "[SDLWindow::SDLWindow] ERROR SDL video init failed.\n");

 d_fps.init(10);

 //gfxPrimitivesSetFont(const void *fontdata, int cw, int ch);

 // SDL palette for 8 bit images
 for(int i = 0; i < 256; ++i) {
  d_8bppPalette[i].r = i;
  d_8bppPalette[i].g = i;
  d_8bppPalette[i].b = i;
 }
}


//==============================================================================
SDLWindow::~SDLWindow()
//==============================================================================
{
 if(d_screen) SDL_FreeSurface(d_screen);
 if( SDL_WasInit(SDL_INIT_VIDEO) ) SDL_QuitSubSystem(SDL_INIT_VIDEO);
}


//==============================================================================
int SDLWindow::init(int w, int h, const char *title)
//==============================================================================
{

 if(d_screen) SDL_FreeSurface(d_screen);
 int flags = SDL_HWSURFACE | SDL_ANYFORMAT | SDL_ASYNCBLIT | SDL_DOUBLEBUF;
 d_screen = SDL_SetVideoMode(w, h, 0, flags);
 if(d_screen == NULL) {
  fprintf(stderr, "[SDLWindow::init] ERROR setting %dx%dx8 mode: %s\n",
          w, h, SDL_GetError());
  return -1;
 }
 printInfo();
 if(title) SDL_WM_SetCaption(title, NULL);
 return 0;
}


//==============================================================================
int SDLWindow::updateScreenBuffer(char *buf, int w, int h, int bpp, const char *msg)
//==============================================================================
{
 SDL_Surface *img = NULL;

 // image buffer not initialized
 if(buf == NULL || w == 0 || h == 0) {
  fprintf(stderr, "[SDLWindow::show]: Image buffer is invalid.\n");
  return -1;
 }

 // initialize screen if necessary
 if(d_screen == NULL) if( init(w, h, NULL) != 0) return -1;
 if( w != d_screen->w || h != d_screen->h ) if( init(w, h, NULL) != 0) return -1;

 int rmask = 0, gmask = 0, bmask = 0;
 if( bpp == 3) { rmask = 0x0000FF; gmask = 0x00FF00; bmask = 0xFF0000; }
 if( bpp == 1) { rmask = 0x0000FF; gmask = 0x0000FF; bmask = 0x0000FF; }

 // img->SDL surface
 img = SDL_CreateRGBSurfaceFrom(buf, w, h, 8 * bpp, w * bpp, rmask, gmask, bmask, 0x00);
 if ( img == NULL ) {
  fprintf(stderr, "[SDLWindow::show] ERROR: %s.\n", SDL_GetError());
  return(-1);
 }
 SDL_SetColors(img, d_8bppPalette, 0, 256);
 
 // blit to video surface
 if ( SDL_BlitSurface(img, NULL, d_screen, NULL) < 0 ) {
  fprintf(stderr, "[SDLWindow::show] ERROR: %s.\n", SDL_GetError());
  return(-1);
 }
 if(img) SDL_FreeSurface(img);

 if(msg) stringColor(d_screen, 2, h-10, msg, 0xfd1b04FF);

 return 0;
}


//==============================================================================
void SDLWindow::refresh() 
//==============================================================================
{
 d_fps.compute();
 snprintf(d_message, 20, "FPS: %3.2f\0", d_fps.report());
 stringColor(d_screen, 2, (d_screen->h)-20, d_message, 0xfd1b04FF);
 SDL_Flip(d_screen);
}


//==============================================================================
void SDLWindow::printInfo()
//==============================================================================
{
 if(d_screen == NULL) return;
 
 fprintf(stdout, "Video mode %dx%dx%d ", d_screen->w, d_screen->h, d_screen->format->BitsPerPixel);
 if(d_screen->flags & SDL_HWSURFACE)
  fprintf(stdout, "(video memory), ");
 else
  fprintf(stdout, "(system memory), ");
 if(d_screen->flags & SDL_ASYNCBLIT)
  fprintf(stdout, "async blit, ");
 if(d_screen->flags & SDL_DOUBLEBUF)
  fprintf(stdout, "double buffer, ");
 if(d_screen->flags & SDL_HWACCEL)
  fprintf(stdout, "hw accel.\n");
 else
  fprintf(stdout, "no hw accel.\n");
}

