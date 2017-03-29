//==============================================================================
// SDLWindow.t.cpp : Example program for SDLWindow and Pixmap class.
// Author          : Vilas Kumar Chitrakaran
//==============================================================================

#include "TrackerUtils.hpp"
#include "SDL/SDL_events.h"
#include "Pixmap.hpp"
#include <iostream>

using namespace std;

static int quit = 0;
int filterSDLQuitEvent(const SDL_Event *event);
 // filter out SDL_QUIT and handle it here.
 
//==============================================================================
// This example demonstrates how to display an image, and process user mouse
// clicks.
//==============================================================================
int main(int argc, char *argv[])
{
 SDLWindow window;  // image window
 PixmapRgb image;   // image
 char *pointer;
 int w, h, bpp;
 
 // open an image
 if( image.loadPixmap("images/ash_P6.ppm") != 0 )
  return -1;

 pointer = (char *)image.getPointer(0);
 w = image.getWidth();
 h = image.getHeight();
 bpp = image.getBytesPerPixel();
  
 // Display the image on screen
 if( window.updateScreenBuffer(pointer, w, h, bpp, NULL) != 0)
  return -1;
 window.refresh();

 // handle mouse events (standard SDL event handling)
 SDL_Event event;
 SDL_SetEventFilter(filterSDLQuitEvent); // handle SDL_QUIT
 while( SDL_WaitEvent(&event) && !quit ) {
  switch(event.type) {
   int x, y;
   case SDL_MOUSEBUTTONDOWN:
    x = event.button.x;
    y = event.button.y;
    cout << image(x,y) << endl;
    image(x,y) = rgb_t(0xFF, 0, 0);
   break;
  
   default:
   break;
  }
  if( window.updateScreenBuffer(pointer, w, h, bpp, NULL) != 0)
   return -1;
  window.refresh();
 }
 return 0;
}

 
//==============================================================================
// filterSDLQuitEvent
//==============================================================================
int filterSDLQuitEvent(const SDL_Event *event)
{ 
 if( event->type == SDL_QUIT) {
  cout << "Quitting." << endl;
  quit = 1;
 }
 return(1);
}
