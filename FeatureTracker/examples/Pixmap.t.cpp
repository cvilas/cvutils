//==============================================================================
// Pixmap.t.cpp : Example program for Pixmap class.
// Author         : Vilas Kumar Chitrakaran
//==============================================================================

#include "Pixmap.hpp"

//==============================================================================
// This example demonstrates how to read an image, modify it and
// write it back as a file.
//==============================================================================
using namespace std;

int main()
{
 PixmapRgb img;
 
 // open an image
 if( img.loadPixmap("images/ash_P6.ppm") != 0 ) {
  fprintf(stderr, "OOPS\n");
  return -1;
 }

 // print image dimensions.
 fprintf(stdout, "Opened image of size: %d x %d\n", 
         img.getWidth(), img.getHeight() );

 // modify a pixel
 img(3,4) = rgb_t(255,0,0);
 
 // save to file 
 if( img.savePixmap("new_image.ppm") != 0 ) {
  fprintf(stderr, "OOPS\n");
  return -1;
 }
 
 return 0;
}
