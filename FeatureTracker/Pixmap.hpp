//==============================================================================
// Pixmap.hpp - PPM image handling routines
// Vilas Chitrakaran, May 2006
//==============================================================================

#ifndef _PIXMAP_HPP_INCLUDED
#define _PIXMAP_HPP_INCLUDED

#include <math.h>
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <typeinfo>
#include <inttypes.h>

//==============================================================================
/*! \struct _rgb
    \brief RGB pixel data type, 8 bits per channel (24 bpp). Use with class
     Pixmap.
    
    For grayscale, the pixel data type is uint8_t. */
//==============================================================================
typedef struct _rgb
{
 _rgb(uint8_t R = 0, uint8_t G = 0, uint8_t B = 0) : r(R), g(G), b(B) {};
 uint8_t r; //!< red component
 uint8_t g; //!< green component
 uint8_t b; //!< blue component
}rgb_t;


//==============================================================================
// Operators for rgb_t
//==============================================================================
bool operator==(const rgb_t &c1, const rgb_t &c2);
bool operator!=(const rgb_t &c1, const rgb_t &c2);
std::ostream &operator<< (std::ostream &out, const rgb_t &rgb);
std::istream &operator>> (std::istream &in, rgb_t &rgb);


//==============================================================================
// class Pixmap
//------------------------------------------------------------------------------
// \brief
// The template class for pixmap (ppm, pgm) images. 
//
// The class provides methods to read and write images only as pixmaps (PPM). 
// However, methods to directly access the image buffer is provided, hence the
// user can develop her own additional functions to support other image 
// formats. The image buffer created by this class stores images either as 
// 8 bit grayscale (T = uint8_t), or 24 bit RGB (T = rgb_t) in packed pixel format 
// (ie, all the data for a pixel lie next to each other in memory.
//
// <b>Example Program:</b>
// \include Pixmap.t.cpp
//==============================================================================
template <class T> // supported types T = uint8_t, rgb_t
class Pixmap
{
 public:
  Pixmap();
   // Default constructor. Does nothing
   
  Pixmap(int w, int h);
   // Constructor that allocates memory buffer.
   //  w     image width (pixels)
   //  h     image height (pixels)
   
  Pixmap(uint8_t *buffer, int w, int h);
   // Constructor that hooks to an externally allocated
   // memory buffer instead of allocating memory of it's own.
   // It is user's responsbility to ensure that buffer size 
   // is adequate for an image of size w x h of specified type. 
   //  buffer  Pointer to image data
   //  w       image width (pixels)
   //  h       image height (pixels)
  
  virtual ~Pixmap();
   // The destructor. Frees any allocated memory.
   
  int create(int w, int h);
   // Allocates a new data buffer for image data, or resizes a previously
   // allocated buffer. The buffer values are not initialized, and 
   // may be anything arbitrary.
   //  w       width (pixels).
   //  h       height (pixels).
   //  return  0 on success, -1 if failed.
   
  int attach(uint8_t *buffer, int w, int h);
   // Hook to an externally provided buffer for image data (such as framebuffer 
   // of a frame grabber). It is user's responsbility to ensure that 
   // buffer size is adequate for an image of size w x h of specified type.
   //  buffer  Pointer to image data
   //  w       width (pixels).
   //  h       height (pixels).
   //  return  0 on success, -1 if failed.

  inline int getWidth() const;
   //  return  width of image in pixels.
   
  inline int getHeight() const;
   //  return  height of image in pixels.
  
  inline int getBytesPerPixel() const;
   //  return  Bytes per pixel.

  inline bool isIndexValid(int i);
   //  i       1D index into data buffer (starts at 0).
   //  return  true if index within image boundaries, 
   //          else false.

  inline bool isIndexValid(int c, int r);
   //  c,r     column, row index into data buffer ( starts at (0,0) ).
   //  return  true if index within image boundaries, 
   //          else false.

  inline T* getPointer(int i);
   //  i       1D index into data buffer (starts at 0).
   //  return  Pointer to pixel data at specified index, 
   //          NULL if index is out of range.
   
  inline T* getPointer(int c, int r);
   //  c,r     column, row index into data buffer ( starts at (0,0) ).
   //  return  Pointer to pixel data at specified index, 
   //          NULL if index is out of range.

  inline T &operator()(int i);
   // Access image data at a specified location. For example:
   // \code myImage(2)=255; \endcode or
   // \code cout << myImage(2) << endl; \endcode
   //  i       1D index into data buffer (starts at 0).
   //  return  Pointer to pixel data at specified index 
   
  inline T &operator()(int c, int r);
   // Access image data at a specified location. For example:
   // \code myImage(2,2)=255; \endcode or
   // \code cout << myImage(2,2) << endl; \endcode
   //  c,r     column, row index into data buffer ( starts at (0,0) ).
   //  return  Pointer to pixel data at specified index 

  Pixmap<T> &operator=(const Pixmap<T> &p);
   // Assignment between two images of same type and dimensions. 
   //  p  The Pixmap object.
  
  int loadPixmap(const char *fileName);
   // Load a pixmap image (pgm, ppm). 
   //  fileName  The name of the image file
   //  return    0 on success, -1 on error.
   
  int savePixmap(char *fileName); 
   // Save the image as a pixmap (ppm)
   //  fileName  The name of the image file
   //  return    0 on success, -1 on error.

 protected:
  bool d_usingExternalBuffer;
  T *d_imgData;
  int d_w;
  int d_h;

 private:
  Pixmap(Pixmap &p) {return;};
   // prevents initialization by copying.
};


//==============================================================================
// class PixmapGray
//------------------------------------------------------------------------------
// \brief
// Class for 1 byte-per-pixel greyscale (pgm) images. 
//==============================================================================
class PixmapGray : public Pixmap<uint8_t>{};  

//==============================================================================
// class PixmapRgb
//------------------------------------------------------------------------------
// \brief
// Class for 3 bytes-per-pixel RGB (ppm) images. 
//==============================================================================
class PixmapRgb : public Pixmap<rgb_t>{};

// ========== END OF INTERFACE ==========

//==============================================================================
// Pixmap::Pixmap
//==============================================================================
template <class T>
Pixmap<T>::Pixmap()
{ 
 d_usingExternalBuffer = false;
 d_w = 0;
 d_h = 0;
 d_imgData = NULL;
}

template <class T>
Pixmap<T>::Pixmap(int w, int h)
{
 create(w,h);
}

template <class T>
Pixmap<T>::Pixmap(uint8_t *buffer, int w, int h)
{
 attach(buffer,w,h);
}


//==============================================================================
// Pixmap::~Pixmap
//==============================================================================
template <class T>
Pixmap<T>::~Pixmap()
{
 if(!d_usingExternalBuffer && d_imgData)
  free(d_imgData);
}


//==============================================================================
// Pixmap::create
//==============================================================================
template <class T>
int Pixmap<T>::create(int w, int h)
{
 d_usingExternalBuffer = false;
 if (w <= 0 || h <= 0 ) {
  fprintf(stderr, "[Pixmap::create]: Invalid Params (%d, %d)\n", w, h);
  return -1;
 }
 d_imgData = (T *)realloc(d_imgData, sizeof(T) * w * h);
 if( d_imgData == NULL) {
  fprintf(stderr, "[Pixmap::create]: Error allocating image buffer.\n");
  return -1;
 }
 d_w = w;
 d_h = h;
 return 0;
}


//==============================================================================
// Pixmap::attach
//==============================================================================
template <class T>
int Pixmap<T>::attach(uint8_t *buffer, int w, int h)
{
 d_usingExternalBuffer = true;
 d_w = w;
 d_h = h;
 d_imgData = (T *)buffer;
 return 0;
}


//==============================================================================
// Pixmap::operator=
//==============================================================================
template <class T>
Pixmap<T> &Pixmap<T>::operator=(const Pixmap<T> &p)
{
 if(this == &p)
  return *this;
 
 if(p.d_h == d_h && p.d_w == d_w) {
  memcpy(d_imgData, p.d_imgData, d_h * d_w * sizeof(T));
 } else {
  fprintf(stderr, "[Pixmap::operator=]: Image dims. mismatch. Data not copied.\n");
 }
 return *this;
}


//==============================================================================
// Pixmap::loadPixmap
//==============================================================================
template <class T>
int Pixmap<T>::loadPixmap(const char *fileName)
{
 FILE *source;
 char header[80];
 int maxPixVal = -1;
 int w = -1;
 int h = -1;
 int bpp = 0;
 int status = 0;
 char fileType;
 
	
 // Open image file
 if( (source = fopen(fileName, "rb")) == NULL ) {
  fprintf(stderr, "[Pixmap::loadPixmap]: Could not open %s.\n", fileName);
  return -1;
 }
	
 // Read the header
 header[0] = fgetc(source);
 header[1] = fgetc(source);		
 header[2] = fgetc(source);				
 header[2] = '\0';
 if(header[0] == EOF) {
  fprintf(stderr, "[Pixmap::loadPixmap]: Could not read %s.\n", fileName);
  return -1;
 }
 
 fileType = header[1]; // indicates whether type is P2, P3, P5 or P6
 if( fileType == '3' || fileType == '6')
  bpp = 3;
 else if (fileType == '2' || fileType == '5')
  bpp = 1;
 
 // could not read image type
 if(!bpp) {
  fprintf(stderr, "[Pixmap::loadPixmap]: Unknown/unsupported image format.\n");
  return -1;
 }
 
 while( maxPixVal == -1 ) {
  fscanf( source, "%s", header);
  if ( header[0] == '#' ) {
   while( (header[0] = fgetc(source)) != '\n' );
    continue;
  }
  if ( w == -1 ) w = atoi ( header );
  else if ( h == -1 ) h = atoi ( header);
  else maxPixVal = atoi ( header );
 }
 header[0] = fgetc(source); //reading whitespace right after header

 // Allocate memory for the image data
 d_w = w;
 d_h = h;
 if( create(d_w, d_h) == -1)
  return -1;
  
 // Read the pixel values into buffer
 int pixVal[3];
 uint8_t *buffer = (uint8_t *)d_imgData;
 for(int i = 0; i < d_w * d_h; i++) {
  pixVal[0] = 0; pixVal[1] = 0; pixVal[2] = 0;
  if( bpp == 1 ) { // 8bpp
   if(fileType == '2')
    status = fscanf(source, "%d", &pixVal[0]);
   else
    status = fscanf(source, "%c", (unsigned char *)&pixVal[0]);
   if(status != 1) {
    fprintf(stderr, "[Pixmap::loadPixmap]: Could not read %s.\n", fileName);
    return -1;
   }
   if(typeid(T) == typeid(uint8_t))
    buffer[i] = (uint8_t)pixVal[0];
   if(typeid(T) == typeid(rgb_t)) {
    buffer[3*i]   = (uint8_t)pixVal[0];
    buffer[3*i+1] = (uint8_t)pixVal[0];
    buffer[3*i+2] = (uint8_t)pixVal[0];
   }
  }
  if(bpp == 3) { // 24bpp
   if( fileType == '3' )
    status = fscanf( source, "%d %d %d", &pixVal[0], &pixVal[1], &pixVal[2] );
   if( fileType == '6' )
    status = fscanf( source, "%c%c%c", (unsigned char *)(&pixVal[0]), 
                   (unsigned char *)(&pixVal[1]), (unsigned char *)(&pixVal[2]) );
   if(status != 3) {
    fprintf(stderr, "[Pixmap::loadPixmap]: Could not read %s.\n", fileName);
    return -1;
   }
   if(typeid(T) == typeid(uint8_t)) {
    buffer[i] = (uint8_t)(0xFF & ((pixVal[0] + pixVal[1] + pixVal[2])/3));
   }
   if(typeid(T) == typeid(rgb_t)) {
    buffer[3*i]   = (uint8_t)pixVal[0];
    buffer[3*i+1] = (uint8_t)pixVal[1];
    buffer[3*i+2] = (uint8_t)pixVal[2];
   }
  }
 }
 fclose(source);
 return 0;
}


//==============================================================================
// Pixmap::savePixmap
//==============================================================================
template <class T>
int Pixmap<T>::savePixmap(char *fileName)
{
 FILE *destination;
 char *header = 0;
 int retVal = -1;
 unsigned int size = 0;
	
 if(d_imgData == NULL) {
  fprintf(stderr, "[Pixmap::savePixmap]: Image buffer is empty.\n");
  return retVal;
 }
 
 destination = fopen(fileName, "w+");
 if ( destination == NULL) {
  fprintf(stderr, "[Pixmap::savePixmap]: Could not open %s.\n", fileName);
  return retVal;
 }
	
 // write header
 if(typeid(T) == typeid(uint8_t)) {
  header = "P5";
  size = d_h * d_w;
 }
 if(typeid(T) == typeid(rgb_t)) {
  header = "P6";
  size = d_h * d_w * 3;
 }
 fprintf(destination, "%s %d %d %d\n", header, d_w, d_h, 0xFF);
 if( size == fwrite(d_imgData, sizeof(uint8_t), size, destination) )
  retVal = 0;
 fclose(destination);

 return retVal;
}


//==============================================================================
// Pixmap::getWidth
//==============================================================================
template <class T>
int Pixmap<T>::getWidth() const
{
 return d_w;
}


//==============================================================================
// Pixmap::getHeight
//==============================================================================
template <class T>
int Pixmap<T>::getHeight() const
{
 return d_h;
}


//==============================================================================
// Pixmap::getBytesPerPixel
//==============================================================================
template <class T>
int Pixmap<T>::getBytesPerPixel() const
{
 if(typeid(T) == typeid(uint8_t)) 
  return 1;
 if(typeid(T) == typeid(rgb_t)) 
  return 3;
 return -1;
}

//==============================================================================
// Pixmap::isIndexValid
//==============================================================================
template <class T>
bool Pixmap<T>::isIndexValid(int i)
{
 if( (i < 0) || (i > d_w * d_h - 1) )
  return false;
 return true;
}

template <class T>
bool Pixmap<T>::isIndexValid(int c, int r)
{
 return (isIndexValid(r * d_w + c));
}


//==============================================================================
// Pixmap::getPointer
//==============================================================================
template<class T>
T *Pixmap<T>::getPointer(int i)
{
 if( !isIndexValid(i) ) {
  fprintf(stderr, "[Pixmap::getPointer]: Index %d is invalid.\n", i);
  return NULL;
 }
 return ( &(d_imgData[i]) );
}

template<class T>
T *Pixmap<T>::getPointer(int c, int r)
{
 return ( getPointer(r * d_w + c) );
}


//==============================================================================
// Pixmap::operator()
//==============================================================================
template <class T>
T &Pixmap<T>::operator()(int i)
{
 if( !isIndexValid(i) ) {
  fprintf(stderr, "[Pixmap::operator()]: Index %d is invalid.\n", i);
  return d_imgData[0];
 }
 return d_imgData[i];
}

template <class T>
T &Pixmap<T>::operator()(int c, int r)
{
 return ( (*this)(r * d_w + c) );
}

#endif //_PIXMAP_HPP_INCLUDED
