//======================================================================== 
// Project: SFM
// ---------------------------------------------------------------------- 
// Package: Image capture
// Authors: Vilas Kumar Chitrakaran
// Start Date: Oct 06 2005
// ----------------------------------------------------------------------  
// File: pxcTrigCapture.cpp
// Triggered image capture using PXC200F framegrabber
//========================================================================  

#include "pxc200/pxc.h"
#include "pxc200/frame.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>

//========================================================================  
// 1. Allocate memory for max possible number of grayscale images 
//    specified as cmd line arg.
// 2. Configure FG for double buffering. 
// 3. Trigger image capture (into memory) on rising edge at digital input 0.
// 4. Continue capture loop until specified no. of images, or cntrl-c.
// 5. Separate function to write all images to disk in serial order. Call
//    this function on exit, and in signal handler.
// 6. command line parameters: a) num images, b) file path and base name
//    for images.
//========================================================================  

#define PIXEL_FORMAT PBITS_Y8

static char *imgBuffer = NULL;
static int numImagesStored = 0;
static char outFileBaseName[80] ="./image_";
static int imgWidth = 0;
static int imgHeight = 0;
static FRAME __PX_FAR *frh = 0;
static int fgh = 0;
static PXC200 pxc;
static FRAMELIB frame;


//========================================================================  
// usage msg
//========================================================================  
void usage(char *prog)
{
  fprintf(stderr, "usage: %s -n <num_images> -f <file base name> \n -t <trigger channel (-1 for no trigger)> \n", prog);
}


//========================================================================  
// writePgm
//========================================================================  
int writePgm(const char *fname, const char *buf, int w, int h)
{
 FILE *fp;

 if(buf == NULL) {
  fprintf(stderr, "invalid image buffer\n");
  return -1;
 }
 
 if( (fp = fopen(fname, "w+")) == NULL) {
  fprintf(stderr, "ERROR opening file\n");
  return -1;
 }
 
 fprintf(fp, "%s %d %d %d\n", "P5", w, h, 0xFF);
 int n = fwrite(buf, sizeof(char), w * h, fp);
 if( n != w * h) {
  fprintf(stderr, "ERROR writing image to file %s\n", fname);
  fclose(fp);
  return -1;
 }
 fclose(fp);
 return 0;
}


//========================================================================  
// storeImages
//========================================================================  
void storeImages(int signo)
{
 if(signo == SIGINT) {
  signal(SIGINT, SIG_DFL);
  fprintf(stdout, "\nSignal SIGINT received\n");
 }
 
 if(!imgBuffer || numImagesStored == 0) { // nothing to store
  
  // free resources
  if(fgh) pxc.Reset(fgh);
  if(fgh) pxc.FreeFG(fgh);
  if(frh) frame.FreeFrame(frh);
  if(imgBuffer) free(imgBuffer);
 
  PXC200_CloseLibrary(&pxc);
  FRAMELIB_CloseLibrary(&frame);
  exit(EXIT_SUCCESS);
 }
 
 // store images to disk
 char fileNum[10];
 char fileName[95];
 int i = 0;
 int offset = imgWidth * imgHeight * sizeof(char);
 for(i = 0; i < numImagesStored; i++) {
  itoa(i, fileNum, 10);
  strncpy(fileName, outFileBaseName, 80);
  strncat(fileName, fileNum, 10);
  strncat(fileName, ".pgm", 4);
  fprintf(stdout, "\rStoring image         : %s", fileName);
  writePgm(fileName, imgBuffer + (offset * i), imgWidth, imgHeight);
 }
 fprintf(stdout, "\nStored %d images\n", i);

 // free resources
 if(fgh) pxc.Reset(fgh);
 if(fgh) pxc.FreeFG(fgh);
 if(frh) frame.FreeFrame(frh);
 if(imgBuffer) free(imgBuffer);
 
 PXC200_CloseLibrary(&pxc);
 FRAMELIB_CloseLibrary(&frame);

 exit(EXIT_SUCCESS);
}


//========================================================================  
// main function
//========================================================================  
int main(int argc, char *argv[])
{
 if(geteuid() != 0){
  fprintf(stderr, "[%s]: ERROR. Program must be executed as root.\n",argv[0]);
  return(-1);
 }
 int i, nImages = 0;
 int camera = 0;
 int triggerChannel = -1;
 
 // parse command line args
 while( (i = getopt(argc, argv, "f:n:t:")) != -1)
 {
  switch(i)
  {
   case 'f': // set output file name
    strncpy(outFileBaseName, optarg, 80);
    break;
   case 'n': // set input scene file name
    nImages = atoi(optarg);
    break;
   case 't':
    triggerChannel = atoi(optarg);
    break;
   default:
    usage(argv[0]);
    return(-1);
   }
 }
 if(optind != 7) {
  usage(argv[0]);
  return(-1);
 }

 // user passed info
 fprintf(stdout, "Max. number of images : %d\n", nImages);
 fprintf(stdout, "File base name        : %s\n", outFileBaseName);
 
 // open libraries
 if( !PXC200_OpenLibrary(&pxc, sizeof(pxc)) ) {
  fprintf(stderr, "Unable to open pxc200 library\n");
  return(-1);
 }
 if( !FRAMELIB_OpenLibrary(&frame, sizeof(frame)) ) {
  fprintf(stderr, "Unable to open frame library\n");
  PXC200_CloseLibrary(&pxc);
  return(-1);
 }

 // setup signal handler
 signal(SIGINT, storeImages);

 // Allocate framegrabber
 if( (fgh = pxc.AllocateFG(-1)) == 0 ) {
  fprintf(stderr, "Unable to find frame grabber\n");
  PXC200_CloseLibrary(&pxc);
  FRAMELIB_CloseLibrary(&frame);
  return(-1);
 }
 
 pxc.Reset(fgh);
 
 // Setup video capture parameters
 if( pxc.SetCamera(fgh, camera, 0) == 0) {        
  fprintf(stderr, "Unable to set camera\n");
  PXC200_CloseLibrary(&pxc);
  FRAMELIB_CloseLibrary(&frame);
  return(-1);
 }
 fprintf(stdout, "Video capture source  : %d\n", pxc.GetCamera(fgh));
 
 if( pxc.SetPixelFormat(fgh, PIXEL_FORMAT) == 0 ) {
  fprintf(stderr, "Unable to set pixel format\n");
  PXC200_CloseLibrary(&pxc);
  FRAMELIB_CloseLibrary(&frame);
  return(-1);
 }
 
 if( pxc.SetVideoDetect(fgh, NTSC_FORMAT) == 0 ) {
  fprintf(stderr, "Unable to set video format\n");
  PXC200_CloseLibrary(&pxc);
  FRAMELIB_CloseLibrary(&frame);
  return(-1);
 }
 imgWidth = pxc.GetWidth(fgh);
 imgHeight = pxc.GetHeight(fgh);
 fprintf(stdout, "Image size            : %d x %d\n", imgWidth, imgHeight);
 
 // Allocate frames
 if( (frh = frame.AllocateMemoryFrame(imgWidth, imgHeight, PIXEL_FORMAT)) == 0) {
  fprintf(stderr, "Unable to allocate frame buffer\n");
  PXC200_CloseLibrary(&pxc);
  FRAMELIB_CloseLibrary(&frame);
  return(-1);
 }

 // set latch triggered input
 if(triggerChannel >= 0) {
  if( pxc.SetIOType(fgh, triggerChannel, LATCH_RISING) == 0) {
   fprintf(stderr, "Unable to configure input trigger type\n");
   PXC200_CloseLibrary(&pxc);
   FRAMELIB_CloseLibrary(&frame);
   return(-1);
  }
  fprintf(stdout, "Trigger channel       : %d\n", triggerChannel);
 } else
  fprintf(stdout, "Trigger channel       : none\n");
 
 
 // Allocate buffer to store images
 int imgBytes = sizeof(char) * imgHeight * imgWidth;
 imgBuffer = (char *)malloc(imgBytes * nImages);
 if(imgBuffer == NULL) {
  fprintf(stderr, "Unable to allocate memory for images\n");
  frame.FreeFrame(frh);
  PXC200_CloseLibrary(&pxc);
  FRAMELIB_CloseLibrary(&frame);
  return(-1);
 } 
 
 numImagesStored = 0;

 
 //--- triggered grab loop ---
 int gh;
 int mask;
 char *fbAddr;
 fbAddr = (char *)frame.FrameBuffer(frh);
 while(numImagesStored < nImages) {

  // wait for trig, then grab 
  if(triggerChannel >= 0 )	{
   mask  = 1 << triggerChannel;
   pxc.WaitAnyEvent(fgh, fgh, mask, 1, QUEUED);
  }
  
  if( (gh = pxc.Grab(fgh, frh, QUEUED)) == 0) {
   fprintf(stderr, "Unable to grab frame\n");
   frame.FreeFrame(frh);
   free(imgBuffer);
   PXC200_CloseLibrary(&pxc);
   FRAMELIB_CloseLibrary(&frame);
   return(-1);
  }
  
  pxc.WaitFinished(fgh, gh);

  // copy frame to our buffer
  memcpy(imgBuffer + imgBytes * numImagesStored, fbAddr, imgBytes);
  numImagesStored++;
 
  fprintf(stdout, "\rGrab no.              : %d", numImagesStored);
 }
 fprintf(stdout, "\n");
 //--- end triggered grab loop ---

 storeImages(0);
 return(0);
}
