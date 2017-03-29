//==============================================================================
// FeatureClientServer.hpp - UDP server/client for feature points
// Vilas Chitrakaran, May 2006
//==============================================================================

#ifndef INCLUDED_FEATURECLIENTSERVER_HPP
#define INCLUDED_FEATURECLIENTSERVER_HPP

#include "putils/UDPClientServer.hpp"
#include "putils/RWLock.hpp"
#include "putils/Thread.hpp"
#include "TrackerUtils.hpp"

//==============================================================================
/*! \struct _FeatureServerContext
    \brief Parameters for UDP feature server. Use with class FeatureServer. */
//==============================================================================
typedef struct _FeatureServerContext
{
 int port;             /*!< Server port number. */
 int thread_priority;  /*!< Priority of the server thread. */
 int num_features;     /*!< Max. number of features to serve. */
}FeatureServerContext_t;


//==============================================================================
// class FeatureServer
//------------------------------------------------------------------------------
// \brief
// A UDP network server for feature tracker.
//
// An object of this class starts a separate thread and replies to clients 
// (FeatureClient object) with the latest feature point list.
//
// <b>Example Program:</b>
// \include FeatureServer.t.cpp
// \include FeatureClient.t.cpp
//==============================================================================
class FeatureServer: public Thread, public UDPServer
{ 
 public:
  FeatureServer();
   // Default constructor. Does a few initializations.
   
  ~FeatureServer();
   // Default destructor. Frees resources
   
  int initialize(FeatureServerContext_t &cxt);
   // Initializes and starts the server thread. This must be the first method 
   // called before using any other method in this class.
   //  return  0 on success, -1 on error (error message redirected to stderr).
 
  int updateFeatures(feature_list_t &features, int srcFrameNumber);
   // Update the features buffer in the server.
   //  features        The feature list.
   //  srcFrameNumber  the image/video frame number corresponding to this 
   //                  feature list. The internal buffer is not updated 
   //                  unless this number is different from an internally
   //                  maintained counter. This avoid unecessary copy operations.
   //  return          0 on success, -1 on error (error message redirected to stderr).

 protected:
  virtual const char *receiveAndReply(const char *inMsgBuf, int inMsgLen, int *outMsgLen); 
   // Reimplemented from UDPServer class.
   
  virtual void enterThread(void *arg);
   // Reimplemented from Thread class.
   
  virtual int executeInThread(void *arg);
   // Reimplemented from Thread class.

  virtual void exitThread(void *arg);
   // Reimplemented from Thread class.

 private:
  int d_port;
  int d_priority;
  bool d_isInit;
  char *d_srcFeatures;
  char *d_dstFeatures;
  int d_msgSize;
  int d_numFeatures;
  RWLock d_rwLock;
};


//==============================================================================
// class FeatureClient
//------------------------------------------------------------------------------
// \brief
// A UDP network client for FeatureServer.
//
// <b>Example Program:</b>
// See examples for FeatureServer class.
//==============================================================================
class FeatureClient
{
 public:
  FeatureClient();
   // Default constructor. Does nothing.
   
  ~FeatureClient();
   // Default destructor. Does nothing.
   
  int initialize(const char *serverIp, int port, int msTimeOut, int nFeatures);
   // Connect to remote feature server.
   //  serverIp   The IP address of the remote server.
   //  port       The server port.
   //  msTimeOut  Connection timeout (in milliseconds).
   //  nFeatures  Number of feature points expected in the server message.
   //  return     0 on success, -1 on error (error message redirected to stderr).
   
  int receiveFeatureList(feature_list_t &features);
   //  features  feature list structure with updated features.
   //  return    0 on success, -1 on error (error message redirected to stderr).
   
 protected:
 private:
  UDPClient d_client;
  int d_inMsgLen;
  char *d_featureList;
};

#endif // INCLUDED_FEATURECLIENTSERVER_HPP
