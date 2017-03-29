//==============================================================================
// FeatureClientServer.cpp - Implementation of FeatureServer class
// UAV follower experiment
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "FeatureClientServer.hpp"

//#define DEBUG


//==============================================================================
FeatureServer::FeatureServer() : Thread(), UDPServer()
//==============================================================================
{
 d_port = -1;
 d_priority = 0;
 d_msgSize = 0;
 d_isInit = false;
 d_srcFeatures = NULL;
 d_dstFeatures = NULL;
 d_numFeatures = 0;
}


//==============================================================================
FeatureServer::~FeatureServer()
//==============================================================================
{
 Thread::cancel();
 Thread::join();
 if(d_srcFeatures) free(d_srcFeatures);
 if(d_dstFeatures) free(d_dstFeatures);
}


//==============================================================================
int FeatureServer::initialize(FeatureServerContext_t &cxt)
//==============================================================================
{
 d_isInit = false;
 
 // init variables
 d_numFeatures = cxt.num_features;
 d_port = cxt.port;
 d_priority = cxt.thread_priority;
 d_msgSize = d_numFeatures * sizeof(feature_t) + 2 * sizeof(int);
 
 if((d_srcFeatures = (char *)realloc(d_srcFeatures, d_msgSize)) == NULL) {
  fprintf(stderr, "[FeatureServer::initialize] ERROR allocating memory.\n");
  return -1;
 }
 
 if((d_dstFeatures = (char *)realloc(d_dstFeatures, d_msgSize)) == NULL) {
  fprintf(stderr, "[FeatureServer::initialize] ERROR allocating memory.\n");
  return -1;
 }
 
 // data in buffer follows the structure of features_list_t 
 * (int *)d_srcFeatures = -1; // frame_number
 * (int *)d_dstFeatures = -1;
 * (int *)(d_srcFeatures + sizeof(int)) = d_numFeatures; 
 * (int *)(d_dstFeatures + sizeof(int)) = d_numFeatures;
 
 // initialize server
 if( UDPServer::init(d_port, sizeof(char)) == -1) {
  fprintf(stderr, "[FeatureServer::initialize] %s\n", UDPServer::getStatusMessage());
  return -1;
 }

 // start server thread
 if( Thread::run((void *)this) != 0) {
  fprintf(stderr, "[FeatureServer::initialize] ERROR starting server thread\n");
  return(-1);
 }
 
 d_isInit = true;
 
 return 0;
}


//==============================================================================
int FeatureServer::updateFeatures(feature_list_t &features, int frame)
//==============================================================================
{
#ifdef DEBUG
 fprintf(stderr, "[FeatureServer::updateFeatures]: Enter.\n");
#endif

 if(!d_isInit){
  fprintf(stderr, "[FeatureServer::updateFeatures] ERROR. Server not initialized.\n");
  return(-1);
 }

 if(features.num_features != d_numFeatures){
  fprintf(stderr, "[FeatureServer::updateFeatures] ERROR Expecting %d features, got %d.\n",
          d_numFeatures, features.num_features);
  return(-1);
 }
 
 char *buf = d_srcFeatures;
 
 d_rwLock.writeLock();
 if( *(int *)buf != frame ) {
  *(int *)buf = frame; buf += 2 * sizeof(int);
  for(int i = 0; i < d_numFeatures; ++i) {
   * (float *)buf = features.features[i].x; buf += sizeof(float);
   * (float *)buf = features.features[i].y; buf += sizeof(float);
   * (int *)buf = features.features[i].val; buf += sizeof(int);
  }
 }
 d_rwLock.unlock();
 
#ifdef DEBUG
 fprintf(stderr, "[FeatureServer::updateFeatures]: exit.\n");
#endif
 return 0;
}


//==============================================================================
const char *FeatureServer::receiveAndReply(const char *inMsgBuf, int inMsgLen, int *outMsgLen)
//==============================================================================
{
#ifdef DEBUG
 fprintf(stderr, "[FeatureServer::receiveAndReply]: Enter.\n");
#endif
 inMsgBuf = inMsgBuf;
 inMsgLen = inMsgLen;
 d_rwLock.readLock();
 if( * (int *)d_srcFeatures != * (int *)d_dstFeatures) { // check for new frame
  memcpy(d_dstFeatures, d_srcFeatures, d_msgSize);
 }
 d_rwLock.unlock();
 *outMsgLen = d_msgSize;
#ifdef DEBUG
 fprintf(stderr, "[FeatureServer::receiveAndReply]: exit.\n");
#endif
 return (const char *)(d_dstFeatures);
}


//==============================================================================
void FeatureServer::enterThread(void *arg)
//==============================================================================
{
 struct sched_param param;
 int policy;

 pthread_getschedparam(pthread_self(), &policy, &param);
 policy = SCHED_FIFO;
 param.sched_priority = ((FeatureServer *)arg)->d_priority;
 pthread_setschedparam(pthread_self(), policy, &param);
 pthread_getschedparam(pthread_self(), &policy, &param);
 fprintf(stdout, "[FeatureServer::enterThread]: Thread priority set to %d\n", 
         param.sched_priority);
}


//==============================================================================
int FeatureServer::executeInThread(void *arg)
//==============================================================================
{
 ((FeatureServer*)arg)->doMessageCycle();
 return 0;
}


//==============================================================================
void FeatureServer::exitThread(void *arg)
//==============================================================================
{
 arg = arg;
}


//==============================================================================
FeatureClient::FeatureClient()
//==============================================================================
{
 d_inMsgLen = 0;
 d_featureList = NULL;
}


//==============================================================================
FeatureClient::~FeatureClient()
//==============================================================================
{
 if(d_client.getStatusCode())
  fprintf(stderr, "[FeatureClient::~FeatureClient]: %s\n", d_client.getStatusMessage());
 if(d_featureList) free(d_featureList);
}


//==============================================================================
int FeatureClient::initialize(const char *serverIp, int port, int msTimeOut, int nFeatures)
//==============================================================================
{
 struct timeval timeout;
 timeout.tv_sec = 0;
 timeout.tv_usec = msTimeOut * 1000;
 
 // connect to server
 d_client.init(serverIp, port, timeout);
 if(d_client.getStatusCode()) {
  fprintf(stderr, "[FeatureClient::initialize]: %s\n", d_client.getStatusMessage());
  return -1;
 }

 // create receive buffer
 d_inMsgLen = nFeatures * sizeof(feature_t) + 2 * sizeof(int);
 if( (d_featureList = (char *)malloc(d_inMsgLen)) == NULL) {
  fprintf(stderr, "[FeatureClient::initialize]: ERROR allocating receiver buffer.\n");
  return -1;
 }

 return 0;
}


//==============================================================================
int FeatureClient::receiveFeatureList(feature_list_t &l)
//==============================================================================
{
 int inMsgLen;
 
 // receive features from server
 if( d_client.sendAndReceive(d_featureList, 1, d_featureList, d_inMsgLen, &inMsgLen) == -1) {
  fprintf(stderr, "[FeatureClient::receiveFeatureList]: %s\n", d_client.getStatusMessage());
  return -1;
 }
 
 // check for message length
 if( inMsgLen != d_inMsgLen) {
  fprintf(stderr, "[FeatureClient::receiveFeatureList]: unexpected server message size %d (expecting %d).\n",
          inMsgLen, d_inMsgLen);
  return -1;
 }
 
 // reorder for output
 char *buf = d_featureList;
 l.frame_number = * (int *)(buf); buf += sizeof(int);
 l.num_features = * (int *)(buf); buf += sizeof(int);
 for(int i = 0; i < l.num_features; ++i) {
  l.features[i].x = * (float *)(buf); buf += sizeof(float);
  l.features[i].y = * (float *)(buf); buf += sizeof(float);
  l.features[i].val = * (int *)(buf); buf += sizeof(int);
 }
 
 return 0;
}


