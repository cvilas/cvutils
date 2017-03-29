//==============================================================================
// FeatureClient.t.cpp - Examples program for FeatureClient class
// UAV follower experiment
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "FeatureClientServer.hpp"

//==============================================================================
// main: Connects to a feature server and delivers updates from server.
//==============================================================================
int main(int argc, char *argv[])
{
 FeatureClient client;
 feature_list_t features;
 int nFeatures = 10;
 
 // initialize a client and connect to server
 if(client.initialize("127.0.0.1", 8000, 50, nFeatures) != 0) 
  return -1;
 
 // create feature list
 if( allocateFeatureList(features, nFeatures) < 0 )
  return -1;
 
 // server read loop
 int msgNum = 0;
 while(1) {
  if(msgNum > 1000) break;
  ++msgNum;

  // Ask for update from server
  if( client.receiveFeatureList(features) == -1)
   break;
  
  if( features.frame_number < 0 ) // server hasn't started updating frames yet
   continue;

  // print recevied frame number
  fprintf(stdout, "latest frame received: %d.\n", features.frame_number);
 }

 freeFeatureList(features);
 
 return 0;
}
