//==============================================================================
// FeatureServer.t.cpp - Examples program for FeatureServer class
// UAV follower experiment
// Vilas Chitrakaran, May 2006
//==============================================================================

#include "FeatureClientServer.hpp"

int main()
{ 
 FeatureServer server;
 FeatureServerContext_t context;
 feature_list_t features;
 
 // set server parameters
 context.port = 8000;
 context.thread_priority = 10;
 context.num_features = 10;

 if( allocateFeatureList(features, context.num_features) < 0)
  return -1;
 
 // initialize and start server thread
 if( server.initialize(context) != 0)
  return -1;
 
 int frame = 0;
 while(1){
  // do processing here....
  
  // update server buffer
  features.features[0].x += 1;
  if( server.updateFeatures(features, frame++) != 0)
   break;
  sleep(1);
 }
 
 freeFeatureList(features);
 
 return 0;
}
