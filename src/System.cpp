#include <System.h>

#include <Tracker.h>
#include <Viewer.h>
#include <MapPoint.h>
#include <Map.h>
#include <thread>

using namespace SLAM;

System::System(cv::Mat& K){
  mMap = new Map();
  mTracker = new Tracker(K,mMap);
  mViewer = new Viewer(mMap);
  
  // run viewer as daemon thread
  mptViewer = new std::thread(&Viewer::run,mViewer);
}

void 
System::inputFrame(const cv::Mat& image,double& timestamp){
  // == TrackMonocular
  
  // float x = rand() % 10;
  // float y = rand() % 10;
  // float z = rand() % 10;
  // mMap->addMapPoint(new MapPoint(x,y,z) );
  
  // create keyframe from tracking image
  cv::Mat frame = mTracker->trackKeyFrame(image,timestamp);
  
  mViewer->updateFrame(frame);
}

System::~System(){
  mViewer->requestFinish();
  mptViewer->join();
  delete(mptViewer);
  delete(mTracker);
  delete(mViewer);
  delete(mMap);
}