#include <System.h>

#include <Tracker.h>
#include <Viewer.h>
#include <MapPoint.h>
#include <Map.h>
#include <thread>

using namespace SLAM;

System::System(){
  mTracker = new Tracker();
  mMap = new Map();
  mViewer = new Viewer(mMap);
  
  // run viewer as daemon thread
  mptViewer = new std::thread(&Viewer::run,mViewer);
}

void 
System::inputFrame(const cv::Mat& image,double& timestamp){
  
  float x = rand() % 10;
  float y = rand() % 10;
  float z = rand() % 10;
  mMap->addMapPoint(new MapPoint(x,y,z) );
  
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