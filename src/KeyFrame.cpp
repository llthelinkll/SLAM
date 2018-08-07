#include <KeyFrame.h>
#include <MapPoint.h>

using namespace SLAM;

KeyFrame::KeyFrame(Frame& frame1,Frame& frame2){
  mTimeStamp = 0;
  F1P3D = frame1.P3D;
  F2P3D = frame2.P3D;
  // center
  CP3D = (F1P3D+F2P3D)/ 2.0f;
}

KeyFrame::~KeyFrame(){
  for(MapPoint* mappoint : mvpMapPoints){
    if (mappoint)
      delete(mappoint);
  }
}

std::vector<MapPoint*>&
KeyFrame::getMapPoints(){
  return mvpMapPoints;
}

void 
KeyFrame::addMapPoint(MapPoint* mapPoint){
  mvpMapPoints.push_back(mapPoint);
}

cv::Point3f& 
KeyFrame::getWorldPos(){
  return F1P3D;
}