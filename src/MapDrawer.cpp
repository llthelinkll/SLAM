#include <MapDrawer.h>
#include <vector>
#include <MapPoint.h>
#include <KeyFrame.h>
#include <Map.h>
#include <pangolin/pangolin.h>

using namespace SLAM;

MapDrawer::MapDrawer(Map* map) : mMap(map){
  
}

void
MapDrawer::drawMapPoints(){
  
  std::vector<MapPoint*>& mapPoints = mMap->getMapPoints();
  
  glPointSize(5);
  glBegin(GL_POINTS);
  glColor3f(0.0,1.0,0.0);
  for ( auto mapPoint : mapPoints){
    cv::Mat& pos = mapPoint->getWorldPos();
    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
  }
  glEnd();
  
  std::vector<KeyFrame*>& keyFrames = mMap->getKeyFrames();
  
  glPointSize(8);
  glBegin(GL_POINTS);
  glColor3f(1.0,0.0,0.0);
  for ( auto keyframe : keyFrames){
    cv::Point3f& pos = keyframe->getWorldPos();
    glVertex3f(pos.x,pos.y,pos.z);
  }
  glEnd();
  
}