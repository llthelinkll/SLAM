#include <MapPoint.h>
#include <vector>
#include <iostream>


using namespace SLAM;

MapPoint::MapPoint(cv::Mat& mat){
  mWorldPos = mat;
  std::cout << mWorldPos << '\n';
}

MapPoint::MapPoint(float x,float y,float z){
  float tmp[3] = {x,y,z};
  mWorldPos = cv::Mat(3,1,CV_32F,tmp).clone();
}

MapPoint::~MapPoint(){
}

cv::Mat&
MapPoint::getWorldPos(){
  return mWorldPos;
}

