#include <Map.h>
#include <MapPoint.h>
#include <KeyFrame.h>
#include <iostream>

using namespace SLAM;

Map::Map(){
  
}

Map::~Map(){
  for(MapPoint* mappoint : mvpMapPoints){
    delete(mappoint);
  }
  for(KeyFrame* keyframe : mvpKeyFrames){
    delete(keyframe);
  }
}

void
Map::addMapPoint(MapPoint* mapPoint){
  // std::cout << "add MapPoint " << mapPoint->getWorldPos().t() << '\n';
  
  std::unique_lock<std::mutex> lock(mMutexMap);
  mvpMapPoints.push_back(mapPoint);
}

void
Map::addKeyFrame(KeyFrame* keyFrame){
  std::unique_lock<std::mutex> lock(mMutexMap);
  mvpKeyFrames.push_back(keyFrame);
}

std::vector<MapPoint*>&
Map::getMapPoints(){
  return mvpMapPoints;
}

std::vector<KeyFrame*>&
Map::getKeyFrames(){
  return mvpKeyFrames;
}