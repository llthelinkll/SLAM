#include <Map.h>
#include <MapPoint.h>
#include <iostream>

using namespace SLAM;

Map::Map(){
  
}

Map::~Map(){
  for(MapPoint* mappoint : mvpMapPoints){
    delete(mappoint);
  }
}

void
Map::addMapPoint(MapPoint* mapPoint){
  std::cout << "add MapPoint " << mapPoint->getWorldPos().t() << '\n';
  
  std::unique_lock<std::mutex> lock(mMutexMap);
  mvpMapPoints.push_back(mapPoint);
}

std::vector<MapPoint*>&
Map::getMapPoints(){
  return mvpMapPoints;
}