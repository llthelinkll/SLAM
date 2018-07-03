#include <Map.h>
#include <MapPoint.h>
#include <iostream>

using namespace SLAM;

Map::Map(){
  
}

void
Map::addMapPoint(MapPoint* mapPoint){
  std::cout << "add MapPoint " << mapPoint->getWorldPos().t() << '\n';
  mvpMapPoints.push_back(mapPoint);
}