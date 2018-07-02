#include <Map.h>

using namespace SLAM;

Map::Map(){
  
}

void
Map::addMapPoint(MapPoint* mapPoint){
  mvpMapPoints.push_back(mapPoint);
}