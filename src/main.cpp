#include <Viewer.h>
#include <MapPoint.h>
#include <Map.h>
#include <mutex>
#include <thread>
#include <iostream>

using namespace SLAM;

int main(int argc, char const *argv[]) {
  
  Map* map = new Map();
  
  Viewer* viewer = new Viewer(map);
  
  map->addMapPoint(new MapPoint(1.0f,0.0f,0.0f) );
  map->addMapPoint(new MapPoint(2.0f,0.0f,0.0f) );
  map->addMapPoint(new MapPoint(3.0f,0.0f,0.0f) );
  map->addMapPoint(new MapPoint(4.0f,0.0f,0.0f) );
  map->addMapPoint(new MapPoint(5.0f,0.0f,0.0f) );
  map->addMapPoint(new MapPoint(6.0f,0.0f,0.0f) );
  
  std::thread thread = std::thread(&Viewer::run,viewer);
  
  
  
  std::cout << "Do other" << '\n';
  
  thread.join();
  
  return 0;
}