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
  
  map->addMapPoint(new MapPoint(1.0f,2.0f,3.0f) );
  map->addMapPoint(new MapPoint(0.0f,0.0f,1.0f) );
  map->addMapPoint(new MapPoint(1.0f,1.0f,0.4f) );
  map->addMapPoint(new MapPoint(1.0f,0.4f,0.2f) );
  
  std::thread thread = std::thread(&Viewer::run,viewer);
  
  
  
  std::cout << "Do other" << '\n';
  
  thread.join();
  
  return 0;
}