#ifndef SLAM_MAP_H
#define SLAM_MAP_H

#include <vector>
#include <mutex>

namespace SLAM{
    
    class MapPoint;
    class MapDrawer;
    class Map{
    friend MapDrawer;
    public:
      Map();
      
      // add new MapPoint
      void addMapPoint(MapPoint*);
      std::vector<MapPoint*>& getMapPoints();
      
      // DO NOT UPDATE this Map when someone is updating it
      std::mutex mMutexMap;
    protected:
      std::vector<MapPoint*> mvpMapPoints;
    };
}

#endif