#ifndef SLAM_MAP_H
#define SLAM_MAP_H

#include <vector>

namespace SLAM{
    
    class MapPoint;
    class MapDrawer;
    class Map{
    friend MapDrawer;
    public:
      Map();
      
      // add new MapPoint
      void addMapPoint(MapPoint*);
      
    protected:
      std::vector<MapPoint*> mvpMapPoints;
    };
}

#endif