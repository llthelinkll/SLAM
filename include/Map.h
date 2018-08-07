#ifndef SLAM_MAP_H
#define SLAM_MAP_H

#include <vector>
#include <mutex>

namespace SLAM{
    
    class MapPoint;
    class KeyFrame;
    class MapDrawer;
    class Map{
    friend MapDrawer;
    public:
      Map();
      ~Map();
      
      // add new MapPoint
      void addMapPoint(MapPoint*);
      void addKeyFrame(KeyFrame*);
      std::vector<MapPoint*>& getMapPoints();
      std::vector<KeyFrame*>& getKeyFrames();
      
      // DO NOT UPDATE this Map when someone is updating it
      std::mutex mMutexMap;
    protected:
      std::vector<MapPoint*> mvpMapPoints;
      std::vector<KeyFrame*> mvpKeyFrames;
    };
}

#endif