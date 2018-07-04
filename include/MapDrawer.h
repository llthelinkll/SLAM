#ifndef SLAM_MAPDRAWE_H
#define SLAM_MAPDRAWE_H


namespace SLAM{
    
    class Map;
    class MapDrawer{
    public:
      MapDrawer(Map* map);
      
      void drawMapPoints();
      
    private:
      Map* mMap;
    };
}

#endif