#ifndef SLAM_VIEWER_H
#define SLAM_VIEWER_H

namespace SLAM{
    
    class Map;
    class MapDrawer;
    
    class Viewer{
    public:
      
      Viewer(Map*);
      
      // run this viewer (thread please)
      void run();
      
    private:
      Map* mMap;
      MapDrawer* mMapDrawer;
    };
}

#endif