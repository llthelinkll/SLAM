#ifndef SLAM_VIEWER_H
#define SLAM_VIEWER_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

namespace SLAM{
    
    class Map;
    class MapDrawer;
    class FrameDrawer;
    
    class Viewer{
    public:
      
      Viewer(Map*);
      ~Viewer();
      
      void updateFrame(const cv::Mat& image);
      
      void requestFinish();
      // run this viewer (thread please)
      void run();
      
    private:
      Map* mMap;
      MapDrawer* mMapDrawer;
      FrameDrawer* mFrameDrawer;
      bool mRequestFinish;
    };
}

#endif