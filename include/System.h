#ifndef SLAM_SYSTEM_H
#define SLAM_SYSTEM_H

#include <string>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <thread>

namespace SLAM{
    
    class Tracker;
    class Viewer;
    class Map;
    
    class System{
    public:
      System();
      ~System();
      
      // add input
      void inputFrame(const cv::Mat& image,double& timestamp);
      
    private:
      Map* mMap;
      Tracker* mTracker;
      Viewer* mViewer;
      std::thread* mptViewer;
      std::thread* mptTracker;
    };
}

#endif