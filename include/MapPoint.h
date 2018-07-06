#ifndef SLAM_MAPPOINT_H
#define SLAM_MAPPOINT_H

#include <opencv2/core/core.hpp>

namespace SLAM{
    
    class MapPoint{
    public:
      MapPoint(cv::Mat&);
      MapPoint(float x,float y,float z);
      
      ~MapPoint();
      
      cv::Mat& getWorldPos();
      
    private:
      cv::Mat mWorldPos;
    };
}

#endif