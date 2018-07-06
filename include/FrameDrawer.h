#ifndef SLAM_FRAMEDRAWER_H
#define SLAM_FRAMEDRAWER_H

#include <string>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

namespace SLAM{
    
    class FrameDrawer{
    public:
      FrameDrawer();
      
      // add input
      void drawFrame();
      void updateFrame(const cv::Mat& image);
      
    private:
      cv::Mat mCurrentImage;
    };
}

#endif