#ifndef SLAM_TRACKER_H
#define SLAM_TRACKER_H

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Frame.h>

namespace SLAM{
    
    class KeyFrame;
    class Extractor;
    
    class Tracker{
    public:
      Tracker();
      ~Tracker();
      
      cv::Mat trackKeyFrame(const cv::Mat& image,double& timestamp);
    private:
      Extractor* mpExtractor;
      Frame mCurrentFrame;
      Frame mPrevFrame;
    };
}

#endif