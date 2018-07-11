#ifndef SLAM_EXTRACTOR_H
#define SLAM_EXTRACTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace SLAM{
    
    class Extractor{
    public:
      Extractor();
      
      ~Extractor();
      
      cv::Mat extractFAST(const cv::Mat& image);
    private:
      uint fastThreshold;
      uint pointRad;
    };
}

#endif