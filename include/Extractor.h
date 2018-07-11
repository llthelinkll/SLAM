#ifndef SLAM_EXTRACTOR_H
#define SLAM_EXTRACTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

namespace SLAM{
    
    class Extractor{
    public:
      Extractor();
      
      ~Extractor();
      
      void extractFAST(const cv::Mat& image,std::vector<cv::KeyPoint>& keypoints);
    private:
      uint fastThreshold;
      uint pointRad;
    };
}

#endif