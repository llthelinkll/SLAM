#ifndef SLAM_MATCHER_H
#define SLAM_MATCHER_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

namespace SLAM{
    
    class Frame;
    
    class Matcher{
    public:
      Matcher(float threshold);
      
      ~Matcher();
      
      uint matchKeyPoints(Frame* F1,Frame* F2,std::vector<cv::DMatch>& matches);
    private:
      float mThreshold;
    };
}

#endif