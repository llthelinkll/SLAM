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
      Tracker(size_t maxIterations = 100);
      ~Tracker();
      
      cv::Mat trackKeyFrame(const cv::Mat& image,double& timestamp);
    private:
      Extractor* mpExtractor;
      Frame mCurrentFrame;
      Frame mPrevFrame;
      size_t mMaxIterations;
      
      void reconstruct(std::vector<cv::DMatch>& matches);
      void FindHomography(std::vector<cv::DMatch>& matches,cv::Mat& H12);
      cv::Mat ComputeH12(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);
      float CheckHomography(cv::Mat& H12n,std::vector<cv::DMatch>& matches,std::vector<cv::Point2f>& vPoints1,std::vector<cv::Point2f>& vPoints2);
      void ReconstructH(cv::Mat& H12);
    };
}

#endif