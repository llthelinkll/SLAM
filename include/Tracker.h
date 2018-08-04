#ifndef SLAM_TRACKER_H
#define SLAM_TRACKER_H

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Frame.h>
#include <vector>

namespace SLAM{
    
    class KeyFrame;
    class Extractor;
    class Map;
    
    class Tracker{
    public:
      Tracker(cv::Mat& K,Map* map,size_t maxIterations = 100);
      ~Tracker();
      
      cv::Mat trackKeyFrame(const cv::Mat& image,double& timestamp);
    private:
      Extractor* mpExtractor;
      Map* mMap;
      Frame mCurrentFrame;
      Frame mPrevFrame;
      size_t mMaxIterations;
      
      // calibration matrix
      cv::Mat K;
      
      void reconstruct(std::vector<cv::DMatch>& matches);
      void FindHomography(std::vector<cv::DMatch>& matches,cv::Mat& H12);
      void FindFundamental(std::vector<cv::DMatch>& matches,cv::Mat& F12);
      cv::Mat ComputeH12(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);
      cv::Mat ComputeF12(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);
      float CheckHomography(cv::Mat& H12n,std::vector<cv::DMatch>& matches,std::vector<cv::Point2f>& vPoints1,std::vector<cv::Point2f>& vPoints2);
      float CheckFundamental(cv::Mat& F12n,std::vector<cv::DMatch>& matches,std::vector<cv::Point2f>& vPoints1,std::vector<cv::Point2f>& vPoints2);
      void ReconstructH(cv::Mat& H12);
      void ReconstructF(cv::Mat& F12,std::vector<cv::DMatch>& matches);
      
      void DecomposeE(cv::Mat& E12,cv::Mat& R2,cv::Mat& bx,std::vector<cv::DMatch>& matches);
      
      bool CheckRbx(cv::Mat& R2,cv::Mat& bx,std::vector<cv::DMatch>& matches);
      
      void Triangulate(const cv::Point2f& p1,const cv::Point2f& p2,cv::Mat& P1,cv::Mat& P2,cv::Mat& p3d);
    };
}

#endif