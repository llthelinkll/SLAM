#ifndef SLAM_FRAME_H
#define SLAM_FRAME_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace SLAM{
    
    class Extractor;
    
    class Frame{
    public:
      Frame();
      Frame(const cv::Mat& image,double& timestamp,Extractor* extractor);
      
      ~Frame();
      
      cv::Mat& getCurrentFrame();
      std::vector<cv::KeyPoint>& getKeyPoints();
      
      bool isUndefined = false;
      
      void getPoints(std::vector<cv::Point2f>& points);
      void getNormalizedPoints(std::vector<cv::Point2f>& points);
      void update(cv::Point3f& b,cv::Mat& R2);
      
      // point in 3D world
      cv::Point3f P3D;
      
      // rotation matrix
      cv::Mat R;
    private:
      
      cv::Mat mImage;
      std::vector<cv::KeyPoint> mvKeyPoints;
    };
}

#endif