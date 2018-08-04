#include <Frame.h>
#include <Extractor.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace SLAM;

Frame::Frame(){
  isUndefined = true;
}

Frame::Frame(const cv::Mat& image,double& timestamp,Extractor* extractor){
  isUndefined = false;
  
  this->mImage = image.clone();
  extractor->extractFAST(image,mvKeyPoints);
}


Frame::~Frame(){
  
}

cv::Mat&
Frame::getCurrentFrame(){
  
  size_t pointRad = 5;
  for (cv::KeyPoint& keypoint : mvKeyPoints){
  
    // std::cout << "keypoint : " << keypoint.pt << '\n';
    cv::circle(mImage,keypoint.pt,pointRad,cv::Scalar(0,255,0));
  
  }
  return mImage;
}

std::vector<cv::KeyPoint>& 
Frame::getKeyPoints(){
  return mvKeyPoints;
}

void 
Frame::getPoints(std::vector<cv::Point2f>& points){
  points.clear();
  points.reserve(mvKeyPoints.size());
  for(auto keypoint : mvKeyPoints){
    points.push_back(keypoint.pt);
  }
}

void 
Frame::getNormalizedPoints(std::vector<cv::Point2f>& points){
  points.clear();
  points.reserve(mvKeyPoints.size());
  for(auto keypoint : mvKeyPoints){
    points.push_back(keypoint.pt- cv::Point2f(mImage.rows,mImage.cols));
  }
}