#include <Tracker.h>
#include <Extractor.h>
#include <KeyFrame.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace SLAM;

Tracker::Tracker(){
    mpExtractor = new Extractor();
}

Tracker::~Tracker(){
    delete(mpExtractor);
}

cv::Mat 
Tracker::trackKeyFrame(const cv::Mat& image,double& timestamp){
  
  cv::Mat mat = mpExtractor->extractFAST(image);
  
  return mat;
}