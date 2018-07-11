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

Extractor::Extractor(){
  fastThreshold = 20;
  pointRad = 2;
}

Extractor::~Extractor(){
  
}

void
Extractor::extractFAST(const cv::Mat& image,std::vector<cv::KeyPoint>& keypoints){
  
  cv::Mat computedImage = image.clone();
  cv::Mat computedGrayImage;
  
  // convert RGB to Gray
  // because FAST need somthings simple like that
  cvtColor(computedImage, computedGrayImage, cv::COLOR_RGB2GRAY);
  
  // may be you need to blur this image
  // cv::GaussianBlur(computedImage,computedImage,cv::Size(7,7),2,2,cv::BORDER_REFLECT_101);
  
  // start find corners by FAST
  cv::FAST(computedGrayImage,keypoints,fastThreshold);
  
}