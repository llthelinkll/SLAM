#include <FrameDrawer.h>
#include <opencv2/opencv.hpp>

using namespace SLAM;

FrameDrawer::FrameDrawer(){
  
}

void
FrameDrawer::updateFrame(const cv::Mat& image){
  std::cout << image.size() << '\n';
  mCurrentImage = image;
}

void
FrameDrawer::drawFrame(){
  
  if( mCurrentImage.size().width <= 0 && mCurrentImage.size().width <= 0){
    return;
  }
  cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
  cv::imshow("Display window",mCurrentImage);
}