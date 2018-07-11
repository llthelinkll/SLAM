#include <Tracker.h>
#include <Extractor.h>
#include <KeyFrame.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Matcher.h>

using namespace SLAM;

Tracker::Tracker(){
    mpExtractor = new Extractor();
}

Tracker::~Tracker(){
    delete(mpExtractor);
}

cv::Mat 
Tracker::trackKeyFrame(const cv::Mat& image,double& timestamp){
  
  if (mPrevFrame.isUndefined){
    mPrevFrame = Frame(image,timestamp,mpExtractor);
    return mPrevFrame.getCurrentFrame();
  }
  
  // extract keypoints (FAST) from image then store in Frame
  mCurrentFrame = Frame(image,timestamp,mpExtractor);
  
  
  
  Matcher matcher(0.9);
  uint nmatches = matcher.matchKeyPoints(&mCurrentFrame,&mPrevFrame);
  std::cout << "matcher number " << nmatches << '\n';
  
  // std::cout << "ORB desc" << '\n';
  // std::cout << orb_desc.size() << '\n';
  
  // cv::BFMatcher matcher(cv::NORM_HAMMING);
  // std::vector<std::vector<cv::DMatch> > nn_matches;


  // TODO ORB
  // TODO PTAM
  
  // create homography / fundamental
  
  mPrevFrame = mCurrentFrame;
  
  return mCurrentFrame.getCurrentFrame();
}