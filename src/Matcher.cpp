#include <Matcher.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Frame.h>

using namespace SLAM;

Matcher::Matcher(float threshold){
  mThreshold = threshold;
}

Matcher::~Matcher(){
  
}

uint
Matcher::matchKeyPoints(Frame* F1,Frame* F2){
  // extract descriptor
  cv::Mat orb_desc1;
  cv::Mat orb_desc2;
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  orb->compute(F1->getCurrentFrame(), F1->getKeyPoints(), orb_desc1);
  orb->compute(F2->getCurrentFrame(), F2->getKeyPoints(), orb_desc2);
  
  return 0;
}