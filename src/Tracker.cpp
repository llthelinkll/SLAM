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
#include <vector>
#include <thread>

#include <Matcher.h>

using namespace SLAM;

Tracker::Tracker(size_t maxIterations) : mMaxIterations(maxIterations){
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
  
  // pair of keypoint index of frame1 and frame2
  std::vector<cv::DMatch> vpMatches;
  
  Matcher matcher(0.9);
  
  // ORB matcher
  uint nmatches = matcher.matchKeyPoints(&mCurrentFrame,&mPrevFrame,vpMatches);
  std::cout << "matcher number " << nmatches << '\n';
  
  // TODO PTAM
  reconstruct(vpMatches);
  
  // create homography / fundamental
  
  mPrevFrame = mCurrentFrame;
  
  return mCurrentFrame.getCurrentFrame();
}



void
Tracker::reconstruct(std::vector<cv::DMatch>& matches){
  
  cv::Mat H12;
  std::thread threadH(&Tracker::FindHomography,this,std::ref(matches),std::ref(H12));
  cv::Mat F12;
  std::thread threadF(&Tracker::FindFundamental,this,std::ref(matches),std::ref(F12));
  threadH.join();
  threadF.join();
  
  std::cout << "H : " << H12 << '\n';
  std::cout << "F : " << F12 << '\n';
  std::thread reconstructH(&Tracker::ReconstructH,this,std::ref(H12));
  reconstructH.join();
  // TODO create threadF for fundamental matrix initialization
}

void
Tracker::FindFundamental(std::vector<cv::DMatch>& matches,cv::Mat& F12){
  std::vector<cv::Point2f> vPoints1;
  std::vector<cv::Point2f> vPoints2;
  
  /*
    GET POINT2F
    get point2f from keypoint of each Frame
  */
  Frame& F1 = mPrevFrame;
  Frame& F2 = mCurrentFrame;
  F1.getNormalizedPoints(vPoints1);
  F2.getNormalizedPoints(vPoints2);
  
  std::vector<cv::Point2f> vInliersPoints1(8);
  std::vector<cv::Point2f> vInliersPoints2(8);
  
  /*
    RANSAC
    find the best iterator which return the best homography score
    the score from every matche points
    ITERATIONS RANDOM INDEX
    create random index of each iteration
  */
  size_t max_random = matches.size();
  float currentDist;
  float bestDist = -1.0f;
  for (size_t iteration = 0; iteration < mMaxIterations; ++iteration) {
    
    // random 8 correspondences
    for (size_t i = 0; i < 8; ++i) {
      
      // random
      int randIdx = std::rand() % max_random;
      
      vInliersPoints1[i] = vPoints1[matches[randIdx].queryIdx];
      vInliersPoints2[i] = vPoints2[matches[randIdx].trainIdx];
    }
    
    cv::Mat F12n = ComputeF12(vInliersPoints1,vInliersPoints2);
    
    currentDist = CheckFundamental(F12n,matches,vPoints1,vPoints2);

    if(currentDist < bestDist || bestDist == -1)
    {
      F12 = F12n;
      bestDist = currentDist;
    }
  }
}

cv::Mat 
Tracker::ComputeF12(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2)
{
  const int N = vP1.size();

  cv::Mat A(N,9,CV_32F);

  for(int i=0; i<N; i++)
  {
    const float u1 = vP1[i].x;
    const float v1 = vP1[i].y;
    const float u2 = vP2[i].x;
    const float v2 = vP2[i].y;

    A.at<float>(i,0) = u1*u2;
    A.at<float>(i,1) = u2*v1;
    A.at<float>(i,2) = u2;
    A.at<float>(i,3) = u1*v2;
    A.at<float>(i,4) = v2*v1;
    A.at<float>(i,5) = v2;
    A.at<float>(i,6) = v2;
    A.at<float>(i,7) = v1;
    A.at<float>(i,8) = 1;
    
  }

  cv::Mat u,w,vt;

  cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
  
  cv::Mat Fpre = vt.row(8).reshape(0, 3);
  
  cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

  w.at<float>(2)=0;

  return  u*cv::Mat::diag(w)*vt;
}

float 
Tracker::CheckFundamental(cv::Mat& F12n,std::vector<cv::DMatch>& matches,std::vector<cv::Point2f>& vPoints1,std::vector<cv::Point2f>& vPoints2){
  
  const float f11 = F12n.at<float>(0,0);
  const float f12 = F12n.at<float>(0,1);
  const float f13 = F12n.at<float>(0,2);
  const float f21 = F12n.at<float>(1,0);
  const float f22 = F12n.at<float>(1,1);
  const float f23 = F12n.at<float>(1,2);
  const float f31 = F12n.at<float>(2,0);
  const float f32 = F12n.at<float>(2,1);
  const float f33 = F12n.at<float>(2,2);
  
  // sum of distance
  float totalDist = 0;
  
  for (auto match : matches){
      
      // input
      float u1 = vPoints1[match.queryIdx].x;
      float v1 = vPoints1[match.queryIdx].y;
      float u2 = vPoints2[match.trainIdx].x;
      float v2 = vPoints2[match.trainIdx].y;
      
      // Reprojection error in second image
      // l2 = F*x1 = (a2,b2,c2)

      const float a2 = f11*u1+f12*v1+f13;
      const float b2 = f21*u1+f22*v1+f23;
      const float c2 = f31*u1+f32*v1+f33;

      const float num2 = a2*u2+b2*v2+c2;

      const float squareDist1 = num2*num2;
      
      // Reprojection error in first image
      // l1 = F*x2 = (a1,b1,c1)

      const float a1 = f11*u2+f12*v2+f13;
      const float b1 = f21*u2+f22*v2+f23;
      const float c1 = f31*u2+f32*v2+f33;

      const float num1 = a1*u1+b1*v1+c1;

      const float squareDist2 = num1*num1;
      
      totalDist += squareDist1 + squareDist2;
  }
  
  // NOTE: becareful when totalDist is leak
  return totalDist;
}

void 
Tracker::FindHomography(std::vector<cv::DMatch>& matches,cv::Mat& H12){
  
  std::vector<cv::Point2f> vPoints1;
  std::vector<cv::Point2f> vPoints2;
  
  /*
    GET POINT2F
    get point2f from keypoint of each Frame
  */
  Frame& F1 = mPrevFrame;
  Frame& F2 = mCurrentFrame;
  F1.getNormalizedPoints(vPoints1);
  F2.getNormalizedPoints(vPoints2);
  
  std::vector<cv::Point2f> vInliersPoints1(4);
  std::vector<cv::Point2f> vInliersPoints2(4);
  
  /*
    RANSAC
    find the best iterator which return the best homography score
    the score from every matche points
    ITERATIONS RANDOM INDEX
    create random index of each iteration
  */
  size_t max_random = matches.size();
  float currentDist;
  float bestDist = -1.0f;
  for (size_t iteration = 0; iteration < mMaxIterations; ++iteration) {
    
    // random 4 correspondences
    for (size_t i = 0; i < 4; ++i) {
      
      // random
      int randIdx = std::rand() % max_random;
      
      vInliersPoints1[i] = vPoints1[matches[randIdx].queryIdx];
      vInliersPoints2[i] = vPoints2[matches[randIdx].trainIdx];
    }
    
    cv::Mat H12n = ComputeH12(vInliersPoints1,vInliersPoints2);
    
    currentDist = CheckHomography(H12n,matches,vPoints1,vPoints2);

    if(currentDist < bestDist || bestDist == -1)
    {
      H12 = H12n;
      bestDist = currentDist;
    }
  }
  
  
}

cv::Mat 
Tracker::ComputeH12(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2)
{
  const int N = vP1.size();

  cv::Mat A(2*N,9,CV_32F);

  for(int i=0; i<N; i++)
  {
    const float u1 = vP1[i].x;
    const float v1 = vP1[i].y;
    const float u2 = vP2[i].x;
    const float v2 = vP2[i].y;

    A.at<float>(2*i,0) = 0.0;
    A.at<float>(2*i,1) = 0.0;
    A.at<float>(2*i,2) = 0.0;
    A.at<float>(2*i,3) = -u1;
    A.at<float>(2*i,4) = -v1;
    A.at<float>(2*i,5) = -1;
    A.at<float>(2*i,6) = v2*u1;
    A.at<float>(2*i,7) = v2*v1;
    A.at<float>(2*i,8) = v2;

    A.at<float>(2*i+1,0) = -u1;
    A.at<float>(2*i+1,1) = -v1;
    A.at<float>(2*i+1,2) = -1;
    A.at<float>(2*i+1,3) = 0.0;
    A.at<float>(2*i+1,4) = 0.0;
    A.at<float>(2*i+1,5) = 0.0;
    A.at<float>(2*i+1,6) = u2*u1;
    A.at<float>(2*i+1,7) = u2*v1;
    A.at<float>(2*i+1,8) = u2;

  }

  cv::Mat u,w,vt;

  cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

  return vt.row(8).reshape(0, 3);
}

float 
Tracker::CheckHomography(cv::Mat& H12n,std::vector<cv::DMatch>& matches,std::vector<cv::Point2f>& vPoints1,std::vector<cv::Point2f>& vPoints2){
  
  const float h11 = H12n.at<float>(0,0);
  const float h12 = H12n.at<float>(0,1);
  const float h13 = H12n.at<float>(0,2);
  const float h21 = H12n.at<float>(1,0);
  const float h22 = H12n.at<float>(1,1);
  const float h23 = H12n.at<float>(1,2);
  const float h31 = H12n.at<float>(2,0);
  const float h32 = H12n.at<float>(2,1);
  const float h33 = H12n.at<float>(2,2);
  
  // sum of distance
  float totalDist = 0;
  
  for (auto match : matches){
      
      // input
      float u1 = vPoints1[match.queryIdx].x;
      float v1 = vPoints1[match.queryIdx].y;
      
      // predict
      float pw2 = 1/(u1*h31+v1*h32+h33);
      float pu2 = (u1*h11+v1*h12+h13)*pw2;
      float pv2 = (u1*h21+v1*h22+h23)*pw2;
      
      // real
      float u2 = vPoints2[match.trainIdx].x;
      float v2 = vPoints2[match.trainIdx].y;
      
      totalDist += (u2-pu2)*(u2-pu2) + (v2-pv2)*(v2-pv2);
  }
  
  // NOTE: becareful when totalDist is leak
  return totalDist;
}

void
Tracker::ReconstructH(cv::Mat& H12){
  
}