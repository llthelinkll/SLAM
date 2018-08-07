#include <Tracker.h>
#include <Extractor.h>
#include <KeyFrame.h>
#include <Frame.h>
#include <MapPoint.h>
#include <Map.h>

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

Tracker::Tracker(cv::Mat& _K,Map* map,size_t maxIterations) : mMap(map),mMaxIterations(maxIterations),K(_K){
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
  std::thread reconstructF(&Tracker::ReconstructF,this,std::ref(F12),std::ref(matches));
  reconstructF.join();
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

void
Tracker::ReconstructF(cv::Mat& F12,std::vector<cv::DMatch>& matches){
  
  cv::Mat E12 = K.t()*F12*K;
  
  cv::Mat R2, bx;

  // Recover the 4 motion hypotheses
  // nx' * F * nx''T = 0
  // nx' *  K-t * R' * bx * R''t * K-1 * nx''t = 0
  // where bx is baseline (skew matrix)
  // Essential matrix ; E = R' * bx * R''t
  // R1 is I
  DecomposeE(E12,R2,bx,matches);
  
}

void 
Tracker::DecomposeE(cv::Mat& E12,cv::Mat& R2,cv::Mat& bx,std::vector<cv::DMatch>& matches){
  
  // Essential matrix ; E = R' * bx * R''t
  // we will assume which R' is I (no rotation)
  // so Essential matrix ; E = bx * R''t    ------ (1)
  
  cv::Mat u,w,vt;
  cv::SVD::compute(E12,w,u,vt);
  
  // Decompose E to u * w * vt
  // define Z and W
  // Z =  | 0   1   0  |
  //      | -1  0   0  |
  //      | 0   0   0  |
  // W =  | 0   -1  0  |
  //      | 1   0   0  |
  //      | 0   0   1  |
  // when Z*W = | 1   0   0  | = w
  //            | 0   1   0  |
  //            | 0   0   0  |
  // NOTE: the reason, w is the matrix above.
  // 1. singular values of skew matrix is
  //            | d   0   0  | 
  //            | 0   d   0  |
  //            | 0   0   0  |
  // 2. rotation matrix doesn't effect singular values
  // conclusion : ideal `w` should be like that
  
  // from this case we can use Z*W instead of w
  // but we have 4 possibilities -> Z*W,Zt*Wt,-Zt*W,-Z*Wt
  
  // now; E = u * Z * W * vt
  // add u-1*u which equal to ut*u (u is orthogonal matrix)
  // the value of u*ut is going to be I
  // then; E = u * Z * ut * u * W * vt ------- (2)
  // the equation above, we will know somehow that
  // from (1) ; E = bx * R''t 
  // u * Z * ut = bx
  // u * W * vt = R''t
  // TODO: I still don't understand why they're so sure about decompose bx and R''T from that pattern of matrix
  
  cv::Mat Z(3,3,CV_32F,cv::Scalar(0));
  Z.at<float>(0,1)=1;
  Z.at<float>(1,0)=-1;
  
  cv::Mat W(3,3,CV_32F,cv::Scalar(0));
  W.at<float>(0,1)=-1;
  W.at<float>(1,0)=1;
  W.at<float>(2,2)=1;

  // from ORB SLAM
  cv::Mat t;
  u.col(2).copyTo(t);
  t=t/cv::norm(t);
  std::cout << "t : " << t << '\n';
  
  std::vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
  std::vector<uint> vNGood = {0,0,0,0};
  cv::Mat bx_,R2_;
  
  // possibility 1 : Z * W
  bx_ = u*Z*u.t();
  R2_ = u*W*vt;
  std::cout << "bx_ : " << bx_ << '\n';
  vNGood[0] = CheckRbx(R2_,bx_,matches,vP3D1);
  
  // possibility 2 : Zt * Wt
  bx_ = u*Z.t()*u.t();
  R2_ = u*W.t()*vt;
  std::cout << "bx_ : " << bx_ << '\n';
  vNGood[1] = CheckRbx(R2_,bx_,matches,vP3D2);
  
  // possibility 3 : -Z * Wt
  bx_ = -u*Z*u.t();
  R2_ = u*W.t()*vt;
  std::cout << "bx_ : " << bx_ << '\n';
  vNGood[3] = CheckRbx(R2_,bx_,matches,vP3D3);
  
  // possibility 4 : -Zt * W
  bx_ = -u*Z.t()*u.t();
  R2_ = u*W*vt;
  std::cout << "bx_ : " << bx_ << '\n';
  vNGood[4] = CheckRbx(R2_,bx_,matches,vP3D4);
  
  // OPTIMIZE: I think CheckRbx, 3 possibility will alway fail.
  // should we check only 1 point?
  std::vector<uint>::iterator max_elem = std::max_element(vNGood.begin(),vNGood.end());
  size_t dist = std::distance(max_elem,vNGood.begin());
  
  std::vector<cv::Point3f> vP3D;
  if (dist == 0){
    vP3D = vP3D1;
    bx = bx_;
    R2 = R2_;
  }else if(dist == 1){
    vP3D = vP3D2;
    bx = bx_;
    R2 = R2_;
  }else if(dist == 2){
    vP3D = vP3D3;
    bx = bx_;
    R2 = R2_;
  }else if(dist == 3){
    vP3D = vP3D4;
    bx = bx_;
    R2 = R2_;
  }
  
  // update baseline and rotation matrix to current frame
  float x = -bx.at<float>(1,2);
  float y = bx.at<float>(0,2);
  float z = -bx.at<float>(0,1);
  cv::Point3f F2P3D = {x,y,z};
  F2P3D += mPrevFrame.P3D;
  
  std::cout << "frame F1P3D: " << mPrevFrame.P3D << '\n';
  std::cout << "frame F2P3D: " << F2P3D << '\n';
  mCurrentFrame.update(F2P3D,R2);
  
  KeyFrame* keyFrame = new KeyFrame(mPrevFrame,mCurrentFrame);
  
  // add map point
  for (auto P3D : vP3D){
    MapPoint* mappoint = new MapPoint(P3D.x,P3D.y,P3D.z);
    keyFrame->addMapPoint(mappoint);
    mMap->addMapPoint(mappoint);
  }
  mMap->addKeyFrame(keyFrame);
  
}

uint 
Tracker::CheckRbx(cv::Mat& R2,cv::Mat& bx,std::vector<cv::DMatch>& matches,std::vector<cv::Point3f>& vP3D){
  
  // number of good point from this R2 and bx
  uint nGood = 0;
  
  // Threshold for re-projection
  float th2 = 4;
  
  // from calibration
  const float fx = K.at<float>(0,0);
  const float fy = K.at<float>(1,1);
  const float cx = K.at<float>(0,2);
  const float cy = K.at<float>(1,2);
  
  // get vector from skew matrix
  cv::Mat t(3,1,CV_32F,cv::Scalar(0));
  t.at<float>(0,0) = - bx.at<float>(1,2);
  t.at<float>(1,0) = bx.at<float>(0,2);
  t.at<float>(2,0) = - bx.at<float>(0,1);
  
  // Camera 1 Projection Matrix; P1 = K[I|0]
  cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
  K.copyTo(P1.rowRange(0,3).colRange(0,3));

  cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);
  
  // Camera 2 Projection Matrix P2 = K[R2|bx]
  cv::Mat P2(3,4,CV_32F);
  R2.copyTo(P2.rowRange(0,3).colRange(0,3));
  t.copyTo(P2.rowRange(0,3).col(3));
  P2 = K*P2;
  
  cv::Mat O2 = -R2.t()*t;
  
  std::vector<cv::Point2f> vPoints1;
  std::vector<cv::Point2f> vPoints2;
  Frame& F1 = mPrevFrame;
  Frame& F2 = mCurrentFrame;
  F1.getNormalizedPoints(vPoints1);
  F2.getNormalizedPoints(vPoints2);
  
  for (auto match : matches){
      
      // input
      const cv::Point2f& kp1 = vPoints1[match.queryIdx];
      const cv::Point2f& kp2 = vPoints2[match.trainIdx];
      
      cv::Mat p3dC1;

      // I think this function will do intersection of vector for us
      // the triangle of epipolar planar !!!!
      Triangulate(kp1,kp2,P1,P2,p3dC1);
      
      // TODO: I don't understand this
      // if(isfinite(p3dC1.at<float>(0)) || isfinite(p3dC1.at<float>(1)) || isfinite(p3dC1.at<float>(2)))
      // {
      //     continue;
      // }
      
      // NOTE: when 2 vector is quite parallel (infinity)
      // Check parallax
      cv::Mat normal1 = p3dC1 - O1;
      float dist1 = cv::norm(normal1);
      
      cv::Mat normal2 = p3dC1 - O2;
      float dist2 = cv::norm(normal2);
      
      float cosParallax = normal1.dot(normal2)/(dist1*dist2);
      
      // 2 vector should not be parallax (quite parallel)
      if (cosParallax >= 0.99998){
        continue;
      }
      
      // 3D point must stay in front of both camera
      // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
      if(p3dC1.at<float>(2)<=0)
          continue;

      // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
      cv::Mat p3dC2 = R2*p3dC1+t;

      if(p3dC2.at<float>(2)<=0)
          continue;
          
      // NOTE: when you re-project from 3d to 2d image
      // we should get nearby point
      
      // Check reprojection error in first image
      float im1x, im1y;
      float invZ1 = 1.0/p3dC1.at<float>(2);
      im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
      im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

      float squareError1 = (im1x-kp1.x)*(im1x-kp1.x)+(im1y-kp1.y)*(im1y-kp1.y);

      if(squareError1>th2)
          continue;

      // Check reprojection error in second image
      float im2x, im2y;
      float invZ2 = 1.0/p3dC2.at<float>(2);
      im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
      im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

      float squareError2 = (im2x-kp2.x)*(im2x-kp2.x)+(im2y-kp2.y)*(im2y-kp2.y);

      if(squareError2>th2)
          continue;
      
      float& x  = p3dC1.at<float>(0,0);
      float& y  = p3dC1.at<float>(0,1);
      float& z  = p3dC1.at<float>(0,2);        
      
      // append 3d point
      vP3D.push_back({x,y,z});
      
      // count number of good points
      ++nGood;
      // std::cout << "3d point!! : " << p3dC1 << '\n';
      
    }
      
  
  return nGood;
}

void 
Tracker::Triangulate(const cv::Point2f& p1,const cv::Point2f& p2,cv::Mat& P1,cv::Mat& P2,cv::Mat& p3d){
  
  // TODO: find how to using triangulate to compute 3d position
  cv::Mat A(4,4,CV_32F);

  A.row(0) = p1.x*P1.row(2)-P1.row(0);
  A.row(1) = p1.y*P1.row(2)-P1.row(1);
  A.row(2) = p2.x*P2.row(2)-P2.row(0);
  A.row(3) = p2.y*P2.row(2)-P2.row(1);

  cv::Mat u,w,vt;
  cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
  p3d = vt.row(3).t();
  p3d = p3d.rowRange(0,3)/p3d.at<float>(3);
  
}