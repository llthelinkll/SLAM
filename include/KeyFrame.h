#ifndef SLAM_KEYFRAME_H
#define SLAM_KEYFRAME_H

#include <Frame.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace SLAM{
    
    class MapPoint;
    class KeyFrame{
    public:
      KeyFrame(Frame& frame1,Frame& frame2);
      
      ~KeyFrame();
      
      std::vector<MapPoint*>& getMapPoints();
      void addMapPoint(MapPoint* mapPoint);
      cv::Point3f& getWorldPos();
    private:
      
      double mTimeStamp;
      
      // 3d position of Frame1,Frame2 and center beteern frames
      cv::Point3f F1P3D;
      cv::Point3f F2P3D;
      cv::Point3f CP3D;
      
      // MapPoint of this Frame
      std::vector<MapPoint*> mvpMapPoints;
    };
}

#endif