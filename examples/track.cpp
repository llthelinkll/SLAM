#include <System.h>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

using namespace SLAM;


void LoadImages(const std::string &strFile, std::vector<std::string> &vstrImageFilenames,
                std::vector<double> &vTimestamps);
                
void my_handler(int s);


int main(int argc, char const *argv[]) {
  
  std::string datasetPath = "/home/thanakorn/Desktop/gits/SLAM/datasets/rgbd";
  std::string dataFile = datasetPath + "/rgb.txt";
  std::vector<std::string> vsImageFilenames;
  std::vector<double> vdTimeStamps;
  LoadImages(dataFile,vsImageFilenames,vdTimeStamps);
  
  System sys = System();
  
  size_t n = vsImageFilenames.size();
  
  for (size_t i=0;i<n;++i){
    
    std::string& filename = vsImageFilenames[i];
    double& timestamp = vdTimeStamps[i];
    std::string filepath = datasetPath + "/" + filename;
    
    cv::Mat image;
    image = cv::imread(filepath,CV_LOAD_IMAGE_UNCHANGED);
    
    if(image.empty())
    {
      std::cerr << "Can't read image : " << filename << '\n';
      continue;
    }
    
    // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    // cv::imshow("Display window",image);
    // cv::waitKey(10);
    
    sys.inputFrame(image,timestamp);
    
    
  }
  
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  pause();
  
  return 0;
}

void my_handler(int s){
   printf("Caught signal %d\n",s);
}


void LoadImages(const std::string &strFile, std::vector<std::string> &vstrImageFilenames, std::vector<double> &vTimestamps)
{
    std::ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    std::string s0;
    std::getline(f,s0);
    std::getline(f,s0);
    std::getline(f,s0);

    std::string s;
    double t;
    std::string imageFilePath;
    
    while(!f.eof())
    {
      
      // get line
      std::getline(f,s);
      
      if(!s.empty())
      {
        // split `space` : <timeStamp> <imagefilepath>
        std::stringstream ss;
        // set string to sstream
        ss << s;
        
        // get timestamp and imagefilepath
        ss >> t;
        ss >> imageFilePath;
        vTimestamps.push_back(t);
        vstrImageFilenames.push_back(imageFilePath);
      }
    }
}