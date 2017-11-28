#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#pragma once
using namespace std;

class Frame {
  private:
    cv::Mat mmImage;
    cv::Mat mmDescriptor;
    vector<cv::KeyPoint> mvKeypoints;
    vector<cv::Point3f> mvKeyObjPoints;

  public:
    cv::Mat getImage();
    cv::Mat getDescriptors();
    vector<cv::KeyPoint> getKeyPoints();

  private:

  public:
    Frame(cv::Mat& _image, cv::Mat& _desc, vector<cv::KeyPoint>& _kpts );
    Frame( );
};
