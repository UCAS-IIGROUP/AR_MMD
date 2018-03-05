#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

#define _PI 3.14159

class Simulator {
  private:
    float mfAlpha, mfBeta, mfGamma, mfTransX, mfTransY, mfTransZ; 
    cv::Mat mmImage;
    cv::Mat mKf;
    
    const cv::Size mCanvasSize;

    const float mfTRANS_PARAM = 0.01;
    const float mfROT_PARAM = 1.0;

  public:
    Simulator(cv::Mat& image);
    Simulator(cv::Mat& image, cv::Size canvasSize);
    cv::Mat GenerateWarpedImage(const int &key);
    void UpdateParam(const int &key);
};
