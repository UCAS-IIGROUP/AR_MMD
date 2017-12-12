#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

#define _PI 3.14159

class Simulator {
  private:
    float mf_alpha, mf_beta, mf_gamma, mf_trans_u, mf_trans_v, mf_trans_z; 
    cv::Mat m_image;
    cv::Mat mKf;

    const float TRANS_PARAM = 0.01;
    const float ROT_PARAM = 3.0;

  public:
    Simulator(cv::Mat& image);
    cv::Mat GenerateWarpedImage(const int &key);
    void UpdateParam(const int &key);
};
