#include <Saba/Base/UnicodeUtil.h>
#include <Saba/Base/Path.h>
#include <Saba/Model/MMD/MMDModel.h>
#include <Saba/Model/MMD/PMDModel.h>
#include <Saba/Model/MMD/PMXModel.h>
#include <Saba/Model/MMD/VMDFile.h>
#include <Saba/Model/MMD/VMDAnimation.h>
#include <Saba/Model/MMD/VPDFile.h>

#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>

#include <limits>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <string>

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

class viewer {
  private:
    inline  std::unique_lock<std::recursive_mutex> LockerImage()
    {
      return  std::unique_lock<std::recursive_mutex>( mtx_image );
    }

    inline  std::unique_lock<std::recursive_mutex> LockerPose()
    {
      return  std::unique_lock<std::recursive_mutex>( mtx_pose );
    }

    inline  std::unique_lock<std::recursive_mutex> LockerParams()
    {
      return  std::unique_lock<std::recursive_mutex>( mtx_params );
    }

  public:
    viewer(int w, int h);
    viewer();
    ~viewer();
    bool SetImage(cv::Mat& image);
    void Run();
    void SetCamPose(const cv::Mat& Tcb);
    void SetMenuParams(const bool draw_cube, const bool end);
      
  private:
    cv::Mat mImage;
    cv::Mat mTcb_cv;
    pangolin::OpenGlMatrix mTcb_gl;
    int mWidth;
    int mHeight;
    bool mbMenuDrawCube;
    bool mbMenuEnd;

    std::recursive_mutex mtx_image;
    std::recursive_mutex mtx_pose;
    std::recursive_mutex mtx_params;

    std::string msModelPath;
    std::string msMMDDataPath;
    std::string msVMDPath;
    std::shared_ptr<saba::MMDModel> mpModel;

  public:

};
