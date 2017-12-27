#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include "utils.hpp"
#include "frame.hpp"
#include "tracking.hpp"
#include "simulator.hpp"

using namespace std;

int main( int argc, char** argv )
{
  if (argc != 3)
  {
    showUsage();
    return -1;
  }

  // load images with gray scale 
  cv::Mat target_image = cv::imread( argv[1] , 1); 
  string calibration_dir = argv[2];

  TrackingSystem system(target_image, calibration_dir);

  /* ?? Pangolin GUI button works only in main thread ?? */
  pangolin::CreateWindowAndBind("Menu",150,500);
  pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(150));
  pangolin::Var<bool> menu_drawcube("menu.Draw MMD",false,true);
  pangolin::Var<bool> menu_end("menu.End",false,false);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  cv::VideoCapture cap(1);
  if(!cap.isOpened())
  {
    cout << "Failed to open camera device" << endl;
    return -1;
  }

  // for safety
  for(int i = 0; i < 10; i++) {
    cv::Mat m;
    cap >> m;
  }
  
  while(1)
  {
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    cv::Mat image;
    cap >> image;

    if(image.empty()) {
      continue;
    }
  
    system.setParams(menu_drawcube, menu_end);
    system.setQueryImage(image);

    if(!system.run()) {
      cout << "==========>>> Killing the system ...";
      pangolin::Quit();
      system.shutdown();
      cout << "...killed." << endl;
      break;
    }

    pangolin::FinishFrame();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double duration = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout << "Operation Time : " << duration << "[sec]\r";
  }

  cout << "=============================>>> " << "The all system has been killed. ByeBye." << endl;
  return 0;
}

