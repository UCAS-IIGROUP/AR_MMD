#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <fstream>
#include <thread>

#include "./frame.hpp"
#include "./viewer.hpp"

class TrackingSystem {
  private: // variable
    Frame mTargetImage;
    Frame mQueryImage;
    cv::Ptr<cv::AKAZE> mExtractor;// = cv::AKAZE::create();
    // cv::Ptr<cv::ORB> mExtractor;// = cv::AKAZE::create();
    vector<cv::Mat> mvPoseLog;
    long int mCount;
    cv::Mat mExtractorMask;
    int mImageHeight, mImageWidth;
    cv::Mat mK;
    cv::Mat mDist;
    vector<cv::Point3f> mvCubePoints{8};
    vector<cv::Point3f> mvWorldCoordinate{4};
    vector<cv::Point2f> mvPatternCorners{4};
    /* index 
     * 0 1
     * 2 3
     */

    bool mbMarkerMode;
    cv::Ptr<cv::aruco::Dictionary> mpDictionary;// = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    bool mbDrawCube;
    bool mbEnd;

    std::thread* mpthread_viewer;
    viewer* mpViewer;

  public: // variable


  private: // function
    void findKeyPointsAndCalcDescriptors(
           cv::Mat& im, vector<cv::KeyPoint>& kpts, 
           cv::Mat& desc, bool mask_mode = false);

    void pushCurrentPose2Log(cv::Mat& pose);

    static void calcPose(const cv::Mat& H, cv::Mat& pose);

    int findMatches2Images( 
        vector<cv::Point2f>& target_points,
        vector<cv::Point2f>& query_points,
        vector<pair < int, int > >& pair_list);

    void estimateExtractorMask();
    // this write mEstimatedPatternMask
    bool checkIsInlierEnough(
        const cv::Mat& mask, 
        vector<cv::Point2f>& target_points, 
        vector<cv::Point2f>& query_points
        );
    void loadCamParams(string dir_name);

    void prepareCube(float scale = 10.f);
    void prepareWorldCoordinate(float scale = 1.f);
    void drawCube();
    void test();

    static void showCoordinate(
        cv::Mat im, 
        cv::Mat pose, 
        cv::Mat K,
        vector<cv::Point2f> corner_points,
        vector<cv::Point2f> target_points,
        vector<cv::Point2f> query_points,
        vector<cv::Point3f> world_coordinate_points
        );

  public: // function
    TrackingSystem(cv::Mat& target_image, string calibration_dir, bool marker_mode = false);
    ~TrackingSystem();
    cv::Mat getTargetImage();
    cv::Mat getQueryImage();
    void setQueryImage(cv::Mat& _image);
    void setParams(const bool draw_cube, const bool end);
    bool run();
    void shutdown();
};

