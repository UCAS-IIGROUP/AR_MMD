#include "./tracking.hpp"

TrackingSystem::TrackingSystem(cv::Mat& target_image, string calibration_dir) {

  loadCamParams(calibration_dir);
  cv::undistort(target_image.clone(), target_image, mK, mDist);

  mExtractor = cv::AKAZE::create();

  mpDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  mbMarkerMode = checkMarker(target_image);

  cv::Mat gray;
  cv::Mat desc;
  vector<cv::KeyPoint> keypoints;
  cv::cvtColor(target_image, gray,CV_RGB2GRAY);
  findKeyPointsAndCalcDescriptors(gray, keypoints, desc);
  mTargetImage = Frame(target_image, desc, keypoints);
  mImageHeight = target_image.rows;
  mImageWidth = target_image.cols;

  mvPatternCorners[0] = cv::Point2f(-target_image.cols / 2.f, -target_image.rows / 2.f);
  mvPatternCorners[1] = cv::Point2f(target_image.cols/2.f, -target_image.rows / 2.f);
  mvPatternCorners[2] = cv::Point2f(-target_image.cols / 2.f, target_image.rows / 2.f);
  mvPatternCorners[3] = cv::Point2f(target_image.cols / 2.f, target_image.rows / 2.f);

  prepareWorldCoordinate(50.f); // you can set a scale

  mbDrawCube = false;
  mbEnd = false;

  mCount = 0;
  mpViewer = new viewer(target_image.cols, target_image.rows);
  mpthread_viewer = new std::thread(&viewer::Run, mpViewer);
  cout << "========>>> The system has launced now." << endl;
  return;
}

void TrackingSystem::setParams(const bool draw_cube, const bool end) {
  if(draw_cube != mbDrawCube or end != mbEnd) {
    mbDrawCube = draw_cube;
    mbEnd = end;
    mpViewer->SetMenuParams(mbDrawCube, mbEnd);
  }
}

bool TrackingSystem::checkMarker(cv::Mat& image) {
  std::vector<int> markerIDs;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  cv::aruco::detectMarkers(image, mpDictionary, markerCorners, markerIDs);
  if(markerIDs.size() == 1) {
    mTargetMarkerID = markerIDs[0];
    cout << "=========>>> Marker Mode = ON\n";
    return true;
  }
  else {
    mTargetMarkerID = -1;
    cout << "=========>>> Marker Mode = OFF\n";
    return false;
  }
}

// this is not used and probably incorrect
void TrackingSystem::calcPose(const cv::Mat& H, cv::Mat& pose)
{
  pose = cv::Mat::eye(3, 4, CV_64FC1); //3x4 matrix
  float norm1 = (float)norm(H.col(0)); 
  float norm2 = (float)norm(H.col(1));
  float tnorm = (norm1 + norm2) / 2.0f;

  cv::Mat v1 = H.col(0);

  cv::normalize(v1.clone(), v1); // Normalize the rotation
  v1.copyTo(pose.col(0));

  v1.release();
  v1 = H.col(1);

  cv::normalize(v1.clone(), v1);
  v1.copyTo(pose.col(1));

  v1.release();
  v1 = pose.col(0);
  cv::Mat v2 = pose.col(1);

  cv::Mat v3 = v1.cross(v2);  //Computes the cross-product of v1 and v2
  v3.copyTo(pose.col(2));      
  pose.col(3) = H.col(2) / tnorm; //vector t [R|t]

}

void TrackingSystem::loadCamParams(string dir_name) {
  mK = cv::Mat::eye(3, 3, CV_32FC1); 
  mDist = cv::Mat(4, 1, CV_32FC1); 
  std::ifstream ifs1(dir_name + "/k_param.txt");
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      string tmp;
      ifs1 >> tmp;
      mK.at<float>(i, j) = stof(tmp);
    }
  }
  std::ifstream ifs2(dir_name + "/d_param.txt");
  for(int i = 0; i < 4; i++) {
    string tmp;
    ifs2 >> tmp;
    mDist.at<float>(i) = stof(tmp);
  }
  cout << "========>>> Camera Pramas have been loaded." << endl;
}

void TrackingSystem::prepareWorldCoordinate(float scale) {
  mvWorldCoordinate[0] = cv::Point3f{0.f, 0.f, 0.f}; // origin 
  mvWorldCoordinate[1] = cv::Point3f{scale, 0.f, 0.f}; // x 
  mvWorldCoordinate[2] = cv::Point3f{0.f, scale, 0.f}; // y
  mvWorldCoordinate[3] = cv::Point3f{0.f, 0.f, -scale}; // z
  return;
}

void TrackingSystem::findKeyPointsAndCalcDescriptors(
    cv::Mat& image, vector<cv::KeyPoint>& keypoints, 
    cv::Mat& desc, bool mask_mode
    )
{
  if(mask_mode) {
    //double t1 = cv::getTickCount();
    mExtractor -> detectAndCompute( image, mExtractorMask, keypoints, desc );
    //double t2 = cv::getTickCount();
    //double tdet = 1000.0*(t2-t1) / cv::getTickFrequency();
  }
  else {
    //double t1 = cv::getTickCount();
    mExtractor -> detectAndCompute( image, cv::noArray(), keypoints, desc);
    //double t2 = cv::getTickCount();
    //double tdet = 1000.0*(t2-t1) / cv::getTickFrequency();
  }

  return;
}

cv::Mat TrackingSystem::getTargetImage() {
  return mTargetImage.getImage();
}

cv::Mat TrackingSystem::getQueryImage() {
  return mQueryImage.getImage();
}

void TrackingSystem::setQueryImage(cv::Mat& _image) 
{
  cv::Mat desc;
  cv::Mat gray;
  vector<cv::KeyPoint> keypoints;
  cv::undistort(_image.clone(), _image, mK, mDist);
    cv::cvtColor(_image, gray,CV_RGB2GRAY);
    findKeyPointsAndCalcDescriptors(gray, keypoints, desc, !mExtractorMask.empty());
  mQueryImage = Frame(_image, desc, keypoints);
}

void TrackingSystem::pushCurrentPose2Log(cv::Mat& pose) 
{
  mvPoseLog.push_back(pose);
  if(mvPoseLog.size() > 2) 
  {
    mvPoseLog.erase(mvPoseLog.begin()); // it keeps the length 2
    assert(mvPoseLog.size() < 3);
  }
}

void TrackingSystem::estimateExtractorMask() 
{
  cv::Mat last_pose = mvPoseLog[1];
  cv::Mat K;
  mK.convertTo(K, CV_64FC1);

  vector<cv::Point> estimatedPatternCorners;
  for(int i = 0; i < 4; i++) {
    cv::Mat p3d_homo = (cv::Mat_<double>(4,1) << mvPatternCorners[i].x,
                                                 mvPatternCorners[i].y,
                                                 0.d,
                                                 1.d);
    cv::Mat tmp = K * last_pose * p3d_homo; // 3x3 * 3x4 * 4x1
    estimatedPatternCorners.push_back(cv::Point((int)(tmp.at<double>(0)/tmp.at<double>(2)), (int)(tmp.at<double>(1)/tmp.at<double>(2))));
  }


  cv::Point points[] = {
    estimatedPatternCorners[0],
    estimatedPatternCorners[1],
    estimatedPatternCorners[3],
    estimatedPatternCorners[2]
  };

  cv::Mat mask(mImageHeight, mImageWidth, CV_8UC1, cv::Scalar(0));
  cv::fillConvexPoly(mask, points, 4, cv::Scalar(255));
  dilate(mask, mExtractorMask, cv::noArray(), cv::Point(-1, -1), 2, 1, 1);
  mExtractorMask = mask.clone();
}

bool TrackingSystem::checkIsInlierEnough(const cv::Mat& mask)
{
  return mask.rows>10;
}

// the main of system
bool TrackingSystem::run()
{
  if(mbEnd) {
   return false; 
  }

  vector<cv::Point2f> target_points, query_points;
  cv::Mat image_viewer = mQueryImage.getImage().clone();

  if(mbMarkerMode) {
    
  }
  else {
    if(mCount > 5) {
      // using this mask, it deicdes area where extractor calculates key points in stable sequences
      estimateExtractorMask(); 
    }

    vector< pair<int,int> > pair_list;
    if(findMatches2Images(target_points, query_points, pair_list)) {

      vector<cv::Point3f> object_points;
      for(auto p : target_points) {
        object_points.push_back(cv::Point3f(p.x - mImageWidth/2, p.y - mImageHeight/2, 0.f));
      }
      vector<cv::Point2f> image_points = query_points;
      cv::Mat rvec, tvec;
      cv::Mat mask;
      cv::Mat pose = cv::Mat::eye(3, 4, CV_64FC1);
      if(mvPoseLog.size() == 2) {
        rvec = mvPoseLog[1].rowRange(0,3).colRange(0,3);
        tvec = mvPoseLog[1].rowRange(0,3).col(3);
        cv::Rodrigues(rvec.clone(), rvec);
        cv::solvePnPRansac(object_points, image_points, mK, cv::noArray(), rvec, tvec, true, 200, 1.0, 0.999, mask);
      }
      else
        cv::solvePnPRansac(object_points, image_points, mK, cv::noArray(), rvec, tvec, false, 200, 3.0, 0.99, mask);

      assert(pose.type() == rvec.type() and pose.type() == tvec.type()); // 64
      cv::Rodrigues(rvec.clone(), rvec);
      rvec.copyTo(pose.rowRange(0,3).colRange(0,3));
      tvec.copyTo(pose.rowRange(0,3).col(3));

      if(checkIsInlierEnough(mask)) {
        // this thread function shows the world coordinates on a captured image
        if(false) {
          thread viewer_th( TrackingSystem::showCoordinate, 
                            mQueryImage.getImage(), 
                            pose, 
                            mK, 
                            mvWorldCoordinate
                          );
          viewer_th.detach();
      }
        
        mpViewer->SetCamPose(pose);
        pushCurrentPose2Log(pose);
      }
      else {
        // the system has lost
        // it needs to clear camera pose history 
        mvPoseLog.clear();
        mCount = 0;
        mExtractorMask.release();
      }
      mCount++;
    }
    else {
      // the system has lost
      // it needs to clear camera pose history 
      mvPoseLog.clear();
      mCount = 0;
      mExtractorMask.release();
    }
  }
  mpViewer->SetImage(image_viewer);
  image_viewer.release();
  return true;
}

//viewer thread main function 2nd
void TrackingSystem::showCoordinate( 
                      cv::Mat im, 
                      cv::Mat pose, 
                      cv::Mat K,
                      vector<cv::Point3f> world_coordinate_points
                      ) 
{
  K.convertTo(K, CV_64FC1);

  vector<cv::Point2d> coordinate_points_on_image;
  for(int i = 0; i < 4; i++) {
    cv::Mat p3d_homo = (cv::Mat_<double>(4,1) << world_coordinate_points[i].x,
                                                 world_coordinate_points[i].y,
                                                 world_coordinate_points[i].z,
                                                 1.d);
    cv::Mat tmp = K * pose * p3d_homo; // 3x3 * 3x4 * 4x1
    coordinate_points_on_image.push_back(cv::Point2d(tmp.at<double>(0)/tmp.at<double>(2), tmp.at<double>(1)/tmp.at<double>(2)));
    // cout << cv::Point2d(tmp.at<double>(0)/tmp.at<double>(2), tmp.at<double>(1)/tmp.at<double>(2)) << endl;
  }


  for(int i = 1; i < 4; i++) {
    cv::line(im, coordinate_points_on_image[0],
                 coordinate_points_on_image[i],
                 CV_RGB(255 * (i==1), 255 * (i==2), 255 * (i==3)),
                 1, 1, 0);
  }

  cv::imshow("pose test", im);
  cv::waitKey(1);

  return;
}

int TrackingSystem::findMatches2Images( 
      vector<cv::Point2f>& target_points,
      vector<cv::Point2f>& query_points,
      vector<pair <int, int> >& pair_list)
{
  vector<cv::KeyPoint> kp1 = mTargetImage.getKeyPoints();
  vector<cv::KeyPoint> kp2 = mQueryImage.getKeyPoints();
  cv::Mat desp1 = mTargetImage.getDescriptors();;
  cv::Mat desp2 = mQueryImage.getDescriptors();;
  cv::Mat img1 = mTargetImage.getImage();
  cv::Mat img2 = mQueryImage.getImage();

  // cv::BFMatcher* matcher = new cv::BFMatcher(cv::NORM_L2, false); 
  cv::BFMatcher* matcher = new cv::BFMatcher(cv::NORM_HAMMING, false); 
  // cross check flag set to false
  // because i do cross-ratio-test match
  vector< vector<cv::DMatch> > matches_2nn_12, matches_2nn_21;
  matcher->knnMatch( desp1, desp2, matches_2nn_12, 2 );
  matcher->knnMatch( desp2, desp1, matches_2nn_21, 2 );
  const double ratio = 0.7;

  if(matches_2nn_21.size() > 5 and matches_2nn_12.size() > 5) 
  {
    vector<int> matches_12(kp1.size(), -1);
    vector<int> matches_21(kp2.size(), -1);
    for(size_t i = 0; i < matches_2nn_12.size(); i++) { // i is queryIdx
      if( matches_2nn_12[i][0].distance/matches_2nn_12[i][1].distance < ratio 
          and 
          matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].distance 
            / matches_2nn_21[matches_2nn_12[i][0].trainIdx][1].distance < ratio ) 
      {
        if(matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].trainIdx 
              == matches_2nn_12[i][0].queryIdx) 
        {
          pair<int, int> p 
          { matches_2nn_12[i][0].queryIdx,
            matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].queryIdx};
            pair_list.push_back(p);

            target_points.push_back(kp1[matches_2nn_12[i][0].queryIdx].pt);
            query_points.push_back(
                kp2[matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].queryIdx].pt
                );
          }
        }
      }
    }

  if(false) {
    cv::Mat src;
    cv::hconcat(img1, img2, src);
    if(pair_list.size() > 5) 
    {
      for(auto p : pair_list) {
        cv::line(src, kp1[p.first].pt,
                  cv::Point2f(kp2[p.second].pt.x + img1.cols, kp2[p.second].pt.y),
                  1, 1, 0);
      }
    }
    cv::namedWindow("matching" , cv::WINDOW_NORMAL);
    cv::imshow("matching", src);
    cv::waitKey(1);
  }

  return pair_list.size() > 8;
}

void TrackingSystem::shutdown() {

}

TrackingSystem::~TrackingSystem() {

}
