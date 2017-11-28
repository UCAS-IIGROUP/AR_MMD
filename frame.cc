#include "./frame.hpp"

Frame::Frame(cv::Mat& _image, cv::Mat& _desc, vector<cv::KeyPoint>& _kpts) {
  mmImage = _image.clone();
  mmDescriptor = _desc.clone();
  mvKeypoints = _kpts;
  for(auto k : _kpts)
    mvKeyObjPoints.push_back(cv::Point3f(k.pt.x, k.pt.y, 0.f));
  return;
}

Frame::Frame( ) {
  return;
}


cv::Mat Frame::getImage()
{
  return mmImage.clone();
}

cv::Mat Frame::getDescriptors()
{
  return mmDescriptor.clone();
}

vector<cv::KeyPoint> Frame::getKeyPoints()
{
  return mvKeypoints;
}
