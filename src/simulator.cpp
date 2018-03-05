#include "./simulator.hpp"

using namespace std;

Simulator::Simulator(cv::Mat& image): mCanvasSize(image.size()*2)
{
  mmImage = image.clone();

	mfAlpha = 0.f; mfBeta = 0.f; mfGamma = 0.f;
	mfTransX = 0.f; mfTransY = 0.f; mfTransZ = 1.f;

  cv::Mat K = (cv::Mat_<float>(3,3) << 1000.f, 0.f, (float)image.cols/2.f,
                                       0.f, 1000.f, (float)image.rows/2.f,
                                       0.f, 0.f, 1.f);
  mKf = K.clone();
}

Simulator::Simulator(cv::Mat& image, cv::Size CanvasSize): mCanvasSize(CanvasSize)
{
  mmImage = image.clone();

	mfAlpha = 0.f; mfBeta = 0.f; mfGamma = 0.f;
	mfTransX = 0.f; mfTransY = 0.f; mfTransZ = 1.f;

  cv::Mat K = (cv::Mat_<float>(3,3) << 1000.f, 0.f, (float)image.cols/2.f,
                                       0.f, 1000.f, (float)image.rows/2.f,
                                       0.f, 0.f, 1.f);
  mKf = K.clone();
}

cv::Mat Simulator::GenerateWarpedImage(const int &key) {
  cv::Mat warped_image;

  UpdateParam(key);

  cv::Mat mat_rot = cv::Mat::zeros(3,3,CV_32FC1);
  cv::Mat mat_H = cv::Mat::zeros(3,3,CV_32FC1);

  cv::Mat mat_rot_x = (cv::Mat_<float>(3,3) << 1.f, 0.f, 0.f,
                                               0.f, cosf(mfAlpha),sinf(mfAlpha),
                                               0.f, -sinf(mfAlpha),cosf(mfAlpha));

	cv::Mat mat_rot_y = (cv::Mat_<float>(3,3) << cosf(mfBeta), 0.f, -sinf(mfBeta),
                                               0.f, 1.f, 0.f,
                                               sinf(mfBeta), 0.f, cosf(mfBeta));

	cv::Mat mat_rot_z = (cv::Mat_<float>(3,3) << cosf(mfGamma), sinf(mfGamma), 0.f,
                                              -sinf(mfGamma), cosf(mfGamma), 0.f,
                                              0.f, 0.f, 1.f);
	
	mat_rot = mat_rot_z * mat_rot_y * mat_rot_x;

  cv::Mat vec_t = (cv::Mat_<float>(3,1) << mfTransX, mfTransY, mfTransZ);
  
  mat_H.at<float>(0,0) = mat_rot.at<float>(0,0);
  mat_H.at<float>(1,0) = mat_rot.at<float>(1,0);
  mat_H.at<float>(2,0) = mat_rot.at<float>(2,0);
  mat_H.at<float>(0,1) = mat_rot.at<float>(0,1);
  mat_H.at<float>(1,1) = mat_rot.at<float>(1,1);
  mat_H.at<float>(2,1) = mat_rot.at<float>(2,1);
  mat_H.at<float>(0,2) = vec_t.at<float>(0,0);
  mat_H.at<float>(1,2) = vec_t.at<float>(1,0);
  mat_H.at<float>(2,2) = vec_t.at<float>(2,0);

	mat_H = mKf * mat_H.clone() * mKf.inv();
	mat_H = mat_H.clone()/(mat_H.clone()).at<float>(2,2);

	cv::warpPerspective( mmImage, warped_image, mat_H,
                       // mmImage.size(),
                       mCanvasSize,
                       cv::INTER_LINEAR,
                       cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );

  return warped_image;
}

void Simulator::UpdateParam(const int &key)
{
	if(key == 'x') mfAlpha += mfROT_PARAM/180.0*_PI;
	if(key == 'm') mfAlpha -= mfROT_PARAM/180.0*_PI;
	if(key == 'y') mfBeta += mfROT_PARAM/180.0*_PI;
	if(key == 'n') mfBeta -= mfROT_PARAM/180.0*_PI;
	if(key == 'z') mfGamma += mfROT_PARAM/180.0*_PI;
	if(key == 'b') mfGamma -= mfROT_PARAM/180.0*_PI;

	if(key == 't') mfTransZ += mfTRANS_PARAM;
	if(key == 's') mfTransZ -= mfTRANS_PARAM;
	if(key == 'l') mfTransX += mfTRANS_PARAM;
	if(key == 'h') mfTransX -= mfTRANS_PARAM;
	if(key == 'j') mfTransY += mfTRANS_PARAM;
	if(key == 'k') mfTransY -= mfTRANS_PARAM;

	/*----------------------------------------------------------------------------------
	key =	'x' :X軸に5[deg]回転
        'X'	:X軸に-5[deg]回転
        'y' :Y軸に5[deg]回転
        'Y'	:Y軸に-5[deg]回転
        'z' :Z軸に5[deg]回転
        'Z'	:Z軸に-5[deg]回転
        'u' :U方向に1ピクセル移動
        'U'	:U方向に-1ピクセル移動
        --- vim style ---
        'l' :V方向に1ピクセル移動
        'h'	:V方向に-1ピクセル移動
        'k' :Z方向に1ピクセル移動
        'j'	:Z方向に-1ピクセル移動
        'else' そのまま
	---------------------------------------------------------------------------------*/
}
