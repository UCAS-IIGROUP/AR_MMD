#include "./simulator.hpp"

using namespace std;

Simulator::Simulator(cv::Mat& image) {
  m_image = image.clone();

	mf_alpha = 0.f; mf_beta = 0.f; mf_gamma = 0.f;
	mf_trans_u = 0.f; mf_trans_v = 0.f; mf_trans_z = 1.f;

  cv::Mat K = (cv::Mat_<float>(3,3) << 1000.f, 0.f, (float)image.cols/2.f,
                                      0.f, 1000.f, (float)image.rows/2.f,
                                      0.f, 0.f, 1.f);
  mKf = K.clone();
}

cv::Mat Simulator::GenerateWarpedImage(const int &key) {
  cv::Mat dst_image;

  UpdateParam(key);

  cv::Mat mat_rot = cv::Mat::zeros(3,3,CV_32FC1);
  cv::Mat mat_H = cv::Mat::zeros(3,3,CV_32FC1);
  cv::Mat mat(3,3, CV_32FC1);

  cv::Mat mat_rot_x = (cv::Mat_<float>(3,3) << 1.f, 0.f, 0.f,
                                              0.f, cosf(mf_alpha),sinf(mf_alpha),
                                              0.f, -sinf(mf_alpha),cosf(mf_alpha));

	cv::Mat mat_rot_y = (cv::Mat_<float>(3,3) << cosf(mf_beta), 0.f, -sinf(mf_beta),
                                              0.f, 1.f, 0.f,
                                              sinf(mf_beta), 0.f, cosf(mf_beta));

	cv::Mat mat_rot_z = (cv::Mat_<float>(3,3) << cosf(mf_gamma), sinf(mf_gamma), 0.f,
                                              -sinf(mf_gamma), cosf(mf_gamma), 0.f,
                                               0.f, 0.f, 1.f);
	
	mat_rot = mat_rot_z * mat_rot_y * mat_rot_x;

  cv::Mat vec_t = (cv::Mat_<float>(3,1) << mf_trans_u, mf_trans_v, mf_trans_z);
  
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

	cv::warpPerspective( m_image, dst_image, mat_H,
                       m_image.size(),
                       cv::INTER_LINEAR,
                       cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );

  return dst_image;
}

void Simulator::UpdateParam(const int &key)
{
	if(key == 'x') mf_alpha += ROT_PARAM/180.0*_PI;
	if(key == 'X') mf_alpha -= ROT_PARAM/180.0*_PI;
	if(key == 'y') mf_beta += ROT_PARAM/180.0*_PI;
	if(key == 'Y') mf_beta -= ROT_PARAM/180.0*_PI;
	if(key == 'z') mf_gamma += ROT_PARAM/180.0*_PI;
	if(key == 'Z') mf_gamma -= ROT_PARAM/180.0*_PI;

	if(key == 't') mf_trans_z += TRANS_PARAM;
	if(key == 'T') mf_trans_z -= TRANS_PARAM;
	if(key == 'l') mf_trans_u += TRANS_PARAM;
	if(key == 'h') mf_trans_u -= TRANS_PARAM;
	if(key == 'j') mf_trans_v += TRANS_PARAM;
	if(key == 'k') mf_trans_v -= TRANS_PARAM;

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
