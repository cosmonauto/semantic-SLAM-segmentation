
/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#include "uvdisparity.hpp"
#include "stereo.h"
using namespace cv;
using namespace std;








USegmentPars::USegmentPars(int min_intense_, int min_disparity_raw_, int min_area_)
{
    min_intense = min_intense_;
    min_disparity_raw = min_disparity_raw_;
    min_area = min_area_;
}

//constructer
 UVDisparity::UVDisparity()
 {
     out_th_ = 6.0f;                    //outlier rejection threshold
     inlier_tolerance_ = 3;             //inlier tolerance threshold
     min_adjust_intense_ = 19;          //minum adjust intensity


     //init Kalman filter parameters
     pitch1_KF = new KalmanFilter(2,1,0);
     pitch1_KF->transitionMatrix = *(Mat_<float>(2, 2) << 1,0,0,1);
     cv::setIdentity(pitch1_KF->measurementMatrix);
     cv::setIdentity(pitch1_KF->processNoiseCov, Scalar::all(0.000005));
     cv::setIdentity(pitch1_KF->measurementNoiseCov, Scalar::all(0.001));
     cv::setIdentity(pitch1_KF->errorCovPost, Scalar::all(1));

     pitch2_KF = new KalmanFilter(2,1,0);
     pitch2_KF->transitionMatrix = *(Mat_<float>(2, 2) << 1,0,0,1);
     cv::setIdentity(pitch2_KF->measurementMatrix);
     cv::setIdentity(pitch2_KF->processNoiseCov, Scalar::all(0.000005));
     cv::setIdentity(pitch2_KF->measurementNoiseCov, Scalar::all(0.001));
     cv::setIdentity(pitch2_KF->errorCovPost, Scalar::all(1));

     pitch1_measure.create(1,1,CV_32F);
     pitch2_measure.create(1,1,CV_32F);


 }

 //deconstructor
 UVDisparity::~UVDisparity()
 {
     delete pitch1_KF,pitch2_KF;
 }


 /*Since the reconstructed 3D points sometimes not correspond the
  *triangulated coordinates (the reason is disparity map is smoothed
  *while the disparity between two matched points is not smoothed),
  *we re-calculate the 3D position, and filter out the inliers outsid
  *e the ROI
*/
void UVDisparity::filterInOut(const Mat &image, const Mat &roi_mask,const Mat &sgbm_roi,
                              VisualOdometryStereo& vo, const double pitch)
{
  /*calibration parameters*/
  double f = calib_.f;
  double cu = calib_.c_x;
  double cv = calib_.c_y;
  double base = calib_.b;

  double cos_p = cos(pitch);
  double sin_p = sin(pitch);
  int threshold = -3000;

  cv::Mat motion1 = vo.getMotion();
  cv::Mat show_in,show_out,xyz0,xyz1;
  
  cvtColor(image, show_in, CV_GRAY2BGR);
  cvtColor(image, show_out, CV_GRAY2BGR);

  vector<pmatch>::iterator it_in = vo.quadmatches_inlier.begin();
  //cout<<"the inliers: "<<vo.quadmatches_inlier.size()<<endl;


  for(; it_in!=vo.quadmatches_inlier.end(); )
  {
    /*current feature point position*/
    int uc = (*it_in).u1c;
    int vc = (*it_in).v1c;
  
    /*previous feature point position*/
    int up = (*it_in).u1p;
    int vp = (*it_in).v1p;
  
    if(roi_mask.at<uchar>(vc,uc) > 0)
    {
      double d = max(uc - (*it_in).u2c, 1.0f);
      double yc = (vc - cv)*base/d;
      double zc = f*base/d;

      yc = cos_p * yc + sin_p * zc;
  
       if(true)
      {
        cv::circle(show_in, cv::Point(uc,vc),2,cv::Scalar(0,0,255),2,8,0);
        cv::circle(show_in, cv::Point(up,vp),2,cv::Scalar(0,0,255),2,8,0);
        cv::line(show_in,  cv::Point(up,vp),  cv::Point(uc,vc), cv::Scalar(0,255,0),1,8,0);

        (*it_in).dis_c = sgbm_roi.at<short>(vc,uc);

        it_in++;
      }
       else
       {
         it_in = vo.quadmatches_inlier.erase(it_in);
       }
       
    }
    else
    {
      it_in = vo.quadmatches_inlier.erase(it_in);
    }
  }
  //cout<<"After the filter process, the number of inlier is: "<<vo.p_matched_inlier.size()<<endl;
  vector<pmatch>::iterator it_out = vo.quadmatches_outlier.begin();
 // cout<<"before the filter process, the number of outlier is: "<<vo.p_matched_outlier.size()<<endl;
  for(; it_out!=vo.quadmatches_outlier.end(); )
  {
    
    int uc = (*it_out).u1c;
    int vc = (*it_out).v1c;
    //int vc2 = (*it).v2c;

    int up = (*it_out).u1p;
    int vp = (*it_out).v1p;