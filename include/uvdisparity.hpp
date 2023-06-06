
/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#ifndef UVDisparity__HPP
#define UVDisparity__HPP

#include <iostream>
#include "stereo.h"
#include "vo_stereo.hpp"

using namespace std;

//parameters in segmentation of U-disparity image
struct USegmentPars
{
  USegmentPars():min_intense(32), min_disparity_raw(64), min_area(40){};

  USegmentPars(int min_intense_, int min_disparity_raw_, int min_area_);

  inline USegmentPars& operator=(const USegmentPars& t)
    {
      min_intense = t.min_intense;
      min_disparity_raw = t.min_disparity_raw;
      min_area = t.min_area;
      return *this;
    }

  int min_intense;       // minum gray intensity for starting segmentation
  int min_disparity_raw; //minmm raw disparity for starting segmentation
  int min_area;          //minum area required for candidate mask
};


// intergrate a bundle of methods in U-V disparity image understanding
class UVDisparity
{
  public:
    //constructor and deconstructor
    UVDisparity();
    ~UVDisparity();

    // initialization functions
    inline void SetCalibPars(CalibPars& calib_par)
    {
        calib_ = calib_par;
    }
    inline void SetROI3D(ROI3D& roi_3d)
    {
        roi_ = roi_3d;
    }
    inline void SetUSegmentPars(int min_intense, int min_disparity_raw, int min_area)
    {
        this->u_segment_par_.min_intense = min_intense;
        this->u_segment_par_.min_disparity_raw = min_disparity_raw;
        this->u_segment_par_.min_area = min_area;
    }
    inline void SetOutThreshold(double out_th)
    {
        out_th_ = out_th;
    }  // set outlier threshold

    inline void SetInlierTolerance(int inlier_tolerance)
    {
        inlier_tolerance_ = inlier_tolerance;
    } //set inlier tolerance

    inline void SetMinAdjustIntense(int min_adjust_intense)
    {
        min_adjust_intense_= min_adjust_intense;
    } //set minimum adjust intensity for segmentation



    /**UV disparity segmentation:
    return value: the segmentation result
    **/
   cv:: Mat Process(cv::Mat& img_L, cv::Mat& disp_sgbm,           //input image and disparity map
                    VisualOdometryStereo& vo,     //input visual odoemtry(inlier&outlier),motion
                    cv::Mat& xyz, cv::Mat& roi_mask, Mat &ground_mask,
                    double& pitch1, double& pitch2);
private: