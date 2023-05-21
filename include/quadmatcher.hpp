
/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#ifndef QUADMATCHER__HPP
#define QUADMATCHER__HPP

#include <iostream>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/video/tracking.hpp"

using namespace std;
using namespace cv;


enum { DET_FAST, DET_STAR, DET_ORB, DET_SIFT, DET_SURF, DET_GFTT,
       DET_STAR_ADAPT, DET_FAST_ADAPT, DET_FAST_GRID, DET_STAR_GRID,DET_GFTT_GRID};

enum { DES_SIFT, DES_SURF, DES_BRISK, DES_FREAK,DES_ORB};


//core struct storing quadmatching result
struct pmatch {
    float   u1p,v1p; // u,v-coordinates in previous left  image
    int32_t i1p;     // feature index (for tracking)
    float   u2p,v2p; // u,v-coordinates in previous right image
    int32_t i2p;     // feature index (for tracking)
    float   u1c,v1c; // u,v-coordinates in current  left  image
    int32_t i1c;     // feature index (for tracking)
    float   u2c,v2c; // u,v-coordinates in current  right image
    int32_t i2c;     // feature index (for tracking)
    short dis_c,dis_p;//disparity for the current and previous

    pmatch(){}
    pmatch(float u1p,float v1p,int32_t i1p,float u2p,float v2p,int32_t i2p,
            float u1c,float v1c,int32_t i1c,float u2c,float v2c,int32_t i2c):
            u1p(u1p),v1p(v1p),i1p(i1p),u2p(u2p),v2p(v2p),i2p(i2p),
            u1c(u1c),v1c(v1c),i1c(i1c),u2c(u2c),v2c(v2c),i2c(i2c) {}
  };

class QuadFeatureMatch
{
public: QuadFeatureMatch(){};

        QuadFeatureMatch(cv::Mat& img_lc_, cv::Mat& img_rc_,
                         cv::Mat& img_lp_, cv::Mat& img_rp_,
			 cv::Mat& img_s_rc_, cv::Mat& img_s_rp,
			 bool mode_track_);
