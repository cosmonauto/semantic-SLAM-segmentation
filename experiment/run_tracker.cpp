#include "common_headers.h"
#include "rgbdframe.h"
#include "track.h"
#include "readGTPose.h"
#include "vo_stereo.hpp"

using namespace std;
using namespace rgbd_tutor;

int main()
{
    cout<<"running tracker..."<<endl;
    ParameterReader para;
    FrameReader frameReader( para );

    VisualOdometryStereo::parameters voparam; 
    double f = para.getData<double>("camera.fx");
    double c_u = para.getData<double>("camera.cx");
    double c_v = para.getData<double>("camera.cy");
    double base = para.getData<double>("camera.baseline");
    double inlier_threshold = para.getData<double>("inlier_threshold");
    voparam.calib.f  = f;      voparam.calib.cu = c_u;
    voparam.calib.cv = c_v;    voparam.base     = base;	
    voparam.inlier_threshold = inlier_threshold;
    Tracker tracker(para, voparam);
    // plot
    string pose_file = para.getData<string>("gtpose_source");
    PoseReader poseReader(pose_file);
    cv::Mat poseMap(1500,1500,CV_8UC3,cv::Scalar(