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
    cv::Mat poseMap(1500,1500,CV_8UC3,cv::Scalar(0,0,0));
    int n = 0;
    while ( RGBDFrame::Ptr frame = frameReader.next() )
    {
        cout<<"*************************************"<<endl;
        cout<<"tracking frame "<<frame->id<<endl;
        boost::timer    timer;
        Eigen::Isometry3d T = tracker.updateFrame( frame );
        cout<<BOLDBLUE"current T="<<endl<<T.matrix()<<RESET<<endl;
        cv::imshow( "image", frame->rgb );
        if (tracker.getState() == Tracker::LOST)
        {
            cout<<"The tracker has lost"<<endl;
            cv::waitKey(0);
        }
        else
        {
            cv::waitKey(1);
        }
        cout<<"time cost = "<<timer.elapsed()<<endl