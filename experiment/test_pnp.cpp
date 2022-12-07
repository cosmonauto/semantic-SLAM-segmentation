#include "pnp.h"
#include "common_headers.h"
#include "readGTPose.h"

using namespace std;
using namespace rgbd_tutor;

int main()
{
    cout<<"running test pnp"<<endl;
    ParameterReader para;
    FrameReader frameReader( para );
    OrbFeature  orb(para);
    PnPSolver   pnp(para, orb);

    RGBDFrame::Ptr refFrame = frameReader.next();
    orb.detectFeatures( refFrame );
    Eigen::Isometry3d   speed = Eigen::Isometry3d::Identity();

    // plot
    string pose_file = para.getData<string>("gtpose_source");
    std::cout << "Sequence: " << pose_file << std::endl;
    PoseReader poseReader(pose_file);
    cv::Mat poseMap(1500,1500,CV_8UC3,cv::Scalar(0,0,0));
    int n = 0;
    while (1)
    {
        cout<<"*************************************"<<endl;
        boost::timer timer;
        RGBDFrame::Ptr currFrame = frameReader.next();

        if ( currFrame == nullptr )
        {
            break;
        }
        currFrame->T_f_w = speed * refFrame->T_f_w ;
        orb.detectFeatures( currFrame );

        PNP_INFORMATION info;
        bool result = pnp.solvePnPLazy( refFrame, currFrame, info, true );

        if ( result == false )
        {
            cout<<"pnp failed"<<endl;
            refFrame = currFrame;
            cv::waitKey(1);
        }
        else
        {
            currFrame->T_f_w = info.T * refFrame->T_f_w;
            cout<<"result.T="<<endl;
            cout<<info.T.matrix()<<endl;
            cout<<"current = "<< endl << currFrame->T_f_w.matrix() << endl;
            speed = info.T;
            refFra