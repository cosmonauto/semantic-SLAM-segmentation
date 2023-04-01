#ifndef ORB_H
#define ORB_H

#include "common_headers.h"
#include "rgbdframe.h"
#include "Thirdparty/orbslam_modified/include/ORBextractor.h"

/**
 * OrbFeature
 * 对orbslam2的特征部分进行一次封装，具有提取、匹配特征的功能
 */

namespace rgbd_tutor
{

class OrbFeature
{
public:
    OrbFeature( const rgbd_tutor::ParameterReader& para )
    {
        int features  = para.getData<int>("orb_features");
        float   scale = para.getData<float>("orb_scale");
        int     level = para.getData<int>("orb_levels");
        int     ini   = para.getData<int>("orb_iniThFAST");
        int     min   = 