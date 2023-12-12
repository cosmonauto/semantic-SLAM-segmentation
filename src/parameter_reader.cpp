#include "rgbdframe.h"
#include "parameter_reader.h"

rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS rgbd_tutor::ParameterReader::getCamera() const
{
    static rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = this->getData<double>("camera.fx");
    camera.fy = this->getData<dou