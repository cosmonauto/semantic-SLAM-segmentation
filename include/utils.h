#ifndef UTILS_H
#define UTILS_H

#include "common_headers.h"

namespace rgbd_tutor
{
	struct CAMERA_INTRINSIC_PARAMETERS
	{
	    // 标准内参
	    double cx=0, cy=0, fx=0, fy=0, scale=0;
	