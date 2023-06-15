/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#ifndef VO_STEREO_H
#define VO_STEREO_H

#include "vo.hpp"
#include <algorithm>
#include "opencv2/core/core.hpp"
#include <quadmatcher.hpp>
using namespace std;

class VisualOdometryStereo : public VisualOdometry
{

public:

 
// stereo-specific parameters (mandatory: