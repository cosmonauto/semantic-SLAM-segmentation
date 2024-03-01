/*
IRTES-SET laboratory
Authors: You Li (liyou026@gmail.com)
Descirption: This a sample code of my PhD works
*/

#include "vo.hpp"
#include <cmath>

using namespace std;

VisualOdometry::VisualOdometry (parameters param) : param(param) {
  J         = 0;
  p_observe = 0;
  p_predict = 0;
  Tr_delta = cv::Mat::eye(4,4,CV_64F);
  srand(0);
}

VisualOdometry::~VisualOdometry () {
}

bool VisualOdometry::u