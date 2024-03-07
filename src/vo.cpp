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

bool VisualOdometry::updateMotion ()
{
  double time = (double)cv::getTickCount();

  // estimate motion
  vector<double> tr_delta = estimateMotion(quadmatches);
  
  // on failure
  if (tr_delta.size()!=6)
    return false;
  
  // set transformation matrix (previous to current frame)
  Tr_delta = transformationVectorToMatrix(tr_delta);

  time = ((double)cv::getTickCount() - time)/cv::getTickFrequency()*1000;
  //cout<<"The odometry estimation costs "<<time<<" ms"<<endl;

  // success
  return true;
}


cv::Mat VisualOdometry::transformationVectorToMatrix (std::vector<double> tr) {

  // extract parameters
  double rx = tr[0];
  double ry = tr[1];
  double rz = tr[2];
  double tx = tr[3];
  double ty = tr[4];
  double tz = tr[5];

  // precompute sine/cosine
  double sx = sin(rx);
  double cx = cos(rx);
  double sy = sin(ry);
  dou