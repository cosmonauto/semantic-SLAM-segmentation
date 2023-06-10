/*
IRTES-SET laboratory
Authors: You Li (liyou026@gmail.com)
Descirption: This a sample code of my PhD works
*/

#ifndef VO_H
#define VO_H

#include <quadmatcher.hpp>
#include "opencv2/core/core.hpp"

class VisualOdometry {

public:

  // camera parameters (all are mandatory / need to be supplied)
  struct calibration {  
    double f;  // focal length (in pixels)
    double cu; // principal point (u-coordinate)
    double cv; // principal point (v-coordinate)
    calibration () {
      f  = 1;
      cu = 0;
      cv = 0;
    }
  };
    
  // general parameters
  struct parameters {
    VisualOdometry::calibration calib;            // camera calibration parameters
  };

  // constructor
  VisualOdometry (parameters param);
  
  // deconstructor
  ~VisualOdometry ();


  // returns transformation from previous to current coordinates as a 4x4
  // homogeneous transformation matrix Tr_delta, with the following semantics:
  // p_t = Tr_delta * p_ {t-1} takes a 