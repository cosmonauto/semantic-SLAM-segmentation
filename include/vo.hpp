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
  // p_t = Tr_delta * p_ {t-1} takes a point in the camera coordinate system
  // at time t_1 and maps it to the camera coordinate system at time t.
  // note: getMotion() returns the last transformation
  inline cv::Mat getMotion() { return Tr_delta;}
  
  // returns the number of successfully matched points
  inline int getNumberOfMatches () { return quadmatches.size(); }
  
  // returns the number of inliers: num_inliers <= num_matched
  inline int getNumberOfInliers () { return inliers.size(); }
    
  // returns the indices of all inliers
  inline std::vector<int> getInlierIndices () { return inliers; }
    
public:

  std::vector<pmatch>  quadmatches;  // feature point matches
  std::vector<pmatch>  quadmatches_