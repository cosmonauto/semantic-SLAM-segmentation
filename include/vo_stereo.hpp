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

 
// stereo-specific parameters (mandatory: base)
  struct parameters : public VisualOdometry::parameters
  {
    double  base;             // baseline (meters)
    int ransac_iters;         // number of RANSAC iterations
    double  inlier_threshold; // fundamental matrix inlier threshold
    bool    reweighting;      // lower border weights (more robust to calibration errors)
    parameters ()
    {
      base             = 1.0;
      ransac_iters     = 200;
      inlier_threshold = 1.1f;
      reweighting      = true;
    }
  };

  // constructor, takes as inpute a parameter structure
  VisualOdometryStereo (parameters param);
  
  // deconstructor
  ~VisualOdometryStereo ();
  
  // process a new images, push the images back to an internal ring buffer.
  // valid motion estimates are available after calling process for two times.
  // inputs: I1 ........ pointer to rectified left image (uint8, row-aligned)
  //         I2 ........ pointer to rectified right image (uint8, row-aligned)
  //         dims[0] ... width of I1 and I2 (both must be of same size)
  //         dims[1] ... height of