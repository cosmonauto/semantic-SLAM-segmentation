#ifndef COMMON_HEADERS_H
#define COMMON_HEADERS_H

/**
  * 各种可能用到的头文件，放在一起方便include
  */

// C++标准库
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <dirent.h>
#include <time.h>
#include <string>
using namespace std;

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "opencv2/imgproc/imgproc.hpp"

// boost
#include <boost/format.hpp>
#include <boost/timer.hpp>
#include <boost/lexical_cast.hpp>

// g2o
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#i