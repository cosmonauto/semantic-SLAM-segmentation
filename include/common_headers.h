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
#include "opencv2/imgp