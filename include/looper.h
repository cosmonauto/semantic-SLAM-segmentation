#ifndef LOOPER_H
#define LOOPER_H

/**
  * Looper.h
  * The loop closure detector based on dbow2;
  */

#include "common_headers.h"
#include "rgbdframe.h"
#include "converter.h"

#include <opencv2/opencv.hpp>

#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace rgbd_tutor
{
using namespace rgbd_tutor;

class Looper
{
public:

    Loop