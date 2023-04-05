#ifndef POSE_GRAPH_H
#define POSE_GRAPH_H
#include "common_headers.h"
#include "parameter_reader.h"
#include "rgbdframe.h"
#include "looper.h"
#include "pnp.h"
#include "orb.h"
#include "track.h"

#include <thread>
#include <mutex>
#include <functional>
#include <map>
#include <condition_variable>

/**
  * The pose graph performs global optimization.
  * 在tracker跟踪当前帧currentFrame，返回一个粗略的位姿时，会把这个帧尝试放入pose graph中.
  * pose graph将它与自己的refframe比较，得到一个相对位移估计。当这个估计大于给定阈值时，将该帧作为新的frame插入到pose graph中。
  * pose graph本身有一个优化线程，负责查找相