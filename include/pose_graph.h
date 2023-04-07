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
  * pose graph本身有一个优化线程，负责查找相近节点的边并进行优化。
  * 同时它有回环检测模块。检测到大型回环时进行全局优化。
  * 
  * 进行优化时，关键帧序列的位姿会被替换成优化后的值，因此参考帧会发生改变。而tracker存在漂移。
  * 为了使机器人得到全局准确的位姿，需要用pose graph优化后的结果校正tracker。
  */

namespace rgbd_tutor
{
using namespace rgbd_tutor;