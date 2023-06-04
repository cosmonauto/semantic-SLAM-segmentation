
#ifndef FRAME_H
#define FRAME_H

#include "segnet.h"
#include "common_headers.h"
#include "parameter_reader.h"
#include "feature.h"
#include "utils.h"

#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <mutex>

/**
 * RGBDFrame 
 * 该类记录了每个帧的信息。帧是slam的基本单位。
 * 本身可以看作一个struct。
 * 由于Pose可能被若干线程同时访问，需要加锁。