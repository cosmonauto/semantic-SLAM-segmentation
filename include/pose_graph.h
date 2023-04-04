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
  * The pose g