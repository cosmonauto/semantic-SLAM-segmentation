#include "rgbdframe.h"
#include "track.h"
#include "pose_graph.h"
#include "common_headers.h"
#include "mapper.h"
#include "readGTPose.h"
#include "vo_stereo.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace rgbd_tutor;

int main()
{

    ParameterReader	parameterReade