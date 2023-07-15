#include "mapper.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/filters/radius_outlier_removal.h>  
//#include <pcl/filters/statistical_outlier_removal.h>

using namespace rgbd_tutor;

Mapper::PointCloud::Ptr Mapper::generatePointCloud( const RGBDFrame::Ptr &frame )
{
    semantic_motion_fuse(frame);

    PointCloud::Ptr tmp( new PointCloud() );
    if ( frame->pointcloud == nullptr )
    {
        // point cloud is null ptr
        frame->pointcloud = boost::make_shared<PointCloud>();
#pragma o