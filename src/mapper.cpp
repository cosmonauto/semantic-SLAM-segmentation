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
#pragma omp parallel for
        for ( int m=0; m<frame->depth.rows; m+=1 )
        {
	        uchar* motion_ptr = moving_mask.ptr<uchar>(m);
            for ( int n=0; n<frame->depth.cols; n+=1 )
            {
                ushort d = frame->depth.ptr<ushort>(m)[n];
                if (d == 0)
                    continue;  //深度为0，不考虑
                if (d > max_distance * frame->camera.scale)
                    continue; //距离较远的点，不考虑
	        	if (motion_ptr[n] == 255)
		            continue; //移动物体的点，不考虑

                PointT p;
                cv::Point3f p_cv = frame->project2dTo3d(n, m);
                p.b = frame->semantic.ptr<uchar>(m)[n*3];
                p.g = frame->semantic.ptr<uchar>(m)[n*3+1];
                p.r = frame->semantic.ptr<uchar>(m)[n*3+2];

                if (
                        (p.b==128 && p.g==128 && p.r==128) || //sky
                        (p.b==128 && p.g==192 && p.r==192) || // pole
                       // (p.b==0   && p.g==69 && p.r==255)  || // Road_marking
                       // (p.b==128 && p.g==128 && p.r==192) || // SignSymbol
                       // (p.b==128 && p.g==64 && p.r==64)   || // fence
                       // (p.b==0   && p.g==64 && p.r==64)   || // pedestrian
                        (p.b==192 && p.g==128 && p.r==0)  //||    //cyclist

                        //(p.b==128 && p.g==0 && p.r==64)    // car
                      //(p.b==128 && p.g==64 && p.r==128) || // road
                      //(p.b==222 && p.g==40 && p.r==60)  || // Pavement
                      //(p.b==0   && p.g==128 && p.r==128)|| // Tree
                      //(p.b==0   && p.g==0 && p.r==128)  || //building
                    ) continue;

                p.x = p_cv.x;
                p.y = p_cv.y;
                p.z = p_cv.z;

                /////////////////////////////////////
                PointT point_result;//这里是投影点的空间坐标
                point_result.b = frame->result.ptr<uchar>(m)[n*3];
                point_result.g = frame->result.ptr<uchar>(m)[n*3+1];
                point_result.r = frame->result.ptr<uchar>(m)[n*3+2];
                point_result.x = p_cv.x;
                point_result.y = p_cv.y;
                point_result.z = p_cv.z;

                /////////////////////////////////////
                /////////////////////////////////////
                PointT point_img;//这里是投影点的空间坐标
                point_img.b = frame->rgb.ptr<uchar>(m)[n*3];
                point_img.g = frame->rgb.ptr<uchar>(m)[n*3+1];
                point_img.r = frame->rgb.ptr<uchar>(m)[n*3+2];
                point_img.x = p_cv.x;
                point_img.y = p_cv.y;
                point_img.z = p_cv.z;

                /////////////////////////////////////

                //frame->pointcloud->points.push_back( p );
                //frame->pointcloud->points.push_back(point_result);
                frame->pointcloud->points.push_back(point_img);
            }
        }
    }

    //Eigen::Isometry3d T = frame->getTransform().inverse();
    Eigen::Isometry3d T = frame->getTransform();
    pcl::transformPointCloud( *frame->pointcloud, *tmp, T.matrix());
    tmp->is_dense = false;
    return tmp;
}

void Mapper::viewer()
{