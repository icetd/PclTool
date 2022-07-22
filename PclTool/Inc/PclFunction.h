#ifndef _PCL_FUNCTION_H
#define _PCL_FUNCTION_H

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>

/* Boost */
#include <boost/math/special_functions/round.hpp>

/* filter voxel*/
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

/* volume */
#include <pcl/io/vtk_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/mouse_event.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>


typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef struct area_message {
	float S_;
	float h_;
	float volume_;
}aream;

PointCloudT::Ptr PclFilterVoxel(PointCloudT::Ptr cloud_in, float leaf_size);
PointCloudT::Ptr PclFilterAvoxel(PointCloudT::Ptr cloud_in, float leaf_size);
PointCloudT::Ptr PclFilterUniform(PointCloudT::Ptr cloud_in, float leaf_size);
PointCloudT::Ptr PclFilterRandom(PointCloudT::Ptr cloud_in, int num);
PointCloudT::Ptr PclFilterGridmin(PointCloudT::Ptr cloud_in, float resolutin);
PointCloudT::Ptr PclFilterStatistical(PointCloudT::Ptr cloud_in, double stddev_mult, int nr_k);
PointCloudT::Ptr PclFilterRadius(PointCloudT::Ptr cloud_in, double r_, int min_pts);

void getScreenPos(double* displayPos, double* world_point, pcl::visualization::PCLVisualizer::Ptr viewer);
PointCloudT::Ptr ProjectInliers(PointCloudT::Ptr cloud_in, PointCloudT::Ptr clicked_points_3d, pcl::visualization::PCLVisualizer::Ptr viewer, void* args);
pcl::PolygonMesh::Ptr PclGreedyTriangulation(const pcl::PointCloud<PointT>::Ptr  cloud_in);
inline void signelVloume(PointT p1, PointT p2, PointT p3, float reference_plane_h, aream& areaMessage);
void volumeOfMesh(pcl::PolygonMesh mesh, float reference_plane_h, aream& areaMessage);

#endif

