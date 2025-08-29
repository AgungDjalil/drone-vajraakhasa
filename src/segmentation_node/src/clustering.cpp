#include <iostream>
#include <filesystem>

#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

void cloud_save(const std::string& filename, pcl::PointCloud<pcl::PointXYZ> cloud)
{
    std::string path = "/home/dart/Projects/drone-vajraakhasa/src/segmentation_node/clouds/";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>(path+std::string(filename), cloud, false);
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader cloud_reader;

    std::string path = "/home/dart/Projects/drone-vajraakhasa/src/segmentation_node/clouds/";
    cloud_reader.read(path+std::string("non_plane_seg.pcd"), *cloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    voxel_filter.filter(*voxel_cloud);

    // normals computation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normals_estimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> cylinder_segmentator;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZ> extract_cylinder;
    pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cylinder (new pcl::PointCloud<pcl::PointXYZ>);

    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(voxel_cloud);
    normals_estimator.setKSearch(50);
    normals_estimator.compute(*cloud_normals);

    // parameter for segmentation
    cylinder_segmentator.setOptimizeCoefficients(true);
    cylinder_segmentator.setModelType(pcl::SACMODEL_CYLINDER);
    cylinder_segmentator.setMethodType(pcl::SAC_RANSAC);
    cylinder_segmentator.setNormalDistanceWeight(0.5);
    cylinder_segmentator.setMaxIterations(10000);
    cylinder_segmentator.setDistanceThreshold(0.05);
    cylinder_segmentator.setRadiusLimits(0.1, 0.4);

    cylinder_segmentator.setInputCloud(voxel_cloud);
    cylinder_segmentator.setInputNormals(cloud_normals);
    cylinder_segmentator.segment(*inliers, *coefficients);

    // extracting indices
    extract_cylinder.setInputCloud(voxel_cloud);
    extract_cylinder.setIndices(inliers);
    extract_cylinder.setNegative(false);
    extract_cylinder.filter(*extracted_cylinder);

    // save cloud
    cloud_save("cylinder.pcd", *extracted_cylinder);

    return 0;
}