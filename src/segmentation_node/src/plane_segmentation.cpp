#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader cloud_reader;
    pcl::PCDWriter cloud_writer;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // read
    std::string path = "/home/dart/Projects/drone-vajraakhasa/src/segmentation_node/clouds/";
    cloud_reader.read(path+std::string("cloud.pcd"), *cloud);

    // planner segmentation
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.01); // satuannya meter
    plane_seg.setInputCloud(cloud);
    plane_seg.segment(*inliers, *coefficients);

    // Extracting Points
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers);
    extract_indices.setNegative(true);
    extract_indices.filter(*plane_seg_cloud);

    std::cout << (*plane_seg_cloud).height * plane_seg_cloud->width << std::endl;

    // write
    cloud_writer.write<pcl::PointXYZ>(path+std::string("non_plane_seg.pcd"), *plane_seg_cloud, false);

    return 0;
}