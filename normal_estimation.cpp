//
// Created by chli on 11.03.20.
//

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

pcl::Normal normal_estimation()
{

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Output has the PointNormal type in order to store the normals calculated by MLS


    // Use all neighbors in a sphere of radius 3cm
    //ne.setRadiusSearch (0.03);
    ne.setKSearch(20);

    // Compute the features
    ne.compute (*cloud_normals);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*


    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);
    //pcl::io::savePCDFile("/home/chli/cc_code2/deeplab/kitti_image/testing/normal_pcd/um_000015_2.pcd",*cloud_with_normals);

    return cloud_normals
}
