#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <pcl/features/normal_3d.h>
#include <vector>
#include <pcl/point_types.h>
#include <algorithm>
#include <pcl/visualization/cloud_viewer.h>

int
main () {
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Load bun0.pcd -- should be available with the PCL archive in test
    pcl::io::loadPCDFile("/home/cc/Desktop/data/kitti_image/testing/filter_pcd/um_000031.pcd", *cloud);
    std::cout << cloud->points[0] << std::endl;

    // normal estimation
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.3);
    //ne.setKSearch(20);

    // Compute the features
    ne.compute (*cloud_normals);

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);

    // set the same pcl object to save the segmentation cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>());

    std::cout << cloud_normals->width << std::endl;
    std::cout << cloud_with_normals->points[0] << std::endl;
    std::cout << cloud_normals->points[0].normal_x << std::endl;
    std::vector <float> alpha;
    std::vector <float> beta;
    float radio_x;
    const double pi=3.1415926;
    float a[cloud_normals->points.size()];
    //radio = 90-atan(abs(cloud_normals->points[0].normal_z / cloud_normals->points[0].normal_x))*180/pi;
    //std::cout << radio << std::endl;

    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        radio_x = 90 - atan(abs(cloud_normals->points[i].normal_z / cloud_normals->points[i].normal_x))*180/pi;
        alpha.push_back(radio_x);
        std::cout <<"alpha ";
        std::cout << radio_x << std::endl;

        if (radio_x < 5)
        {
            cloud1->push_back(cloud->points[i]);
            std::cout << "radio_x " << radio_x << std::endl;
        }
        else if(radio_x < 10 )
        {
            cloud2->push_back(cloud->points[i]);
            std::cout << "radio_x with a big slope " << radio_x << std::endl;
        }
        else
        {
            cloud3->push_back(cloud->points[i]);
            std::cout << "radio_x with a big slope " << radio_x << std::endl;
        }


    }
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("/home/cc/Desktop/data/pcd/um_000031_r_flat1.pcd", *cloud1, false);
    writer.write<pcl::PointXYZ>("/home/cc/Desktop/data/pcd/um_000031_r_lessslope1.pcd", *cloud2, false);
    writer.write<pcl::PointXYZ>("/home/cc/Desktop/data/pcd/um_000031_r_bigslope1.pcd", *cloud3, false);



    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(cloud1);

    system("pause");

    /*
    auto maxPosition = max_element(alpha.begin(), alpha.end());
    auto minPosition = min_element(alpha.begin(), alpha.end());
    std::cout << *maxPosition << " at the postion of " << maxPosition - alpha.begin() <<std::endl;
    std::cout << *minPosition << " at the postion of " << minPosition - alpha.begin() <<std::endl;
    */


    return 0;
}

//beta = 90 - atan(abs(cloud_normals->points[i].normal_z / cloud_normals->points[i].normal_y))*180/pi;
//std::cout <<"beta ";
//std::cout << beta << std::endl;
