#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/boost.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include "std_msgs/String.h"
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/octree_pointcloud_changedetector.h>
// #include <pcl/point_cloud.h>
#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/octree/impl/octree_search.hpp>
// #include <iostream>
// #include <vector>
// #include <ctime>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_test");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_union (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2::Ptr inputCloudPCL (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr outputCloudPCL (new pcl::PCLPointCloud2());

    // Fill in the CloudIn data
    cloud_in->width    = 5;
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
        << std::endl;


    for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
        cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
        cloud_in->points[i].z << std::endl;
    
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        cloud_out->points[i].x = cloud_in->points[i].x + 3.0f;
        cloud_out->points[i].y = cloud_in->points[i].y + 3.0f;
        cloud_out->points[i].z = cloud_in->points[i].z + 3.0f;
    }

    cloud_out->width    = 7;
    cloud_out->height   = 1;
    cloud_out->is_dense = false;
    cloud_out->points.resize (cloud_out->width * cloud_out->height);
    std::cout << "Transformed " << cloud_out->points.size () << " data points:"
        << std::endl;
    for (size_t i = 0; i < cloud_out->points.size (); ++i)
    {
        std::cout << "    " << cloud_out->points[i].x << " " <<
        cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
    }

    *cloud_sum = *cloud_out + *cloud_in;
    
      // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_sum);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (2.0); // 2cm
    ec.setMinClusterSize (3);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_sum);
    ec.extract (cluster_indices);

    std::cout << "cluster size : " << cluster_indices.size() << std::endl;


    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud_sum->points[*pit]); //*            
        }            
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

        for (size_t i = 0; i < cloud_cluster->points.size (); i++)
        {
            std::cout << "    " << cloud_cluster->points[i].x << " " <<
            cloud_cluster->points[i].y << " " << cloud_cluster->points[i].z << std::endl;
        }
        


        j++;
    }


    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // tree->setInputCloud (cloud_filtered);

    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ec.setClusterTolerance (0.02); // 2cm
    // ec.setMinClusterSize (100);
    // ec.setMaxClusterSize (25000);
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (cloud_filtered);
    // ec.extract (cluster_indices);

    return 0;
}