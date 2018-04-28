#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/boost.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include "std_msgs/String.h"
#include <boost/shared_ptr.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include "velodyne_detect_person/pointCloudVector.h"
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <sstream>
#include <string>

#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/voxel_grid.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_detect_person/ExtractClustersConfig.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

pcl::PCLPointCloud2::Ptr newRawPCL2 (new pcl::PCLPointCloud2());

pcl::PointCloud< pcl::PointXYZ >::Ptr backgroundCloudPCL(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud< pcl::PointXYZ >::Ptr newRawCloudPCL(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud< pcl::PointXYZ >::Ptr diffCloudPCL(new pcl::PointCloud<pcl::PointXYZ>);
int backgroundSize = 0;
sensor_msgs::PointCloud2::Ptr backgroundCloud (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr raw_point_cloud (new sensor_msgs::PointCloud2);




void findPersonBackgroundCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputBackgroundCloud)
{
	backgroundCloud = inputBackgroundCloud;
	backgroundSize = backgroundCloud->width;

	pcl::fromROSMsg(*backgroundCloud, *backgroundCloudPCL);
}


class ExtractClusters
{
protected:
	ros::NodeHandle n;
	ros::Time begin;
	int n_published_msgs;
public:
	ros::Publisher pubdiff;
	ros::Publisher pub;
	ros::Publisher pub2;
	ros::Subscriber subBackground;
	ros::Subscriber sub;

	// float resolution = 0.5;
	// double cluster_tolerance = 1.0;
	// int min_cluster_size = 10;

	float resolution;
	double cluster_tolerance;
	int min_cluster_size;

	void extractClustersCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloud)
	{
		if(backgroundSize == 0)
		{
			ROS_INFO("Extra cluster !! I need a background cloud!");
			return;
		}


		pcl::fromROSMsg(*inputCloud, *newRawCloudPCL);

		// pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
		// grid.setInputCloud (newRawPCL2);
		// grid.setLeafSize (0.04f, 0.04f, 0.04f);
		// grid.filter (*newRawPCL2);
		// pcl::fromPCLPointCloud2(*newRawPCL2,*newRawCloudPCL);


		// float resolution = resolution;
		// Instantiate octree-based point cloud change detection class
		pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

		octree.setInputCloud (backgroundCloudPCL);
		octree.addPointsFromInputCloud ();
		octree.switchBuffers ();

		octree.setInputCloud (newRawCloudPCL);
		octree.addPointsFromInputCloud ();
		std::vector<int> newPointIdxVector;

	// Get vector of point indices from octree voxels which did not exist in previous buffer
		octree.getPointIndicesFromNewVoxels (newPointIdxVector);

		diffCloudPCL->width = newPointIdxVector.size();
		diffCloudPCL->height = 1;
		diffCloudPCL->points.resize(diffCloudPCL->width * diffCloudPCL->height);
		for (size_t i = 0; i < newPointIdxVector.size (); ++i)
		{
			diffCloudPCL->points[i].x = newRawCloudPCL->points[newPointIdxVector[i]].x;
			diffCloudPCL->points[i].y = newRawCloudPCL->points[newPointIdxVector[i]].y;
			diffCloudPCL->points[i].z = newRawCloudPCL->points[newPointIdxVector[i]].z;
			
		}

		sensor_msgs::PointCloud2 publishedCloud;
		pcl::toROSMsg (*diffCloudPCL , publishedCloud);

		publishedCloud.header.frame_id = "/velodyne";
		publishedCloud.header.stamp = ros::Time::now();	

		pubdiff.publish (publishedCloud);

		ROS_INFO("diff size %d", diffCloudPCL->width);



		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

		tree->setInputCloud (diffCloudPCL);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (cluster_tolerance); // 2cm
		ec.setMinClusterSize (min_cluster_size);
		ec.setMaxClusterSize (25000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (diffCloudPCL);
		ec.extract (cluster_indices);

		sensor_msgs::PointCloud2::Ptr clustersCloudRos (new sensor_msgs::PointCloud2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr clustersCloud (new pcl::PointCloud<pcl::PointXYZ>);

		sensor_msgs::PointCloud2::Ptr auxiliarCluster (new sensor_msgs::PointCloud2);

		std::vector<pcl::PointIndices>::const_iterator it;
		velodyne_detect_person::pointCloudVector clusterPointClouds;

		//For every cluster, store it into file and publisher structure
		for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud (*diffCloudPCL, it->indices, *cloud_cluster);
			pcl::toROSMsg (*cloud_cluster , *auxiliarCluster);
			auxiliarCluster->header.frame_id = "/velodyne";
			auxiliarCluster->header.stamp = ros::Time::now();
			clusterPointClouds.pointCloudVector.push_back(*auxiliarCluster);
			*clustersCloud += *cloud_cluster;
		}
		
		pcl::toROSMsg (*clustersCloud , *clustersCloudRos);
		clustersCloudRos->header.frame_id = "/velodyne";
		clustersCloudRos->header.stamp = ros::Time::now();

		pub.publish (clusterPointClouds);
		pub2.publish (*clustersCloudRos);
	}
	
	ExtractClusters()
	{
		
		pub = n.advertise<velodyne_detect_person::pointCloudVector> ("scene_clusters", 1);
		pub2 = n.advertise<sensor_msgs::PointCloud2> ("clustersCloud", 1);
		pubdiff = n.advertise<sensor_msgs::PointCloud2> ("diff_points", 1);
		subBackground = n.subscribe("scene_background", 1, &findPersonBackgroundCallback);
		sub = n.subscribe("velodyne_points", 1, &ExtractClusters::extractClustersCallback, this);
		begin = ros::Time::now();
		n_published_msgs = 0;

		resolution = 0.5;
		cluster_tolerance = 1.0;
		min_cluster_size = 10;
	}
};


	

ExtractClusters* exc_ptr;

void callback(velodyne_detect_person::ExtractClustersConfig &config,  uint32_t level) 
{
  exc_ptr->resolution = config.resolution;
  exc_ptr->cluster_tolerance = config.cluster_tolerance;
  exc_ptr->min_cluster_size = config.min_cluster_size;

  ROS_INFO("Reconfigure Request: %f \t %f \t %d", 
            exc_ptr->resolution, exc_ptr->cluster_tolerance, 
            exc_ptr->min_cluster_size);
}


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "extract_clusters");
	ExtractClusters exC;

	exc_ptr = &exC;

	dynamic_reconfigure::Server<velodyne_detect_person::ExtractClustersConfig> server;
	dynamic_reconfigure::Server<velodyne_detect_person::ExtractClustersConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
