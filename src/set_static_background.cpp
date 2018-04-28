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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
#define GRID_SIZE 0.1
#define X_SIZE 100
#define Y_SIZE 100
#define Z_SIZE 30
#define BUFFER_SIZE 30

//Create a pointcloud which holds background points		
//Create a pointcloud which holds background points candidates
sensor_msgs::PointCloud2::Ptr background (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr backgroundCandidates (new sensor_msgs::PointCloud2);
pcl::PCLPointCloud2::Ptr inputCloudPCL (new pcl::PCLPointCloud2());

pcl::PCLPointCloud2::Ptr inputCloudPCLSUM (new pcl::PCLPointCloud2());

pcl::PCLPointCloud2::Ptr outputCloudPCL (new pcl::PCLPointCloud2());
pcl::PointCloud<pcl::PointXYZ> auxCloud;
sensor_msgs::PointCloud2::Ptr initialBackground (new sensor_msgs::PointCloud2);
pcl::PCLPointCloud2::Ptr initialBackgroundPCL_pc2 (new pcl::PCLPointCloud2());
pcl::PointCloud<pcl::PointXYZ>::Ptr initialBackgroundPCL(new pcl::PointCloud<pcl::PointXYZ>);


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_union (new pcl::PointCloud<pcl::PointXYZ>);


//Create a background grid and a buffer for each grid point
bool backgroundGrid[X_SIZE][Y_SIZE][Z_SIZE];
bool buffer[X_SIZE][Y_SIZE][Z_SIZE][BUFFER_SIZE];

class SetBackground
{
protected:
    ros::NodeHandle n;
public:
    ros::Publisher pub;
    ros::Subscriber sub;
    bool firstTime;
	int frame_num ;
    tf::TransformListener listener;

	void setBackgroundCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloud)
	{		
	  	sensor_msgs::PointCloud2 publishedCloud;
	  	pcl::PointCloud<pcl::PointXYZ> publishedCloudPCL;
	  	
	  	pcl::PCLPointCloud2 pcl_pc2;
	  	pcl_conversions::toPCL(*inputCloud, pcl_pc2);

		if(frame_num < 50)
		{
			pcl::fromROSMsg (*inputCloud, *cloud_in);
			//  pcl::fromPCLPointCloud2(pcl_pc2,*cloud_in);
			 *cloud_sum += *cloud_in;
			
			frame_num++;
			ROS_INFO("Got %d frame", frame_num);
			return;
		}


	  	//Add initial background cloud to prevent shadows
		if(firstTime)
		{
			pcl::toPCLPointCloud2(*cloud_sum,*inputCloudPCLSUM);
			// *inputCloudPCLSUM += *inputCloudPCL;
			pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
			grid.setInputCloud (inputCloudPCLSUM);
			grid.setLeafSize (0.04f, 0.04f, 0.04f);
			grid.filter (*outputCloudPCL);
			pcl::fromPCLPointCloud2(*outputCloudPCL,auxCloud);
			pcl::toROSMsg (auxCloud, *background);
			initialBackground = background;
			pcl_conversions::toPCL(*initialBackground, *initialBackgroundPCL_pc2);
			pcl::fromPCLPointCloud2(*initialBackgroundPCL_pc2,*initialBackgroundPCL);
		}
		publishedCloudPCL += *initialBackgroundPCL;

		ROS_INFO("published Cloud PCL size : %zu", publishedCloudPCL.size());

		firstTime = false;
			
		pcl::toROSMsg (publishedCloudPCL , publishedCloud);
		publishedCloud.header.frame_id = "/velodyne";
		publishedCloud.header.stamp = ros::Time::now();	

		pub.publish (publishedCloud);

	}
	
	SetBackground()
	{
		firstTime = true;
		frame_num = 0;
		pub = n.advertise<sensor_msgs::PointCloud2> ("scene_background", 1);
		sub = n.subscribe("velodyne_points", 1, &SetBackground::setBackgroundCallback, this);
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "set_background");
	SetBackground setB;
	ros::spin();

	return 0;
}
