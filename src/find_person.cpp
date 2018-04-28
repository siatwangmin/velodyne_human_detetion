#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/boost.h>
#include <pcl/common/centroid.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include "geometry_msgs/Point.h"
#include <boost/shared_ptr.hpp>
#include "velodyne_detect_person/pointCloudVector.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/PoseArray.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
sensor_msgs::PointCloud2::Ptr backgroundCloud (new sensor_msgs::PointCloud2);
pcl::PointCloud< pcl::PointXYZ > backgroundCloudPCL;
pcl::PointCloud< pcl::PointXYZ > clusterPCL;
int backgroundSize = 0; //Number of points in background cloud
bool backgroundGrid[100][100][30];
Eigen::Vector4f centroid;
geometry_msgs::PointStamped personCentroid;
geometry_msgs::PointStamped personCentroidTransformed;
geometry_msgs::PointStamped robotPose;
float topPoint, lowestPoint;


void transformPersonPosition (const tf::TransformListener& listener)
{
	personCentroid.header.frame_id = "/velodyne";
	personCentroid.header.stamp = ros::Time();
	try
	{
		listener.transformPoint("world", personCentroid, personCentroidTransformed);
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("Received an exception trying to transform a point from \"personCentroid\" to 		\"personCentroidTransformed\": %s", ex.what());
	}
}

class FindPerson
{
protected:
	ros::NodeHandle n;
	
public:
	ros::Publisher pub;
	ros::Publisher pub2;
	ros::Publisher people_positions_pub;
	
	ros::Subscriber subBackground;
	ros::Subscriber subClusters;
	ros::Subscriber subRobotInfo;
	tf::TransformListener listener;
    
	//Set background cloud in a global variable. One message every X seconds
	void findPersonBackgroundCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputBackgroundCloud)
	{
		backgroundCloud = inputBackgroundCloud;
		pcl::fromROSMsg(*backgroundCloud,backgroundCloudPCL);
		backgroundSize = backgroundCloud->width;


		pcl::PointCloud<pcl::PointXYZ>::iterator it;
		for(it = backgroundCloudPCL.points.begin(); it < backgroundCloudPCL.points.end(); it++)
		{
			if (-5 <= it->x && 
				it->x < 5 && 
				-5 <= it->y && 
				it->y < 5 && 
				-1.5 <= it->z && 
				it->z < 1.5)
			{
				backgroundGrid[int((it->x)*10+50)][int((it->y)*10+50)][int((it->z)*10+15)] = true;
			}
		}	
	}
	
	
	//Receive data and perform the actual person detection. Lots of messages every second
	void findPersonClustersCallback(const boost::shared_ptr<velodyne_detect_person::pointCloudVector>& clusterVector)
	{
		geometry_msgs::PoseArray people_poses_array;
		pcl::PointCloud<pcl::PointXYZ>::Ptr clustersCloud (new pcl::PointCloud<pcl::PointXYZ>); //Contains every person cluster and is visible in rviz
		pcl::PointCloud<pcl::PointXYZ> auxiliarCluster;
		sensor_msgs::PointCloud2::Ptr clustersCloudRos (new sensor_msgs::PointCloud2);
		personCentroid.point.x = 0;
		personCentroid.point.y = 0;
	
	
		if(backgroundSize == 0)
		{
			std::cout << "I need a background cloud!" << std::endl;
		}
		else
		{	
			int human_cluster_count = 0;	
			//For each cluster
			for(int i = 0; i < clusterVector->pointCloudVector.size(); i++)
			{
				topPoint = -100, lowestPoint = 100;
				int clusterPoints = clusterVector->pointCloudVector[i].width;  //cluster size
				int numCoincidentPoints = 0;
				pcl::fromROSMsg(clusterVector->pointCloudVector[i],clusterPCL);
				
				//Add 1 to counter if a point is in cluster and background
				for(int pointCluster = 0; pointCluster < clusterPoints; pointCluster++)
				{
					if (clusterPCL.points[pointCluster].z > topPoint) 
					{
						topPoint = clusterPCL.points[pointCluster].z;
					}
					else if (clusterPCL.points[pointCluster].z < lowestPoint) 
					{
						lowestPoint = clusterPCL.points[pointCluster].z;
					}
					if (-5 <= clusterPCL.points[pointCluster].x &&
				  		clusterPCL.points[pointCluster].x < 5 &&
					  	-5 <= clusterPCL.points[pointCluster].y &&
					  	clusterPCL.points[pointCluster].y < 5 &&
					  	-1.5 <= clusterPCL.points[pointCluster].z &&
					  	clusterPCL.points[pointCluster].z < 1.5)
     				{								
						if(backgroundGrid[int((clusterPCL.points[pointCluster].x)*10+50)][int((clusterPCL.points[pointCluster].y)*10+50)][int((clusterPCL.points[pointCluster].z)*10+15)] == true)
						{
							// ROS_INFO("numCoincidentPoints add one");
							numCoincidentPoints++;
						}	
					}		
				}
				// ROS_INFO("coincident Point ratio %f", float(float(numCoincidentPoints)/float(clusterPoints)));
				//If cluster is not in background (coincident points < 5%),
				if(float(float(numCoincidentPoints)/float(clusterPoints)) < 0.05)
				{
					pcl::fromROSMsg(clusterVector->pointCloudVector[i],auxiliarCluster);	
					pcl::compute3DCentroid(clusterPCL, centroid);
					
					human_cluster_count ++;
					*clustersCloud += auxiliarCluster;
					personCentroid.point.x = centroid(0,0);
					personCentroid.point.y = centroid(1,0);
					personCentroid.point.z = 0;
					personCentroid.header.stamp = ros::Time();
					//Transform position into world frame
					transformPersonPosition(listener);

					geometry_msgs::Pose tmp_pose;
					tmp_pose.position.x = personCentroid.point.x;
					tmp_pose.position.y = personCentroid.point.y;
					tmp_pose.position.z = personCentroid.point.z;
					people_poses_array.poses.push_back(tmp_pose);
				}
			
			}

			ROS_INFO("human cluster num : %d ", human_cluster_count);
			pcl::toROSMsg (*clustersCloud , *clustersCloudRos);
			clustersCloudRos->header.frame_id = "/velodyne";
			clustersCloudRos->header.stamp = ros::Time::now();
			
			pub.publish (clustersCloudRos);		
			
			people_poses_array.header.frame_id = "/velodyne";
			people_poses_array.header.stamp = ros::Time::now();
			people_positions_pub.publish(people_poses_array);
			
			pub2.publish (personCentroid);
		}
	}
	
	//Receive robot position and stores it in a global variable
	void findPersonRobotPositionCallback(const geometry_msgs::PointStamped& robotPosition)
	{
		robotPose = robotPosition;		
	}
	
	FindPerson()
    {
		pub  = n.advertise<sensor_msgs::PointCloud2> ("person_cloud", 1);
		pub2 = n.advertise<geometry_msgs::PointStamped> ("person_position", 1);

		people_positions_pub = n.advertise<geometry_msgs::PoseArray>("people_positions", 1);
		subBackground = n.subscribe("scene_background", 1, &FindPerson::findPersonBackgroundCallback, this);
		subClusters = n.subscribe("scene_clusters", 1, &FindPerson::findPersonClustersCallback, this);
		subRobotInfo = n.subscribe("robot_position", 1, &FindPerson::findPersonRobotPositionCallback, this);
    }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_person");
  //ros::NodeHandle n;
  
  FindPerson findP;
  
  //we'll transform a point once every second
  //ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
  
  ros::spin();

  return 0;
}

