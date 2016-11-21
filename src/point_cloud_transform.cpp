#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


#include "tf/transform_datatypes.h"


const static double	pi=3.1415926;

//pcl::visualization::CloudViewer MyViewer("cloud");
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
ros::Publisher	pub_cloud;

/*


*/

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);
	
	
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.0, 0.0, 0.0;
	transform.rotate(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*cloud_in, *cloud_in, transform);
	
//	pcl::PassThrough<pcl::PointXYZI> pass;
//	pass.setInputCloud(cloud_in);
//	pass.setFilterFieldName("y");
//	pass.setFilterLimits(0,20);
//	pass.filter(*cloud_in);
//	
//	MyViewer.showCloud(cloud_in,"cloud");
	
	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_in, cloud_to_pub);
    pub_cloud.publish(cloud_to_pub);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "point_cloud_transform");
	ros::NodeHandle nh;
	ros::Subscriber sub_imu = nh.subscribe("velodyne_points",2,pointCloudCallback);
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("points_in_car_coord",2);
	
	ros::spin();
	return 0;
}
