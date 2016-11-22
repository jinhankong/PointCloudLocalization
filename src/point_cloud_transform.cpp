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

#include <dynamic_reconfigure/server.h>
#include <localization/calibrationConfig.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);

class PointCloudTransform
{
public:
	PointCloudTransform();
	
	void configCallback(localization::calibrationConfig &config, uint32_t level);
	void leftPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void rightPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void topPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_left_points;
	ros::Subscriber sub_right_points;
	ros::Subscriber sub_top_points;
	ros::Publisher pub_left_transformed;
	ros::Publisher pub_right_transformed;
	ros::Publisher pub_top_transformed;
	//---------参数服务相关变量------------
	dynamic_reconfigure::Server<localization::calibrationConfig> dr_srv;
	dynamic_reconfigure::Server<localization::calibrationConfig>::CallbackType cb;
	
	double trans_lx,trans_ly,trans_lz;
	double rotate_lx,rotate_ly,rotate_lz;
	
	double trans_rx,trans_ry,trans_rz;
	double rotate_rx,rotate_ry,rotate_rz;
	
};

PointCloudTransform::PointCloudTransform()
{
	trans_lx = 0.0;
	trans_ly = 0.0;
	trans_lz = 0.0;
	rotate_lx = 0.0;
	rotate_ly = 0.0;
	rotate_lz = 0.0;
	
	sub_left_points = nh.subscribe("/left/velodyne_points",2,&PointCloudTransform::leftPointCloudCallback,this);
	sub_right_points = nh.subscribe("/right/velodyne_points",2,&PointCloudTransform::rightPointCloudCallback,this);
	sub_top_points = nh.subscribe("/velodyne_points",2,&PointCloudTransform::topPointCloudCallback,this);
	pub_left_transformed = nh.advertise<sensor_msgs::PointCloud2>("left_points_transformed",2);
	pub_right_transformed = nh.advertise<sensor_msgs::PointCloud2>("right_points_transformed",2);
	pub_top_transformed = nh.advertise<sensor_msgs::PointCloud2>("top_points_transformed",2);
	
	//------配置动态更改参数服务-------	
	cb = boost::bind(&PointCloudTransform::configCallback, this, _1, _2);
	dr_srv.setCallback(cb);
	
	ros::NodeHandle pnh("~");
	pnh.param("rotate_ly", rotate_ly, 5.4);
	pnh.param("rotate_ry", rotate_ry, 0.9);
	pnh.param("trans_lx", trans_lx, -0.6);
	pnh.param("trans_ly", trans_ly, 0.0);
	pnh.param("trans_lz", trans_lz, 0.1);
	pnh.param("trans_rx", trans_rx, 0.6);
	pnh.param("trans_ry", trans_ry, 0.0);
	pnh.param("trans_rz", trans_rz, -0.1);
	
	ros::spin();
}



void PointCloudTransform::configCallback(localization::calibrationConfig &config, uint32_t level)
{
	rotate_ly = config.rotate_ly;
	trans_lx = config.trans_lx;
	trans_ly = config.trans_ly;
	trans_lz = config.trans_lz;
	
	rotate_ry = config.rotate_ry;
	trans_rx = config.trans_rx;
	trans_ry = config.trans_ry;
	trans_rz = config.trans_rz;
}

void PointCloudTransform::leftPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);
	
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << trans_lx, trans_ly , trans_lz;
	transform.rotate(Eigen::AngleAxisf(rotate_ly, Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*cloud_in, *cloud_in, transform);
	
//	Eigen::AngleAxisf rot1(1.34, Eigen::Vector3f::UnitY());
//	Eigen::Translation3f tra1(-2.1, 0, 0.72);
//	Eigen::Matrix4f trafin = (tra1 * rot1).matrix();
//	pcl::transformPointCloud(*cloud_new, *cloud_new, trafin);
	
	
	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_in, cloud_to_pub);
    pub_left_transformed.publish(cloud_to_pub);
}

void PointCloudTransform::rightPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);
	
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << trans_rx, trans_ry , trans_rz;
	transform.rotate(Eigen::AngleAxisf(rotate_ry, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*cloud_in, *cloud_in, transform);
	
	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_in, cloud_to_pub);
    pub_right_transformed.publish(cloud_to_pub);
}

void PointCloudTransform::topPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);
		
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.0, 0.0 , 0.0;
	transform.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*cloud_in, *cloud_in, transform);
	
	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_in, cloud_to_pub);
    pub_top_transformed.publish(cloud_to_pub);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "point_cloud_transform");
	PointCloudTransform tr;
	return 0;
}
