#include "ros/ros.h"
#include <iostream>
#include <fstream>   
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <livox_sdk.h>
#include <livox_def.h>
#include "livox_ros_driver/CustomMsg.h"
#include "sensor_msgs/PointCloud.h"
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_types.h"
#include "pcl/conversions.h"
#include "pcl_ros/transforms.h"
#include "pcl_conversions/pcl_conversions.h"

bool flag = false;
geometry_msgs::Twist twist;
std::vector<float> ranges;
ros::Publisher pub_cloud ;
ros::Subscriber sub_livox;

void laserCallback(const livox_ros_driver::CustomMsg::ConstPtr &livox_msg_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto point : livox_msg_ptr->points)
    {
        pcl::PointXYZI pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        pcl_point.intensity = point.reflectivity;
        pcl_cloud->push_back(pcl_point);
    }
    pcl::toROSMsg(*pcl_cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = livox_msg_ptr->header.frame_id;
    pub_cloud.publish(cloud_msg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "livox_receive");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    std::string car_name;
    nh.param<std::string>("car_name", car_name, std::string("car100"));
  
    
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/"+car_name+"/livox/PointCloud2", 10, true);
    sub_livox = nh.subscribe("/"+car_name+"/livox/lidar", 1000, laserCallback);
    
    ros::spin();
}
