#include <ros/ros.h>
#include <chrono>
#include <ros/package.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

using namespace std;

double lq_position_x,lq_position_y;

void cloudCB(const nav_msgs::Odometry &input)
{
    lq_position_x=input.pose.pose.position.x;
    lq_position_y=input.pose.pose.position.y;   
}

int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL, "");
    ros::init (argc, argv, "tf_local_map");
    ros::NodeHandle m_nh;  
    ros::Subscriber odom_sub = m_nh.subscribe("/odom", 1, cloudCB);
    ros::Rate loop_rate(1000);

    while (ros::ok())
    {   
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(lq_position_x, lq_position_y, 0.0) );
        tf::Quaternion q;
        q.setW(1.0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "local_map"));
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
