#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/core/core.hpp"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <QMetaType>
//qRegisterMetaType<QVector<int>>("QVector<int>");

using namespace std;
using namespace cv;

cv::Mat src;
string fileName;
nav_msgs::OccupancyGrid my_global_map;

nav_msgs::OccupancyGrid my_local_map;

nav_msgs::OccupancyGrid BuildMap(string metaPath);
nav_msgs::OccupancyGrid BuildMapDilate(string metaPath);
nav_msgs::OccupancyGrid BuildMapErode(string metaPath);
nav_msgs::OccupancyGrid BuildLocaMap(string metaPath);

pcl::PointCloud<pcl::PointXYZI> cloud;
//pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(&cloud);
//#include <sys/time.h>
visualization_msgs::MarkerArray marker_array;

string map_config_path;
string config_file_name;
string local_map_config_file_name;

string paramFileName;
string paramFileName_local;

double lq_position_x=100,lq_position_y=100;
int startX,startY;

int global_map_width,global_map_height;

class SegMapROSWraper  
{
public:
  ros::NodeHandle m_nh;  
  ros::Subscriber global_map_sub;
  ros::Publisher local_map_pub;
  ros::Subscriber odom_sub;

public:
  SegMapROSWraper()
      : m_nh("")  
  {
    global_map_sub = m_nh.subscribe("ow/global_map", 1, &SegMapROSWraper::global_map_Callback,this);
    local_map_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("ow/local_map", 1);
    odom_sub = m_nh.subscribe("/odom", 1, &SegMapROSWraper::cloudCB,this);
  }

  ~SegMapROSWraper()
  {
      //delete global_map_pub;
      //delete local_map_pub;
  }
  
  void global_map_Callback(const nav_msgs::OccupancyGrid &msg)
{
    my_global_map=msg;
}
 
  void cloudCB(const nav_msgs::Odometry &input)
  {
    
      lq_position_x=input.pose.pose.position.x;
      lq_position_y=input.pose.pose.position.y;
      
  }
  
};
int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL, "");
    ros::init (argc, argv, "local_map_generation");

    
    local_map_config_file_name="local_map.yaml";
    paramFileName_local=ros::package::getPath("map_fusion");
    paramFileName_local=paramFileName_local.append("/initial_map/");
    paramFileName_local=paramFileName_local.append(local_map_config_file_name);
    my_local_map=BuildLocaMap(paramFileName_local);
    
    

    SegMapROSWraper  SM;

    

    ros::Rate loop_rate(10);
    while (ros::ok())
    {   
      if (my_global_map.info.width!=0)
     {
        cv::FileStorage f_yaml(paramFileName_local,cv::FileStorage::READ);
        double resolution=(float)f_yaml["resolution"];
        cv::FileNode arr1 = f_yaml["mapsize"];
        cv::FileNodeIterator it1 = arr1.begin(),it1_end = arr1.end();
        float mapsize_x=(float)(*it1);it1++;
        float mapsize_y=(float)(*it1);it1++;
        int index_origin_x=(int)(lq_position_x/resolution);
        int index_origin_y=(int)(lq_position_y/resolution);

        int start_x=(int)(index_origin_x-mapsize_x/2);
        int end_x=(int)(index_origin_x+mapsize_x/2);
        int start_y=(int)(index_origin_y-mapsize_x/2);
        int end_y=(int)(index_origin_y+mapsize_x/2);
        vector<signed char> local_vec;

        for(int i=start_y;i<end_y;i++)
        {
          for(int j=start_x;j<end_x;j++)
          {
            local_vec.push_back(my_global_map.data[j+(i+0)*(int)(my_global_map.info.width)]);
          }
        }
        my_local_map.data=local_vec;      
        SM.local_map_pub.publish(my_local_map);
      }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}

nav_msgs::OccupancyGrid BuildLocaMap(string metaPath)
{
    nav_msgs::OccupancyGrid mymap;
    mymap.header.frame_id="local_map";
    uint64_t sys_time=std::chrono::duration_cast<std::chrono::microseconds>
                                  (std::chrono::system_clock::now().time_since_epoch()).count();
    int32_t time_second=sys_time / 1000000;
    int32_t time_nsecs=sys_time % 1000000 * 1000;
    mymap.header.stamp.sec = time_second;
    mymap.header.stamp.nsec = time_nsecs;
    
    cv::FileStorage f_yaml(metaPath,cv::FileStorage::READ);
    mymap.info.resolution=(float)f_yaml["resolution"];
  
    cv::FileNode arr1 = f_yaml["mapsize"];
    cv::FileNodeIterator it1 = arr1.begin(),it1_end = arr1.end();
    float mapsize_x=(float)(*it1);it1++;
    float mapsize_y=(float)(*it1);it1++;
    
    cv::Size imageSize;
    imageSize.width=mapsize_x;
    imageSize.height=mapsize_y;
    
    cv::FileNode arr = f_yaml["origin"];
    cv::FileNodeIterator it = arr.begin(),it_end = arr.end();
    mymap.info.origin.position.x=(float)(*it);it++;
    mymap.info.origin.position.y=(float)(*it);it++;
    mymap.info.origin.position.z=(float)(*it);
    mymap.info.origin.orientation.w=1.0;
    mymap.info.origin.orientation.x=0.0;
    mymap.info.origin.orientation.y=0.0;
    mymap.info.origin.orientation.z=0.0;
    f_yaml.release();

    mymap.info.width=imageSize.width;
    mymap.info.height=imageSize.height;

    return mymap;

}