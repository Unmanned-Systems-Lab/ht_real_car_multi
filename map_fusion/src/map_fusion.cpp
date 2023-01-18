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
nav_msgs::OccupancyGrid my_initial_map;

nav_msgs::OccupancyGrid my_local_map;

nav_msgs::OccupancyGrid BuildMap(string metaPath);
nav_msgs::OccupancyGrid BuildMapDilate(string metaPath);
nav_msgs::OccupancyGrid BuildMapErode(string metaPath);
nav_msgs::OccupancyGrid BuildLocaMap(string metaPath,double x,double y);

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

double global_map_resolution;
double global_map_width;
class SegMapROSWraper  
{
public:
  ros::NodeHandle m_nh;  
  ros::Publisher global_map_pub;
  //ros::Publisher local_map_pub;
  //ros::Publisher normal_map_pub;
  ros::Subscriber sub;
  ros::Subscriber lidar_cloud_point_sub;
  ros::ServiceServer server;
  ros::Subscriber odom_sub;

public:
  SegMapROSWraper()
      : m_nh("~")  
  {
    global_map_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("ow/global_map", 1);
    //local_map_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("ow/local_map", 1);
    //normal_map_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("ow/origin_map", 10);
    sub = m_nh.subscribe("/marine_lidar_map",10,&SegMapROSWraper::chatterCallback,this);
    lidar_cloud_point_sub = m_nh.subscribe("/point_cloud_filtered_in_map_coordinate",10,&SegMapROSWraper::lidar_Callback,this);
    server = m_nh.advertiseService("/clear_global_map",&SegMapROSWraper::commandCallback,this);
    // odom_sub = m_nh.subscribe("/odom", 1, &SegMapROSWraper::cloudCB,this);
  }

  ~SegMapROSWraper()
  {
      //delete global_map_pub;
      //delete local_map_pub;
  }
  
  void lidar_Callback(const sensor_msgs::PointCloud2 &msg)
{
        //cout<<"point_cloud_received!"<<endl;
        pcl::fromROSMsg(msg, cloud);

        for(int nIndex = 0;nIndex < cloud.points.size();nIndex++)
        {  
                int x_index=cloud.points[nIndex].x/global_map_resolution;
                int y_index=cloud.points[nIndex].y/global_map_resolution;
                for(int i=1;i<=20;i++)
                {
                    for(int j=1;j<=20;j++)
                    {
                        my_initial_map.data[(y_index-10+i)*global_map_width+x_index-10+j]=100;
                    }
                }  
        }
}
  void chatterCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
   ROS_INFO("Hello test1_a! ");
}
  bool ServiceCallBack(nav_msgs::GetMap::Request &req,nav_msgs::GetMap::Response &res)
{   
    string filePath=ros::package::getPath("map_fusion");
    //string mapFileName=filePath.append("/initial_map/map3.pgm");
    string paramFileName=filePath.append("/initial_map/");
    paramFileName=paramFileName.append(config_file_name);
    res.map=BuildMap(paramFileName);
    return true;
}
//   void cloudCB(const nav_msgs::Odometry &input)
//   {
//     lq_position_x=input.pose.pose.position.x;
//     lq_position_y=input.pose.pose.position.y;
//   }
  bool commandCallback(std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response &res)
  {
     //std::cout<< "triger_ok" <<std::endl;
     string paramFileName=ros::package::getPath("map_fusion");
     paramFileName=paramFileName.append("/initial_map/");
     paramFileName=paramFileName.append(config_file_name);
     my_initial_map=BuildMap(paramFileName);
     res.success=true;
     res.message="clear global map ok !";
     return true;
  }
  
};
int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL, "");
    ros::init (argc, argv, "map_fusion");
    //ros::NodeHandle node_handle;
    SegMapROSWraper  SM;

    // SM.m_nh.param<std::string>("config_file_name",config_file_name,"default_arg1");
    // SM.m_nh.param<std::string>("local_map_config_file_name",local_map_config_file_name,"default_arg1");
    config_file_name="NPU_map.yaml";
    local_map_config_file_name="NPU_map.yaml";
    cout<<config_file_name<<"   "<<local_map_config_file_name<<endl;
    paramFileName=ros::package::getPath("map_fusion");
    paramFileName=paramFileName.append("/initial_map/");
    paramFileName=paramFileName.append(config_file_name);
    my_initial_map=BuildMap(paramFileName);

    // paramFileName_local=ros::package::getPath("map_fusion");
    // paramFileName_local=paramFileName_local.append("/initial_map/");
    // paramFileName_local=paramFileName_local.append(local_map_config_file_name);
    // my_local_map=BuildLocaMap(paramFileName_local,lq_position_x,lq_position_y);
    
    

    
    //ros::Publisher marker_pub1 = node_handle.advertise<visualization_msgs::MarkerArray>("visualization_marker1", 10);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {   
        SM.global_map_pub.publish(my_initial_map);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
//mapType:0，原图
nav_msgs::OccupancyGrid BuildMap(string metaPath)
{
    nav_msgs::OccupancyGrid mymap;
    mymap.header.frame_id="map";

    uint64_t sys_time=std::chrono::duration_cast<std::chrono::microseconds>
                                  (std::chrono::system_clock::now().time_since_epoch()).count();
    int32_t time_second=sys_time / 1000000;
    int32_t time_nsecs=sys_time % 1000000 * 1000;
    mymap.header.stamp.sec = time_second;
    mymap.header.stamp.nsec = time_nsecs;
    //map.header.stamp=ros::Time::now();
    //std::cout<<"12311"<<std::endl;
    
    cv::FileStorage f_yaml(metaPath,cv::FileStorage::READ);
    mymap.info.resolution=(float)f_yaml["resolution"];
    global_map_resolution=(float)f_yaml["resolution"];
    string filePath=ros::package::getPath("map_fusion");
    filePath=filePath.append("/initial_map/");
    filePath=filePath.append((string)f_yaml["image"]);
    //std::cout<<filePath<<std::endl;
    cv::Mat image = cv::imread(filePath,-1);

    
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
    
    vector<signed char> vec;

    for(int i=image.rows-1;i>=0;i--)
    {
        for(int j=0;j<image.cols;j++)
        {
            vec.push_back((255-(uint)image.at<uchar>(i,j))*100/255);      
        }
    }

    mymap.info.width=image.cols;
    mymap.info.height=image.rows;
    global_map_width=image.cols;
    //cout<<"占据网格元信息："<<endl;
    //cout<<mymap<<endl;

    mymap.data =vec;
    return mymap;

}