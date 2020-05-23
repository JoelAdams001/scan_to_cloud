#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>

tf2_ros::Buffer tfbuffer;
ros::Publisher cloud_pub;
pcl::PCLPointCloud2::Ptr cloud_full(new pcl::PCLPointCloud2);

Eigen::MatrixXd create_transform(geometry_msgs::TransformStamped t)
{
  double qx = t.transform.rotation.x;
  double qy = t.transform.rotation.y;
  double qz = t.transform.rotation.z;
  double qw = t.transform.rotation.w;
  double x = t.transform.translation.x;
  double y = t.transform.translation.y;
  double z = t.transform.translation.z;
  Eigen::MatrixXd T(4,4);
  T(0,0) = 1 - 2*pow(qy,2) - 2*pow(qz,2);
  T(0,1) = 2*qx*qy - 2*qz*qw;
  T(0,2) = 2*qx*qz + 2*qy*qw;
  T(0,3) = x;
  T(1,0) = 2*qx*qy + 2*qz*qw;
  T(1,1) = 1 - 2*pow(qx,2) - 2*pow(qz,2);
  T(1,2) = 2*qy*qz - 2*qx*qw;
  T(1,3) = y;
  T(2,0) = 2*qx*qz - 2*qy*qw;
  T(2,1) = 2*qy*qz + 2*qx*qw;
  T(2,2) = 1 - 2*pow(qx,2) - 2*pow(qy,2);
  T(2,3) = z;
  T(3,0) = 0;
  T(3,1) = 0;
  T(3,2) = 0;
  T(3,3) = 1;
  return T;
}

pcl::PointCloud<pcl::PointXYZ> transform_to_cloud(const sensor_msgs::LaserScanPtr& s, Eigen::MatrixXd t) {
  static pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 40000;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  cloud.header.frame_id = "odom";  
  Eigen::VectorXd v(4);
  double ang = s->range_min - s->angle_increment - 0.886;
  int i = 0;
  for(auto &range : s->ranges) {
    ang += s->angle_increment;
    double x = range*sin(ang);
    double y = range*cos(ang);
    v(0) = x;
    v(1) = -y;
    v(2) = 0;
    v(3) = 1;
    v = t*v;
    cloud.points[i].x = v(0);
    cloud.points[i].y = v(1);
    cloud.points[i].z = v(2);
    i++;
  }
  return cloud;
}

pcl::PCLPointCloud2 construct_voxelize(pcl::PointCloud<pcl::PointXYZ> cloud){
  pcl::PCLPointCloud2::Ptr cloud_partial(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(cloud, *cloud_partial);
  pcl::concatenatePointCloud(*cloud_full, *cloud_partial, *cloud_full);
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud(cloud_full);
  vg.setLeafSize(0.05f, 0.05f, 0.05f);
  vg.filter(*cloud_full);

  return *cloud_full;
}

void callback(const sensor_msgs::LaserScanPtr& scan)
{
  geometry_msgs::TransformStamped odom;
  try{
    odom = tfbuffer.lookupTransform("odom", "front_laser", ros::Time(0));
  } catch(tf2::TransformException &ex){
    ROS_WARN("%s", ex.what());
    ros::Duration(0.1).sleep();
  }
  Eigen::MatrixXd T = create_transform(odom);  
  auto cloud_partial = transform_to_cloud(scan, T);
  auto cloud_full = construct_voxelize(cloud_partial);
  cloud_pub.publish(cloud_full);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_to_cloud");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tf_listener(tfbuffer);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  ros::Subscriber scan_sub = nh.subscribe("/front/scan", 1, callback);
  ros::spin();
  pcl::PCDWriter obj;
  obj.writeBinary("cloud.pcd", *cloud_full);
}
