#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <math.h>

static double odom_x = 0.0;
static double odom_y = 0.0;
static double odom_z = 0.0;
static geometry_msgs::Quaternion odom_quat;
static nav_msgs::OccupancyGrid local_map;
static int global_height_map = 0;
static int global_weight_map = 0;
static float global_resolution = 0;

static bool flag = false;

void get_local_map(const nav_msgs::OccupancyGrid &lmap){
  local_map = lmap;
  flag = true;
}

void get_odom(const nav_msgs::Odometry &odom){
  odom_x = odom.pose.pose.position.x;
  odom_y = odom.pose.pose.position.y;
  odom_z = odom.pose.pose.position.z;
  odom_quat.w = odom.pose.pose.orientation.w;
  odom_quat.x = odom.pose.pose.orientation.x;
  odom_quat.y = odom.pose.pose.orientation.y;
  odom_quat.z = odom.pose.pose.orientation.z;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "lab2_2_map_node");
  ros::NodeHandle n_private("~");
  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe ("/odom", 50, get_odom);
  ros::Subscriber local_map_sub = nh.subscribe ("/local_map", 50, get_local_map);
  ros::Publisher global_map_pub = nh.advertise <nav_msgs::OccupancyGrid> ("/global_map", 50);

  tf::TransformBroadcaster laser_broadcaster;
  tf::TransformBroadcaster base_link_broadcaster;
  tf::TransformBroadcaster odom_broadcaster;

  //Инициализация параметров временных штампов
  ros::Time current_time, last_time;

  current_time = ros::Time::now();
  last_time    = ros::Time::now();
  ros::Rate r(50);

  if(n_private.hasParam("height_map")){
    n_private.getParam("height_map", global_height_map);
  }
  else {global_height_map = 120;}
  if(n_private.hasParam("resolution")){
    n_private.getParam("resolution", global_resolution);
  }
  else {global_resolution = 0.1;}
  if(n_private.hasParam("weight_map")){
    n_private.getParam("weight_map", global_weight_map);
  }
  else {global_weight_map = 120;}


  nav_msgs::OccupancyGrid global_map;
  global_map.header.frame_id = "/map";
  global_map.header.stamp = current_time;
  global_map.info.resolution = global_resolution;
  global_map.info.width = global_weight_map;
  global_map.info.height = global_height_map;
  global_map.info.origin.position.x = - global_map.info.resolution * global_map.info.width  / 2;
  global_map.info.origin.position.y = - global_map.info.resolution * global_map.info.height / 2;

  global_map.data.resize(global_map.info.width * global_map.info.height);
  ROS_INFO("%d", global_map.info.width);
  ROS_INFO("%d", global_map.info.height);
  ROS_INFO("%f", global_map.info.resolution);
  for(unsigned long i = 0; i < (global_map.info.width * global_map.info.height); i++){
    global_map.data.at(i) = 50;
  }

  while(nh.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();

    if(flag){
      //Формируем кватернион на основе значения угла курса
      geometry_msgs::Quaternion laser_quat = tf::createQuaternionMsgFromYaw(0.0);

      //Трансформация систем координат base_link <- laser
      geometry_msgs::TransformStamped laser_trans;
      laser_trans.header.stamp = current_time;
      laser_trans.header.frame_id = "base_link";
      laser_trans.child_frame_id = "laser";
      laser_trans.transform.translation.x = 0.24;
      laser_trans.transform.translation.y = 0.0;
      laser_trans.transform.translation.z = 0.0;
      laser_trans.transform.rotation = laser_quat;

      laser_broadcaster.sendTransform(laser_trans);

      //Трансформация систем координат odom <- base_link
      geometry_msgs::TransformStamped base_link_trans;
      base_link_trans. header.stamp = current_time;
      base_link_trans. header.frame_id = "odom";
      base_link_trans. child_frame_id = "base_link";
      base_link_trans. transform.translation.x = odom_x;
      base_link_trans. transform.translation.y = odom_y;
      base_link_trans. transform.translation.z = odom_z;
      base_link_trans. transform.rotation = odom_quat;

      base_link_broadcaster.sendTransform(base_link_trans);

      //Трансформация систем координат map <- odom
      geometry_msgs::TransformStamped odom_trans;
      odom_trans. header.stamp = current_time;
      odom_trans. header.frame_id = "map";
      odom_trans. child_frame_id = "odom";
      odom_trans. transform.translation.x = 0.0;
      odom_trans. transform.translation.y = 0.0;
      odom_trans. transform.translation.z = 0.0;
      odom_trans. transform.rotation = laser_quat;

      odom_broadcaster.sendTransform(odom_trans);
      ROS_INFO("%d", local_map.info.width);
      ROS_INFO("%d", local_map.info.height);
      ROS_INFO("%f", local_map.info.resolution);

      double K = 0.95;

      for(unsigned long i = 0; i < (local_map.info.width * local_map.info.height); i++)
      {
        if(!(local_map.data.at(i) == 50))
        {
          int x = i/local_map.info.width;
          int y = i - x * local_map.info.height;
          float new_x = global_map.info.resolution * ((float)x - local_map.info.width  / 2);
          float new_y = global_map.info.resolution * ((float)y - local_map.info.height  / 2);

          Eigen::Quaternionf q_e;
          q_e.x() = odom_quat.x;
          q_e.y() = odom_quat.y;
          q_e.z() = odom_quat.z;
          q_e.w() = odom_quat.w;
          auto euler = q_e.toRotationMatrix().eulerAngles(0, 1, 2);
          double new_theta = euler(2);

          double glob_x = new_x * cos(new_theta) + new_y * sin(new_theta);
          double glob_y = -new_x * sin(new_theta) + new_y * cos(new_theta);

          int new_i = (glob_x + odom_y) / global_map.info.resolution + local_map.info.width  / 2;
          int new_j = (glob_y + odom_x) / global_map.info.resolution + local_map.info.height  / 2;

          int glob_i = new_j + new_i * global_map.info.width;

          global_map.data.at(glob_i) = K * global_map.data.at(glob_i) + (1 - K) * local_map.data.at(i);
        }
      }
      global_map_pub.publish(global_map);
      flag = false;
    }
    r.sleep();
  }
}
