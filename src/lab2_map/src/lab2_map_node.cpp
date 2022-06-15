#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

static float range=0;
static float teta_min=0;
static float teta_inc=0;
static float teta=0;
static int j_max=0;
static double x=0;
static double X=0;
static double y=0;
static double Y=0;
static int i_max=0;
static int N = 0;
static bool flag = false;
std::vector<float> Range;

static int global_height_map = 0;
static int global_weight_map = 0;
static float global_resolution = 0;

void Callback(const sensor_msgs::LaserScan& msg)
{
  teta_min = msg.angle_min;
  teta_inc = msg.angle_increment;

  Range = msg.ranges;
  i_max = msg.ranges.size();
  j_max = ((-msg.angle_min + msg.angle_max)/teta_inc)+1; //количество положений лазера по горизонтали

  flag = true;
}

int main(int argc, char** argv){
  //Инициализация параметров узла
  ros::init(argc, argv, "lab2_map_node");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  ros::Subscriber sub = n.subscribe("/scan", 100, Callback);
  ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("local_map", 50);
  tf::TransformBroadcaster odom_broadcaster;

  //Инициализация параметров временных штампов
  ros::Time current_time, last_time;
  last_time = ros::Time::now();
  current_time = ros::Time::now();
  if(nh_private.hasParam("height_map"))
  {
    nh_private.getParam("height_map", global_height_map);
  }
  else {global_height_map = 120;}
  if(nh_private.hasParam("resolution"))
  {
    nh_private.getParam("resolution", global_resolution);
  }
  else {global_resolution = 0.1;}
  if(nh_private.hasParam("weight_map"))
  {
    nh_private.getParam("weight_map", global_weight_map);
  }
  else {global_weight_map = 120;}
  ROS_INFO("%d", global_height_map);
  ROS_INFO("%d", global_weight_map);
  ROS_INFO("%f", global_resolution);



  //Инициализация local_map
  nav_msgs::OccupancyGrid local_map;
  local_map.header.stamp = current_time;
  local_map.header.frame_id = "map";
  local_map.info.resolution = global_resolution;
  local_map.info.width = global_weight_map;
  local_map.info.height = global_height_map;
  local_map.info.origin.position.x = -global_resolution*global_height_map/2;
  local_map.info.origin.position.y = -global_resolution*global_weight_map/2;
  local_map.data.resize(global_weight_map*global_weight_map);

  //заполнение карты серыми значениями
  for(int i = 0; i < global_weight_map; i++)
  {
    for(int j = 0; j < global_height_map; j++)
    {
      local_map.data[j * global_height_map + i] = 50;
    }
  }

  ros::Rate r(20.0);

  while(n.ok()){
    ros::spinOnce();
    if(flag == true){
      //заполняем карту серыми значениями
      for(int i = 0; i < global_height_map; i++)
      {
        for(int j = 0; j < global_height_map; j++){local_map.data[j * global_height_map + i] = 50;}
      }
      for(int i = 0; i < i_max; i++)
      {
        range = Range[i];
        if(range<6){
          //вычисление координат препятствий и заполнение их черным цветом
          teta = teta_min+(i*teta_inc);
          x = range * cos(teta);
          int x_cell = x/global_resolution+global_height_map/2;
          y = range * sin(teta);
          int y_cell = y/global_resolution+global_weight_map/2;
          int index = (x_cell + y_cell * local_map.info.width);
          local_map.data[index] = 100;
          N = range/global_resolution;
          //трассировка луча
          for(int n = 0; n < N-1; n++){
            X = n*global_resolution * sin(teta);
            Y = n*global_resolution * cos(teta);
            int cell_x = X/global_resolution+global_height_map/2;
            int cell_y = Y/global_resolution+global_weight_map/2;
            int ind = (cell_y + cell_x * global_weight_map);
            local_map.data[ind] = 0;
          }
        }
      }
      flag = false;
    }

    pub.publish(local_map);
    r.sleep();
  }
}
