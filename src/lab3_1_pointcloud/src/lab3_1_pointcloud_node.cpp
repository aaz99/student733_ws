#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

static sensor_msgs::PointCloud2 kinect_cloud;
static bool flag = false;

void to_point_cloud(const sensor_msgs::PointCloud2 &pCloud, const int u, const int v, geometry_msgs::Point32 &p)
{
  //Вычисляем начало позиции элемента
  int arrayPosition = v * pCloud.row_step + u * pCloud.point_step;

  // Вычисляем сдвиг каждого из полей
  int arrayPosX = arrayPosition + pCloud.fields[0].offset;
  int arrayPosY = arrayPosition + pCloud.fields[1].offset;
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset;

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  p.x = X;
  p.y = Z;
  p.z = -Y;
}

void get_point_cloud(const sensor_msgs::PointCloud2 &pc)
{
  kinect_cloud = pc;
  flag = true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "lab3_1_pointcloud_node");
  ros::NodeHandle nh;
  ros::NodeHandle n_private("~");

  ros::Subscriber pc_sub = nh.subscribe ("/camera/depth/points", 10, get_point_cloud);
  ros::Publisher  pass_map_pub = nh.advertise <nav_msgs::OccupancyGrid> ("/occupancy_map", 50);
  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud>("/lab_point_cloud",10);

  tf::TransformBroadcaster bcaster;
  ros::Time current_time;

  //double pi = M_PI;
  double max_barrier = -0.21;
  double global_resolution = 0;
  double global_height_map = 0;
  double global_width_map = 0;

  if(n_private.hasParam("height_map")){
    n_private.getParam("height_map", global_height_map);
  }
  else {global_height_map = 120;}
  if(n_private.hasParam("resolution")){
    n_private.getParam("resolution", global_resolution);
  }
  else {global_resolution = 0.1;}
  if(n_private.hasParam("width_map")){
    n_private.getParam("width_map", global_width_map);
  }
  else {global_width_map = 120;}

  nav_msgs::OccupancyGrid passability_map;
  passability_map.header.frame_id = "/laser";
  passability_map.header.stamp = ros::Time::now();
  passability_map.info.resolution = global_resolution;   // size of cell (meters)
  passability_map.info.width = global_width_map;   // cells
  passability_map.info.height = global_height_map;   // cells
  passability_map.info.origin.position.x = - passability_map.info.resolution * passability_map.info.width  / 2;
  passability_map.info.origin.position.y = - passability_map.info.resolution * passability_map.info.height / 2;

  passability_map.data.resize(passability_map.info.width * passability_map.info.height);


  for(unsigned long i = 0; i < (passability_map.info.width * passability_map.info.height); i++)
  {
    passability_map.data.at(i) = 50;//заполнение КП серыми точками
  }

  std::vector <geometry_msgs::Point32> points;
  std::vector <geometry_msgs::Point32> points_obstacle; // Вектор непроходимых точек
  std::vector <geometry_msgs::Point32> points_obstacle_cells; // в масштабе карты проходимости
  std::vector <geometry_msgs::Point32> points_free_cells;

  ros::Rate r(10);

  while(ros::ok())
  {
    ros::spinOnce();
    current_time = ros::Time::now();
    //Формируем кватернион на основе значения угла курса
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,M_PI);

    //Формируем сообщение, содержащее трансформацию систем координат
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = "depth_pointcloud";
    trans.child_frame_id = "laser";
    trans.transform.translation.x = 0.0;
    trans.transform.translation.y = 0.0;
    trans.transform.translation.z = 0.0;
    trans.transform.rotation = quat;

    bcaster.sendTransform(trans);

    if(flag){
      //очистка КП
      for(unsigned long i = 0; i < (passability_map.info.width * passability_map.info.height); i++){
        passability_map.data.at(i) = 50;
      }
      points_obstacle_cells.clear();
      points_obstacle.clear();
      points.clear();
      sensor_msgs::PointCloud pointCloud_1;

      // Получение координат точек из облака точек
      for(unsigned long i = 0; i < kinect_cloud.width; i++){
        for(unsigned long j = 0; j < kinect_cloud.height; j++){
          geometry_msgs::Point32 point;

          to_point_cloud(kinect_cloud, i, j, point);

          pointCloud_1.points.push_back(point);
          points.push_back(point);
        }
      }

      pointCloud_1.header.frame_id = "/laser";
      pointCloud_1.header.stamp = ros::Time::now();
      pc_pub.publish(pointCloud_1);

      // Проверка на проходимость
      for(unsigned long i = 0; i < points.size(); i++){
        if(points.at(i).z > max_barrier){
          points_obstacle.push_back(points.at(i));
        }
      }

      // изменение масштаба КП
      points_obstacle_cells.resize(points_obstacle.size());

      for(unsigned long i = 0; i < points_obstacle.size(); i++){
        int x_obstacle_cell = points_obstacle.at(i).x / double(global_resolution) + passability_map.info.width/2;
        int y_obstacle_cell = points_obstacle.at(i).y / double(global_resolution) + passability_map.info.width/2;

        unsigned int index_obstacle = x_obstacle_cell + y_obstacle_cell * passability_map.info.width;

        if(index_obstacle > 0 && index_obstacle < passability_map.info.width * passability_map.info.width){
          passability_map.data.at(index_obstacle) = 100;
        }
      }
      // Перебор по лучу 56 градусов
      double theta_min = -1.0118 / 2;
      double theta_inc = 0.001581;
      for(int i = 0; i < 640; i++){
        double theta_curr = theta_min + i * theta_inc;
        double range_step = 0.0;

        while(range_step < passability_map.info.width){
          range_step += global_resolution;

          int y_free = range_step * cos(theta_curr) / double(global_resolution) + passability_map.info.width  / 2;
          int x_free = range_step * sin(theta_curr) / double(global_resolution) + passability_map.info.height / 2;

          unsigned int index_free = x_free + y_free * passability_map.info.width;

          if(index_free >= 0 && index_free < passability_map.info.width * passability_map.info.width){
            if(passability_map.data.at(index_free) < 100){
              passability_map.data.at(index_free) = 0;
            }
            else{
              break;
            }
          }
        }
      }
      flag = false;
    }
    pass_map_pub.publish(passability_map);
    r.sleep();
  }
}
