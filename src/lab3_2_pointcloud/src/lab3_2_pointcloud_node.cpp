#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

static sensor_msgs::PointCloud velodine_cloud;
static bool flag = false;

void get_point_cloud(const sensor_msgs::PointCloud &pc){
  velodine_cloud = pc;
  flag = true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "lab3_2_pointcloud_node");
  ros::NodeHandle nh;
  ros::NodeHandle n_private("~");
  double global_resolution = 0;
  double global_height_map = 0;
  double global_width_map = 0;

  ros::Subscriber pc_su = nh.subscribe ("/point_cloud", 10, get_point_cloud);
  ros::Publisher pass_map_pub = nh.advertise <nav_msgs::OccupancyGrid> ("/velodine_occup_map", 50);

  tf::TransformBroadcaster bcaster;

  ros::Time current_time;

  double max_barrier = -1.85;

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
  passability_map.info.resolution = global_resolution;  // size of cell (meters)
  passability_map.info.width = global_width_map;    // cells
  passability_map.info.height = global_height_map;    // cells
  passability_map.info.origin.position.x = - passability_map.info.resolution * passability_map.info.width  / 2;
  passability_map.info.origin.position.y = - passability_map.info.resolution * passability_map.info.height / 2;
  passability_map. data.resize(passability_map.info.width * passability_map.info.height);

  // Заполнение КП серым цветом
  for(unsigned long i = 0; i < (passability_map.info.width * passability_map.info.height); i++){
    passability_map.data.at(i) = 50;
  }

  std::vector <geometry_msgs::Point32> points;                // Вектор с точками
  std::vector <geometry_msgs::Point32> points_obstacle;       // Вектор непроходимых точек
  std::vector <geometry_msgs::Point32> points_obstacle_cells; // в масштабе карты проходимости

  ros::Rate r(10);

  while(ros::ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    //Формируем кватернион на основе значения угла курса
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(0.0);

    //Формируем сообщение, содержащее трансформацию систем координат
    geometry_msgs::TransformStamped trans;
    trans. header.stamp = ros::Time::now();
    trans. header.frame_id = "base_link";
    trans. child_frame_id = "laser";
    trans. transform.translation.x = 0.0;
    trans. transform.translation.y = 0.0;
    trans. transform.translation.z = 0.0;
    trans. transform.rotation = quat;

    bcaster.sendTransform(trans);

    if(flag){
      // Очистить все ячейки карты
      for(unsigned long i = 0; i < (passability_map.info.width * passability_map.info.height); i++){
        passability_map.data.at(i) = 50;
      }

      points_obstacle_cells.clear();
      points_obstacle.clear();
      points.clear();

      // Проверка проходимости
      for(unsigned long i = 0; i < velodine_cloud.points.size(); i++){
        if(velodine_cloud.points.at(i).z > max_barrier){
          points_obstacle.push_back(velodine_cloud.points.at(i));
        }
      }
      points_obstacle_cells.resize(points_obstacle.size());// Масштаб КП

      for(unsigned long i = 0; i < points_obstacle.size(); i++) {
        int x_obstacle_cell = points_obstacle.at(i).x / double(global_resolution) + passability_map.info.width/2;
        int y_obstacle_cell = points_obstacle.at(i).y / double(global_resolution) + passability_map.info.width/2;
        int z_obstacle_cell = points_obstacle.at(i).z / double(global_resolution) + passability_map.info.width/2;

        unsigned int index_obstacle = x_obstacle_cell + y_obstacle_cell * passability_map.info.width;

        if(x_obstacle_cell >= 0 &&  x_obstacle_cell < passability_map.info.width && y_obstacle_cell >= 0 &&  y_obstacle_cell < passability_map.info.width){
          passability_map.data.at(index_obstacle) = 100;
        }
      }
      //перебор по лучу
      double theta_inc = 2 * M_PI / (velodine_cloud.points.size() / 32);

      for(float theta = 0.0; theta < 2*M_PI; theta += theta_inc){
        double range_step = 1.0;

        while(range_step < passability_map.info.width/2){
          range_step += global_resolution;
          int y_free = range_step * cos(theta) / double(global_resolution) + passability_map.info.width  / 2;
          int x_free = range_step * sin(theta) / double(global_resolution) + passability_map.info.height / 2;

          unsigned int index_free = x_free + y_free * passability_map.info.width;

          if(x_free >= 0 &&  x_free < passability_map.info.width && y_free >= 0 &&  y_free < passability_map.info.width){
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
