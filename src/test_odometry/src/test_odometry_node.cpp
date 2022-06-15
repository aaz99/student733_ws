#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <math.h>

const double r = 0.03;
const double round_counts_wheel = 20;
const double w = 0.12;
static double d_1 = 0;
static double d_2 = 0;
static double move = 0;
static double angle = 0;
static double old_angle = 0;
static double delta_x = 0;
static double delta_y = 0;

static double enc_1 = 0;
static double enc_2 = 0;

static double enc_1_delt = 0;
static double enc_2_delt = 0;

bool flag_e1 = false;
bool flag_e2 = false;


void wheel_1(const std_msgs::Int32& msg1){
  flag_e1 = true;
  enc_1_delt = msg1.data-enc_1;
  enc_1 = msg1.data;
  d_1 = (2 * M_PI * r * enc_1_delt) / round_counts_wheel;
}

void wheel_2(const std_msgs::Int32& msg2){
  flag_e2 = true;
  enc_2_delt = msg2.data - enc_2;
  enc_2 = msg2.data;
  d_2 = (2 * M_PI * r * enc_2_delt) / round_counts_wheel;
}

int main(int argc, char** argv){
  //Инициализация параметров узла
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  //ros::NodeHandle nh;

  ros::Subscriber encoder_1_sub = n.subscribe("encoder_1", 500, wheel_1);
  ros::Subscriber encoder_2_sub = n.subscribe("encoder_2", 500, wheel_2);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 500);
  tf::TransformBroadcaster odom_broadcaster;

  //Инициализация параметров пространственного положения
  double x = 0.0;
  double y = 0.0;
  double sum_move = 0;
  double sum_angle = 0;

  //Инициализация параметров временных штампов
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100.0);

  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    if((flag_e1 && flag_e2) == true){

      move = (d_1 + d_2) / 2;
      angle = (d_1 - d_2) / w;
      old_angle += angle;
      sum_move += move;
      sum_angle += angle;
      ROS_INFO("move: [%f]", sum_move);
      ROS_INFO("angle: [%f]", sum_angle);

      //Вычисление параметров перемещения наоснове известных значений скорости движения робота
      double dt   = (current_time - last_time).toSec();
      delta_x = move*cos(old_angle);
      delta_y = move*sin(old_angle);

      x += delta_x;
      y += delta_y;

      //Формируем кватернион на основе значения угла курса
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(old_angle);

      //Формируем сообщение, содержащее трансформацию систем координат
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //Отправляем сообщение
      odom_broadcaster.sendTransform(odom_trans);

      //Формируемсообщение, содержащее параметры одометрии
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //Параметрыположения
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //Параметры скорости
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = 0;
      odom.twist.twist.linear.y = 0;
      odom.twist.twist.angular.z = 0;

      //Публикуем сообщение с одометрией
      odom_pub.publish(odom);
      last_time = current_time;

      flag_e1 = false;
      flag_e2 = false;



    }
    r.sleep();

  }
}
