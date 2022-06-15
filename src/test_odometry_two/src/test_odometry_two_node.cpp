#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <math.h>

static double d = 0;
static double D = 0;
static double move = 0;
static double move_prev = 0;
static double angle = 0;
static double angle_2 = 0;
static double wheel_range = 0.69;
static double Rh = 0;
static double angle_2_prev = 0;
static double vx = 0;
static double vy = 0;


static double delta_x = 0;
static double delta_y = 0;

static bool flag_e1 = false;

void sub_wheel(const sensor_msgs::JointState &joint_state){
  flag_e1 = true;
  D = joint_state.velocity.at(1);
  d += D;
 angle = (joint_state.position.at(0)) * 11 / 100;
   move = d / 33000;
}


int main(int argc, char** argv){
  //Инициализация параметров узла
 ros::init(argc, argv, "test_odometry_two_node");
  ros::NodeHandle n;
  //ros::NodeHandle nh;

  ros::Subscriber encoder_1_sub = n.subscribe("/joint_states", 500, sub_wheel);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 500);
  tf::TransformBroadcaster odom_broadcaster;

  //Инициализация параметров пространственного положения
  double x = 0.0;
  double y = 0.0;


  //Инициализация параметров временных штампов
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50.0);

  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    if(flag_e1){
      Rh = wheel_range / tan(angle * M_PI / 180.0);

      double delta_th = ( move -  move_prev) / Rh;
      angle_2 =  angle_2_prev + delta_th;

      double dt = (current_time - last_time).toSec();
      vx = (move - move_prev) / dt;
      delta_x = vx * cos(angle_2) * dt;
      delta_y = vx * sin(angle_2) * dt;
      x += delta_x;
      y += delta_y;

      //Формируем кватернион на основе значения угла курса
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle_2);

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
      move_prev  = move;
      angle_2_prev = angle_2;
      last_time  = current_time;

      flag_e1 = false;
    }
    r.sleep();

  }
}
