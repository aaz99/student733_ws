#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

using namespace Eigen;

static bool flag = false;

static geometry_msgs::Pose2D pose2D_odom;
static geometry_msgs::Pose2D pose2D_final;

void get_final_transform(const geometry_msgs::Pose2D &finTrans){
  pose2D_final = finTrans;
  flag = true;
}
void get_odom(const nav_msgs::Odometry &odometry){
  tf::Quaternion odomQuat(
        odometry.pose.pose.orientation.x,
        odometry.pose.pose.orientation.y,
        odometry.pose.pose.orientation.z,
        odometry.pose.pose.orientation.w);

  tf::Matrix3x3 m(odomQuat);
  double odomRoll, odomPitch, odomYaw;
  m.getRPY(odomRoll, odomPitch, odomYaw);

  pose2D_odom.x = odometry.pose.pose.position.x;
  pose2D_odom.y = odometry.pose.pose.position.y;
  pose2D_odom.theta = odomYaw;
}

void filterCalman(const geometry_msgs::Pose2D &pose2D_odom_curr, const geometry_msgs::Pose2D &pose2D_odom_prev, const geometry_msgs::Pose2D &pose2D_final, float P_coef, float R_coef, float Q_coef, float G_coef, Eigen::Vector3f &est_x){
  // Матрица состояний А
  Matrix3f A;
  A <<  1, 0, 0,
        0, 1, 0,
        0, 0, 1;

  // Матрица управления В
  Matrix3f B;
  B <<  1, 0, 0,
        0, 1, 0,
        0, 0, 1;

  // Матрица P
  Matrix3f P;
  P <<  P_coef, 0,      0,
        0,      P_coef, 0,
        0,      0,      P_coef;

  // Матрица ковариации шума измерений
  Matrix3f R;
  R <<  R_coef, 0,      0,
        0,      R_coef, 0,
        0,      0,      R_coef;

  // Матрица ковариации шума процесса
  Matrix3f Q;
  Q <<  Q_coef, 0,      0,
        0,      Q_coef, 0,
        0,      0,      Q_coef;

  // Матрица усилений
  Matrix3f G;
  G <<  G_coef, 0,      0,
        0,      G_coef, 0,
        0,      0,      G_coef;

  // Единичная матрица
  Matrix3f I;
  I <<  1, 0, 0,
        0, 1, 0,
        0, 0, 1;

  // Оценки с использованием алгоритма ICP
  Eigen::Vector3f z;
  z(0) = float(pose2D_final.x);
  z(1) = float(pose2D_final.y);
  z(2) = float(pose2D_final.theta);

  // Управляющее воздействие
  Vector3f u;
  u(0) = float(pose2D_odom_curr.x - pose2D_odom_prev.x);
  u(1) = float(pose2D_odom_curr.y - pose2D_odom_prev.y);
  u(2) = float(pose2D_odom_curr.theta - pose2D_odom_prev.theta);

  // Этап предсказания
  est_x = A * est_x + B * u;
  P = A * P * A.transpose() + Q;

  // Этап коррекции
  Eigen::Matrix3f PR;
  PR = P + R;
  G = P * PR.inverse();
  est_x = est_x + G * (z - est_x);
  P = (I - G) * P;
}

int main(int argc, char **argv){
  // init
  ros::init(argc, argv, "lab6_kalman_node");
  ros::NodeHandle nh;

  // subs
  ros::Subscriber odom_sub = nh.subscribe ("/odom", 10, get_odom);
  ros::Subscriber final_transform_sub = nh.subscribe ("/final_transform", 10, get_final_transform);

  // pubs
  ros::Publisher PathICP_pub = nh.advertise < nav_msgs::Path > ("pathICP",  10);
  ros::Publisher PathOdom_pub = nh.advertise < nav_msgs::Path > ("pathOdom", 10);
  ros::Publisher PathKalm_pub = nh.advertise < nav_msgs::Path > ("pathKalm", 10);

  // vars
  int scan_iterator = 0;

  geometry_msgs::Pose2D pose2D_odom_prev;

  Eigen::Vector3f estX;
  estX(0) = 0.0;
  estX(1) = 0.0;
  estX(2) = 0.0;

  nav_msgs::Path pathICP;
  nav_msgs::Path pathOdom;
  nav_msgs::Path pathKalm;

  double rcoef = 0.0;
  double qcoef = 0.0;

  // private parameters
  ros::NodeHandle nh_private_1("~");

  if(nh_private_1.hasParam("rcoef")){
    nh_private_1.getParam("rcoef", rcoef);
  }
  if(nh_private_1.hasParam("qcoef")){
    nh_private_1.getParam("qcoef", qcoef);
  }

  // rate
  ros::Rate r(10);

  while(nh.ok()){
    ros::spinOnce();

    if(flag){
      if(scan_iterator > 1){
        filterCalman(pose2D_odom, pose2D_odom_prev, pose2D_final, float(0.5), float(rcoef), float(qcoef), float(0.0), estX);

        Vector3f pose2D_final_v;
        pose2D_final_v(0) = float(pose2D_final.x);
        pose2D_final_v(1) = float(pose2D_final.y);
        pose2D_final_v(2) = float(pose2D_final.theta);


        Vector3f pose2D_odom_v;
        pose2D_odom_v(0) = float(pose2D_odom.x);
        pose2D_odom_v(1) = float(pose2D_odom.y);
        pose2D_odom_v(2) = float(pose2D_odom.theta);

        geometry_msgs::PoseStamped poseProcICP;
        // Путь
        poseProcICP.pose.position.x = pose2D_final.x;
        poseProcICP.pose.position.y = pose2D_final.y;
        poseProcICP.pose.position.z = 0.0;
        pathICP.poses.push_back(poseProcICP);

         geometry_msgs::PoseStamped poseProcOdom;
        poseProcOdom.pose.position.x = pose2D_odom.x;
        poseProcOdom.pose.position.y = pose2D_odom.y;
        poseProcOdom.pose.position.z = 0.0;
        pathOdom.poses.push_back(poseProcOdom);

        poseProcOdom.pose.position.x = double(estX(0));
        poseProcOdom.pose.position.y = double(estX(1));
        poseProcOdom.pose.position.z = 0.0;
        pathKalm.poses.push_back(poseProcOdom);

        // publicate
        pathICP.header.stamp=ros::Time::now();
        pathKalm.header.stamp=ros::Time::now();
        pathOdom.header.stamp=ros::Time::now();

        pathICP.header.frame_id="/odom";
        pathKalm.header.frame_id="/odom";
        pathOdom.header.frame_id="/odom";

        PathICP_pub.publish(pathICP);
        PathKalm_pub.publish(pathKalm);
        PathOdom_pub.publish(pathOdom);
      }
      scan_iterator++;
      pose2D_odom_prev = pose2D_odom;

      flag = false;
    }
    ros::spinOnce();
    r.sleep();
  }
}
