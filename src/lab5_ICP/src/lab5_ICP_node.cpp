#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

static bool flag = false;

static sensor_msgs::LaserScan ls;
static nav_msgs::Odometry odometry;

void get_scan(const sensor_msgs::LaserScan &scan){
  ls = scan;
  flag = true;
}
void get_odom(const nav_msgs::Odometry &odom){
  odometry = odom;
}

struct filteredLaserScan{
  std::vector <float> ranges;
  std::vector <float> thetas;
};

void filterInfsLaserScan (const sensor_msgs::LaserScan &ls, filteredLaserScan &new_ls){
  for(unsigned long i = 0; i < ls.ranges.size(); i++){
    float th_curr = ls.angle_min + i * ls.angle_increment;

    if(ls.ranges.at(i) < ls.range_max && ls.ranges.at(i) > 0.25){
      new_ls.ranges.push_back(ls.ranges.at(i));
      new_ls.thetas.push_back(th_curr);
    }
  }
}

void laserScan2PointCloud (const filteredLaserScan &ls, const nav_msgs::Odometry &odom, sensor_msgs::PointCloud &pc){
  for(unsigned long i = 0; i < ls.ranges.size(); i++){
    geometry_msgs::Point32 point;

    point.x = ls.ranges.at(i) * sin(ls.thetas.at(i));
    point.y = ls.ranges.at(i) * cos(ls.thetas.at(i));
    point.z = 0.0;

    Eigen::Quaternionf q_e;
    q_e.x() = odom.pose.pose.orientation.x;
    q_e.y() = odom.pose.pose.orientation.y;
    q_e.z() = odom.pose.pose.orientation.z;
    q_e.w() = odom.pose.pose.orientation.w;
    auto euler = q_e.toRotationMatrix().eulerAngles(0, 1, 2);
    double theta = euler(2);

    float x =  point.x * cos(theta) + point.y * sin(theta);
    float y = -point.x * sin(theta) + point.y * cos(theta);

    point.x = y + odom.pose.pose.position.x;
    point.y = x + odom.pose.pose.position.y;

    pc.points.push_back(point);
  }
}

void laserScan2PCLPointCloud (const filteredLaserScan &ls, const nav_msgs::Odometry &odom, pcl::PointCloud<pcl::PointXYZ>::Ptr pc){
  pc->width = ls.ranges.size();
  pc->height = 1;
  pc->is_dense = false;
  pc->points.resize(pc->width * pc->height);

  Eigen::Quaternionf q_e;
  q_e.x() = float(odom.pose.pose.orientation.x);
  q_e.y() = float(odom.pose.pose.orientation.y);
  q_e.z() = float(odom.pose.pose.orientation.z);
  q_e.w() = float(odom.pose.pose.orientation.w);
  auto euler = q_e.toRotationMatrix().eulerAngles(0, 1, 2);
  float theta = euler(2);

  for (size_t i = 0; i < pc->points.size (); ++i){
    float x = ls.ranges.at(i) * float(sin(ls.thetas.at(i)));
    float y = ls.ranges.at(i) * float(cos(ls.thetas.at(i)));
    float z = 0.0;

    float x_rot =  x * cos(theta) + y * sin(theta);
    float y_rot = -x * sin(theta) + y * cos(theta);

    pc->points[i].x = y_rot + float(odom.pose.pose.position.x);
    pc->points[i].y = x_rot + float(odom.pose.pose.position.y);
    pc->points[i].z = z;
  }
}

void convertPCLpc_to_pc(const pcl::PointCloud<pcl::PointXYZ> pcl_pc, sensor_msgs::PointCloud &pc, bool clear){
  if(clear == true) {
    pc.points.clear();
  }

  for(size_t i = 0; i < pcl_pc.points.size (); ++i){
    geometry_msgs::Point32 point;

    point.x = pcl_pc.points[i].x;
    point.y = pcl_pc.points[i].y;
    point.z = pcl_pc.points[i].z;

    pc.points.push_back(point);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "lab5_icp_node");
  ros::NodeHandle nh;

  // subs
  ros::Subscriber odom_sub = nh.subscribe ("/odom", 10, get_odom);
  ros::Subscriber scan_sub = nh.subscribe ("/scan", 10, get_scan);

  // pubs
  ros::Publisher  prev_pc_pub = nh.advertise < sensor_msgs::PointCloud > ("/pc_prev", 10);
  ros::Publisher  curr_pc_pub = nh.advertise < sensor_msgs::PointCloud > ("/pc_curr", 10);
  ros::Publisher  final_pc_pub = nh.advertise < sensor_msgs::PointCloud > ("/pcl_pc_final", 10);
  ros::Publisher  final_transform_pub = nh.advertise < geometry_msgs::Pose2D > ("/final_transform", 10);

  // rate
  ros::Rate r(10);

  // vars
  int scan_iterator = 0;

  int max_iters = 0.0;
  double eps = 0.0;
  double corr_dist = 0.0;

  float tx = 0.0;
  float ty = 0.0;
  float yaw = 0.0;

  // private parameters
  ros::NodeHandle nh_private("~");

  if(nh_private.hasParam("maxiters")){
    nh_private.getParam("maxiters", max_iters);
  }
  if(nh_private.hasParam("eps")){
    nh_private.getParam("eps", eps);
  }
  if(nh_private.hasParam("corrdist")){
    nh_private.getParam("corrdist", corr_dist);
  }

  // create objects
  filteredLaserScan f_ls;
  sensor_msgs::PointCloud pc;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_prev (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_curr (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud pc_prev;
  sensor_msgs::PointCloud pc_curr;
  sensor_msgs::PointCloud pc_final;

  while(nh.ok()){
    ros::spinOnce();

    if(flag){
      // clear vectors
      f_ls.ranges.clear();
      f_ls.thetas.clear();
      pcl_pc_curr->clear();

      // поиск бесконечных значений
      filterInfsLaserScan(ls, f_ls);

      laserScan2PCLPointCloud(f_ls, odometry, pcl_pc_curr);

      // Сравниваем предыдущее и текущее облака точек
      if(scan_iterator > 1){
        // Создание объекта ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        // Настройка ICP
        icp.setMaximumIterations(max_iters);
        icp.setTransformationEpsilon(eps);
        icp.setMaxCorrespondenceDistance(corr_dist);

        icp.setInputSource(pcl_pc_curr);
        icp.setInputTarget(pcl_pc_prev);

        pcl::PointCloud<pcl::PointXYZ> pcl_pc_final;
        icp.align(pcl_pc_final);

        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        transformPointCloud(pcl_pc_final, pcl_pc_final, transformation);

        convertPCLpc_to_pc( *pcl_pc_prev,  pc_prev,  false );
        convertPCLpc_to_pc( *pcl_pc_curr,  pc_curr,  true );
        convertPCLpc_to_pc(  pcl_pc_final, pc_final, false );

        Eigen::Matrix4f finTrans = icp.getFinalTransformation();
        Eigen::Matrix3f Rot = finTrans.block <3, 3> (0, 0);

        Eigen::Vector3f euler = Rot.eulerAngles(0,1,2);

        float delta_yaw = euler(2);
        yaw += delta_yaw;

        float delta_tx = finTrans(0,3) * cos(yaw) - finTrans(1,3) * sin(yaw);
        float delta_ty = finTrans(1,3) * sin(yaw) + finTrans(0,3) * cos(yaw);

        tx += delta_tx;
        ty += delta_ty;

        geometry_msgs::Pose2D pose2D_finalTransf;
        pose2D_finalTransf.x = double(tx + odometry.pose.pose.position.x);
        pose2D_finalTransf.y = double(ty + odometry.pose.pose.position.y);
        pose2D_finalTransf.theta = double(yaw);

        ROS_INFO("[x,y,theta] = [%3.3f, %3.3f, %3.3f]", pose2D_finalTransf.x, pose2D_finalTransf.y, pose2D_finalTransf.theta);

        final_transform_pub.publish(pose2D_finalTransf);

        // Опубликовать текущее и предыдущее облака точек
        pc_prev.header.frame_id = "/odom";
        pc_prev.header.stamp = ros::Time::now();
        prev_pc_pub.publish(pc_prev);

        pc_curr.header.frame_id = "/odom";
        pc_curr.header.stamp = ros::Time::now();
        curr_pc_pub.publish(pc_curr);

        // Опубликовать результирующее облако точек
        pc_final.header.frame_id = "/odom";
        pc_final.header.stamp = ros::Time::now();
        final_pc_pub.publish(pc_final);
      }


      pcl_pc_prev->clear();


      *pcl_pc_prev = *pcl_pc_curr;
      scan_iterator++;

      flag = false;
    }

    ros::spinOnce();
    r.sleep();
  }
}
