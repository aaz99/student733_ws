#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
  int n = 3;
  ros::init(argc, argv, "test_transform_broadcaster_node");
  ros::NodeHandle nh;

  tf::TransformBroadcaster br_laser;
  tf::StampedTransform transform_laser;

  tf::TransformBroadcaster br_ultrasonic;
  tf::StampedTransform transform_ultrasonic;

  tf::TransformBroadcaster br_base_footprint;
  tf::StampedTransform transform_base_footprint;

  transform_laser.setOrigin(tf::Vector3(n*0.1,1/n,0.05+0.2*n));
  tf::Quaternion l;
  l.setRPY(0.57*n,0.2,0);
  transform_laser.setRotation(l);
  transform_laser.frame_id_ = "/base_link";
  transform_laser.child_frame_id_ = "/laser";

  transform_ultrasonic.setOrigin(tf::Vector3(n*0.1,1/n,0.15-0.2*n));
  tf::Quaternion u;
  u.setRPY(0.57*n,0.2,0);
  transform_ultrasonic.setRotation(u);
  transform_ultrasonic.frame_id_ = "/base_link";
  transform_ultrasonic.child_frame_id_ = "/ultrasonic";

  transform_base_footprint.setOrigin(tf::Vector3(0.1,0.0,0.4));
  tf::Quaternion b;
  b.setRPY(0,0,0);
  transform_base_footprint.setRotation(b);
  transform_base_footprint.frame_id_ = "/base_link";
  transform_base_footprint.child_frame_id_ = "/base_footprint";

  ros::Rate r(10);

  while(ros::ok()){
    transform_laser.stamp_ = ros::Time::now();
    br_laser.sendTransform(transform_laser);

    transform_ultrasonic.stamp_ = ros::Time::now();
    br_ultrasonic.sendTransform(transform_ultrasonic);

    transform_base_footprint.stamp_ = ros::Time::now();
    br_base_footprint.sendTransform(transform_base_footprint);

    ros::spinOnce();
    r.sleep();
  }

}
