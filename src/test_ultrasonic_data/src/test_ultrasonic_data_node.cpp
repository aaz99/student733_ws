#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>

static float range = 0;
void range_msgs_callback(const sensor_msgs::Range &msg)
{
  ROS_INFO("Ultrasonic: [%s %d] | %d | %f | %f | %f | %f",
           msg.header.frame_id.c_str(),
           msg.header.seq,
           msg.radiation_type,
           msg.field_of_view,
           msg.min_range,
           msg.max_range,
           msg.range);

  range = msg.range;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_ultrasonic_data_node");
  ros::NodeHandle nh;

  ros::Subscriber range_msgs_sub = nh.subscribe("ultrasonic/data", 5, range_msgs_callback);
  ros::Publisher range_msgs_pub = nh.advertise<sensor_msgs::Range>("range_data", 5);

  while(ros::ok())
  {
    sensor_msgs::Range range_msg;

    range_msg.header.frame_id = "my_ultrasonic";
    range_msg.header.stamp=ros::Time::now();
    range_msg.radiation_type = 0;
    range_msg.field_of_view = 0.2;
    range_msg.min_range = 0.0;
    range_msg.max_range = 2.55;
    range_msg.range = range;

    range_msgs_pub.publish(range_msg);
    ros::spinOnce();

  }
  ros::spin();
  return 0;

}
