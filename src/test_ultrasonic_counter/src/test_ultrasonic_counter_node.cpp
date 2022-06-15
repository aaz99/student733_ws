#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

static int objects_count = 0;
static bool curr_obj_state = false;
static bool prev_obj_state = false;

int observe_object(bool prev, bool curr)
{
  if((prev == true) && (curr == false)){return 1;}
  else {return 0;}
}
void objects_count_callback(sensor_msgs::Range us_data){
  if(us_data.range < 1) {curr_obj_state = true;}
  else {curr_obj_state = false;}
  objects_count += observe_object(prev_obj_state, curr_obj_state);
  prev_obj_state = curr_obj_state;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_ultrasonic_counter_node");
  ros::NodeHandle nh;
  ros::Subscriber range_data_sub = nh.subscribe("range_data", 5, objects_count_callback);
  ros::Publisher objects_count_pub = nh.advertise<std_msgs::Int32>("objects_count", 5);

  while(ros::ok()){
    std_msgs::Int32 objects_count_msg;
    objects_count_msg.data = objects_count;
    ROS_INFO("%d", objects_count);
    objects_count_pub.publish(objects_count_msg);
    ros::spinOnce();
  }
  ros::spin();
  return 0;


}
