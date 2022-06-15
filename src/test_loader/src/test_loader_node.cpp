#include <ros/ros.h>
#include "opencv2/core/core.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//void imageCallback(const sensor_msgs::Image& msg)
//{
//  cv::Mat img;
//  cv_bridge::CvImagePtr img_bridge;
//  img_bridge = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8);
//  img = img_bridge -> image;
//  ROS_INFO("Image size %d x %d", img.cols, img.rows);
//}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_loader_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string path;
  if(nh_private.hasParam("path")){
    nh_private.getParam("path", path);
    ROS_INFO("path is %s", path.c_str());
  }
  else {
    ROS_ERROR("Failed to read path parameter");
    return 1;
  }

  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/img_test", 16);

  cv::Mat img;                    //создаём объект
  img = cv::imread(path);         // записываем в него параметр
  cv_bridge::CvImage img_bridge;  //конвертер
  std_msgs::Header header;
  header.frame_id = "optical";    //система кооринат с камеры
  sensor_msgs::Image img_msg;
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
  img_bridge.toImageMsg(img_msg); //преобразование из типов img_bridge и img_msg в тип sensor_msgs.Image
  ros::Rate r(10);
  while(ros::ok()) {
    img_pub.publish(img_msg);
    ros::spinOnce();
    r.sleep();
  }


}
