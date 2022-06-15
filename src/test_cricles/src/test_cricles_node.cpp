#include <ros/ros.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <vector>
#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>

static sensor_msgs::Image exit_file;
static std_msgs::Int32 center_x;
static std_msgs::Int32 center_y;

static size_t max_position = 0;
void imageCallback(const sensor_msgs::Image& msg) {
  cv::Mat img, img_gray;
  cv_bridge::CvImagePtr img_bridge;                                               // создание объекта
  img_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);      //преобразование изображения рос в opencv
  img = img_bridge -> image;                                                      // загрузка в объект содержимое принятого сообщения то есть msg
  ROS_INFO("Image size %d x %d", img.cols, img.rows);                             //вывод размеров картинки на экран
  cvtColor(img, img_gray, CV_BGR2GRAY);                                           //преобразование в оттенки серого
  cv::GaussianBlur(img_gray, img_gray, cv::Size(9, 9),2 , 2);                     //размытие по Гауссу
  std::vector<cv::Vec3f>circles;                                                  //задание вектора окружности
  HoughCircles(img_gray, circles, CV_HOUGH_GRADIENT, 1, img_gray.rows/8, 200, 100,0 ,0 ); //поиск окружности

  int max_radius = 0;
  for(size_t i = 0; i < circles.size(); i++)
  {
    int radius = cvRound(circles[i][2]);
    if (radius > max_radius){
      max_radius = radius;
      max_position = i;
    }
  }
  //отрисовка круга
  cv::Point center(cvRound(circles[max_position] [0]), cvRound(circles[max_position] [1]));
  cv::circle(img, center, 3, cv::Scalar(180,255,180), -1, 8, 0);
  cv::circle(img, center, max_radius, cv::Scalar(50,100,50), 3, 8, 0);
  std_msgs::Header header;
  header.frame_id = "optical_exit";
  cv_bridge::CvImage exit_bridge;
  exit_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img); //обратное преобразование в топик
  exit_bridge.toImageMsg(exit_file);
  //центр найденного изображения
  center_x.data = center.x;
  center_y.data = center.y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lab6_circles_node");
  ros::NodeHandle nh;
  ros::Subscriber img_sub = nh.subscribe("/img_test", 16, imageCallback);
  ros::Publisher public_image  = nh.advertise<sensor_msgs::Image>("processed_image", 10);
  ros::Publisher public_cent_x = nh.advertise<std_msgs::Int32>("Xcenter", 10);
  ros::Publisher public_cent_y = nh.advertise<std_msgs::Int32>("Ycenter", 10);

  while(ros::ok())
  {
    public_image.publish(exit_file);
    public_cent_x.publish(center_x);
    public_cent_y.publish(center_y);
    ros::spinOnce();
  }
  ros::spin();
  return 0;
}

