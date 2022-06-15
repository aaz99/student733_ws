#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <numeric>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
static float Anglemin;
static float Angleinc;
static float Anglemax;
static float range_max;
static float range_min;
static bool flag = false;
std::vector<float> scan_vector;
static float range;

const float treshold = 1;
const float nd = 5;
const float bins = 8;
const float histdist = 0.8;
const float coefcorr = 0.85;
const float appearencecoef = 5;

static double odom_x = 0.0;
static double odom_y = 0.0;
static double odom_z = 0.0;

static int mass_size_new = 0;
static int mass_size_last = 0;

static geometry_msgs::Quaternion odom_quat;

struct Feature {
  geometry_msgs::Point32 point;
  std::vector <int> hist;
  float delta;
  unsigned long nbins;
  float step =2*delta/nbins;
  int appearence;
  int x_coord;
  int y_coord;
};

void scanInfoCallback(const sensor_msgs::LaserScan& scanmsg){
  Anglemin=scanmsg.angle_min;
  Anglemax=scanmsg.angle_max;
  Angleinc=scanmsg.angle_increment;
  scan_vector=scanmsg.ranges;
  range_max = scanmsg.range_max;
  range_min = scanmsg.range_min;
  flag = true;
}
void odomCallback(const nav_msgs::Odometry& odom){
  odom_x = odom.pose.pose.position.x;
  odom_y = odom.pose.pose.position.y;
  odom_z = odom.pose.pose.position.z;
  odom_quat.w = odom.pose.pose.orientation.w;
  odom_quat.x = odom.pose.pose.orientation.x;
  odom_quat.y = odom.pose.pose.orientation.y;
  odom_quat.z = odom.pose.pose.orientation.z;
  flag = true;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "lab4_uncommon_point_node");
  ros::NodeHandle n;
  int number=0;
  ros::Subscriber scan = n.subscribe("/base_scan", 50, scanInfoCallback);
  ros::Subscriber odom = n.subscribe("/base_odometry/odom", 50, scanInfoCallback);
  ros::Publisher odom_1_pub = n.advertise<nav_msgs::OccupancyGrid>("marker", 50);
  ros::Publisher pc_pub = n.advertise<sensor_msgs::PointCloud>("/kinect_cloud",50);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/marker_cloud",50);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time    = ros::Time::now();

  std::vector <Feature> GlobFeatures;//глобальные структуры
  std::vector <Feature> Features;//вектор структур

  ros::Rate r(10);
  //номер иттерации
  while(n.ok()){
    ros::spinOnce();
    if((flag) == true){
      current_time = ros::Time::now();
      std::vector <float> vec_x(scan_vector.size());
      std::vector <float> vec_y(scan_vector.size());

      std::vector <float> crit(scan_vector.size());

      sensor_msgs::PointCloud pointCloud_1;
      sensor_msgs::PointCloud crit_Cloud;
      Features.clear();
      GlobFeatures.clear();


      for( unsigned long i=0;i<scan_vector.size();i++){
        //receive x y of ranges
        geometry_msgs::Point32 point;
        pointCloud_1.points.clear();
        for( unsigned long i=0;i<scan_vector.size();i++){
          double theta = Anglemin + i * Angleinc;
          if(scan_vector.at(i) < range_max){
            //Для расчѐта пространственного положения точки, в которой было получено отражение лазерного излучения от объекта, можно использовать следующее соотношение
            vec_x.at(i) = scan_vector.at(i) * sin(theta);
            vec_y.at(i) = scan_vector.at(i) * cos(theta);
            point.x= vec_x.at(i);
            point.y= vec_y.at(i);
            point.z=0;
            Eigen::Quaternionf q_e;
            q_e.x() = odom_quat.x;
            q_e.y() = odom_quat.y;
            q_e.z() = odom_quat.z;
            q_e.w() = odom_quat.w;
            auto euler = q_e.toRotationMatrix().eulerAngles(0, 1, 2);
            double new_theta = euler(2);
            float x = point.x * cos(new_theta) + point.y * sin(new_theta);
            float y = -point.x * sin(new_theta) + point.y * cos(new_theta);

            point.x = x + odom_y;
            point.y = y + odom_x;
            pointCloud_1.points.push_back(point);
          }
        }
        pointCloud_1.header.frame_id = "marker";

        pointCloud_1.header.stamp = ros::Time::now();

        pc_pub.publish(pointCloud_1);
      }
      crit_Cloud.points.clear();
      crit.clear();
      //Нахожу критерий для точек
      for( unsigned long i=1;i<pointCloud_1.points.size()-1;i++){
        float criteria=sqrt(pow((pointCloud_1.points.at(i).x- pointCloud_1.points.at(i-1).x),2)-
                            pow((pointCloud_1.points.at(i).y-pointCloud_1.points.at(i-1).y),2)-
                            pow((pointCloud_1.points.at(i).z-pointCloud_1.points.at(i-1).z),2))+
            sqrt(pow((pointCloud_1.points.at(i+1).x-pointCloud_1.points.at(i).x),2)-
                 pow((pointCloud_1.points.at(i+1).y-pointCloud_1.points.at(i).y),2)-
                 pow((pointCloud_1.points.at(i+1).z-pointCloud_1.points.at(i).z),2))-
            2*sqrt(pow((pointCloud_1.points.at(i+1).x-pointCloud_1.points.at(i-1).x),2)-
                   pow((pointCloud_1.points.at(i+1).y-pointCloud_1.points.at(i-1).y),2)-
                   pow((pointCloud_1.points.at(i+1).z-pointCloud_1.points.at(i-1).z),2));
        //отсеиваю особые точки
        if (!(criteria!=criteria)&&(abs(criteria)>treshold)) {
          //добавляю особые точки в облако точек
          crit_Cloud.points.push_back(pointCloud_1.points.at(i));
          //добавляю индекс особой точки
          crit.push_back(i);
        }

      }
      std::vector <float> hist_points;
      for (unsigned long i=1;i<crit_Cloud.points.size();i++) {

        //добавляю границы поиска
        if ((crit.at(i)>5) && (crit.at(i)<pointCloud_1.points.size()-5)) {

          Feature feat;
          feat.delta=0.8;
          feat.nbins=8;
          feat.hist.resize(feat.nbins);
          feat.hist.clear();
          feat.appearence=0;
          feat.point=crit_Cloud.points.at(i);
          //создаю вектор 5 точек слева от особой тчоки и 5 справа
          for (int k=-nd;k<=nd;k++) {
            double theta=Anglemin+i*Angleinc;
            float points_hist =pointCloud_1.points.at(crit.at(i)+k).x/sin(theta);
            hist_points.push_back(points_hist);
            //ROS_INFO("hist=%f",points_hist);
          }
          std::vector <int> Histogram;
          //пределы для построения гистораммы
          float low_limit= hist_points.at(5)-feat.delta;
          float top_limit= hist_points.at(5)+feat.delta;
          //ROS_INFO("low=%f, high=%f",low_limit,top_limit);
          //пробегаюсь по бинам
          for (float b=low_limit;b<=top_limit;b+=0.2)
          {
            //ROS_INFO("Bin %f",b);
            std::vector <int> HISTP;
            //пробегаюсь по точкам
            for (int p=0;p<=10;p++) {
              //если точка находится в бине, добавляю ее в вектор
              if (hist_points.at(p)>b && hist_points.at(p)<b+0.2) {
                HISTP.push_back(hist_points.at(p));
              }
            }
            //ROS_INFO("HISTP size %ld",HISTP.size());
            //размер вектора в гистограмму
            Histogram.push_back(HISTP.size());
          }
          feat.hist=Histogram;
          Features.push_back(feat);
           visualization_msgs::MarkerArray markers;
          for (int appear_points_itterator=0;appear_points_itterator<Features.size();appear_points_itterator++) {

              //Объявлениеобъекта
              visualization_msgs::Marker marker;
              //Название локальной системы координат
              marker.header.frame_id = "marker";
              //Временная отметка
              marker.header.stamp = ros::Time();
              //Название множества маркеров
              marker.ns = "my_namespace";
              //Числовой идентификатор (должен быть уникальным)
              marker.id = appear_points_itterator;
              //Тип фигуры (в данном случае сфера)
              marker.type = visualization_msgs::Marker::SPHERE;
              //Режим визуализации (в данном случае добавление к существующим
              marker.action = visualization_msgs::Marker::ADD;
              //Параметры положения объекта
              marker.pose.position.x = Features.at(appear_points_itterator).point.x;
              marker.pose.position.y = Features.at(appear_points_itterator).point.y;
              marker.pose.position.z = Features.at(appear_points_itterator).point.z;
              //Параметры ориентации объекта
              marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
              //Размер объекта по каждому из измерений
              marker.scale.x = 0.1;
              marker.scale.y = 0.1;
              marker.scale.z = 0.1;
              //Цвет объекта -прозрачность
              marker.color.a = 1.0;
              //Цвет объекта по каждому из трёх каналов
              marker.color.r = 1.0;
              marker.color.g = 0.0;
              marker.color.b = 0.0;
              //ROS_INFO("x=%f.y=%f",GlobFeatures.at(appear_points_itterator).point.x,GlobFeatures.at(appear_points_itterator).point.y);
              markers.markers.push_back(marker);

          }
          marker_pub.publish(markers);
          // Features.clear();


          ROS_INFO("ft.size %ld",Features.size());
        }
      }
//      if (number==0){
//        GlobFeatures=Features;
//      }

//      else if(number >= 1){ // На последующих сканах ищем корреляцию
//        // Цикл по особым точкам на текущем скане
//        for(unsigned long feat_iter = 0; feat_iter < Features.size(); feat_iter++){
//          // Среднее значение гистограммы текущей особой точки с текущего скана
//          float hist_mean = std::accumulate(Features.at(feat_iter).hist.begin(), Features.at(feat_iter).hist.end(), 0) / Features.at(feat_iter).hist.size();

//          bool isFeatureFinded = false;
//          int index = 0;
//          // Цикл по глобальным особым точкам
//          for(unsigned long glob_feat_iter = 0; glob_feat_iter < GlobFeatures.size(); glob_feat_iter++){
//            // Среднее значение гистограммы текущей глобальной особой точки
//            float glob_hist_mean = std::accumulate(GlobFeatures.at(glob_feat_iter).hist.begin(), GlobFeatures.at(glob_feat_iter).hist.end(), 0) / GlobFeatures.at(glob_feat_iter).hist.size();

//            // Вычисляем корреляцию двух особых точек
//            float corr = 0.0;
//            float num  = 0.0;
//            float dev  = 0.0;

//            for(unsigned long bin_iterator = 0; bin_iterator < GlobFeatures.at(glob_feat_iter).nbins; bin_iterator++){
//              num +=          (GlobFeatures.at(glob_feat_iter).hist.at(bin_iterator) - glob_hist_mean) * (Features.at(feat_iter).hist.at(bin_iterator) - hist_mean);
//              dev += pow((GlobFeatures.at(glob_feat_iter).hist.at(bin_iterator) - glob_hist_mean) * (Features.at(feat_iter).hist.at(bin_iterator) - hist_mean), 2);
//            }
//            dev = sqrt(dev);
//            corr = num / dev;
//            // Фильтруем nan
//            if(!(corr != corr)){
//              if(corr >= coefcorr){
//                //                ROS_INFO("corr %3.3f", corr);

//                // Если такая точка уже была, то увеличить итератор встречаемости данной точки
//                index = glob_feat_iter;
//                isFeatureFinded = true;
//              }
//            }
//          }
//         ROS_INFO("glob.size = %d", GlobFeatures.size());
//         ROS_INFO("ne_glob.size = %d", Features.size());
//          if(isFeatureFinded) {
//            GlobFeatures.at(index).appearence++;
//           GlobFeatures.push_back(Features.at(feat_iter));
//          }
//          else {
//            GlobFeatures.push_back(Features.at(feat_iter));
//          }
//        }
//      }

//      ROS_INFO("mass_delta %d",mass_size_new);
//      mass_size_new = GlobFeatures.size();
//      mass_size_new = mass_size_new - mass_size_last;
//      mass_size_last = GlobFeatures.size();

//      visualization_msgs::MarkerArray markers;
//      for (int appear_points_itterator=0;appear_points_itterator<GlobFeatures.size();appear_points_itterator++) {
//        if (GlobFeatures.at(appear_points_itterator).appearence>=5) {
//          //Объявлениеобъекта
//          visualization_msgs::Marker marker;
//          //Название локальной системы координат
//          marker.header.frame_id = "marker";
//          //Временная отметка
//          marker.header.stamp = ros::Time();
//          //Название множества маркеров
//          marker.ns = "my_namespace";
//          //Числовой идентификатор (должен быть уникальным)
//          marker.id = appear_points_itterator;
//          //Тип фигуры (в данном случае сфера)
//          marker.type = visualization_msgs::Marker::SPHERE;
//          //Режим визуализации (в данном случае добавление к существующим
//          marker.action = visualization_msgs::Marker::ADD;
//          //Параметры положения объекта
//          marker.pose.position.x = GlobFeatures.at(appear_points_itterator).point.x;
//          marker.pose.position.y = GlobFeatures.at(appear_points_itterator).point.y;
//          marker.pose.position.z = GlobFeatures.at(appear_points_itterator).point.z;
//          //Параметры ориентации объекта
//          marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
//          //Размер объекта по каждому из измерений
//          marker.scale.x = 0.1;
//          marker.scale.y = 0.1;
//          marker.scale.z = 0.1;
//          //Цвет объекта -прозрачность
//          marker.color.a = 1.0;
//          //Цвет объекта по каждому из трёх каналов
//          marker.color.r = 1.0;
//          marker.color.g = 0.0;
//          marker.color.b = 0.0;
//          //ROS_INFO("x=%f.y=%f",GlobFeatures.at(appear_points_itterator).point.x,GlobFeatures.at(appear_points_itterator).point.y);
//          markers.markers.push_back(marker);
//        }
//      }
//      marker_pub.publish(markers);
      number++;

    }
    flag = false;
    r.sleep();
GlobFeatures.clear();
  }
}

