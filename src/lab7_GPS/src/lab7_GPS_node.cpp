#include <ros/ros.h>
#include <QXmlStreamReader>
#include <QFile>
#include <QString>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>

static double path_1 = -200;
static double path_2 = -200;
static double global_path = 0;
static double x_last = 0;
static double y_last = 0;

static bool flag = true;
void readFile(QString &pathToFile, double &x0, double &y0, sensor_msgs::NavSatFix &coordinates, geometry_msgs::PoseStamped &poseStamped, nav_msgs::Path &path, ros::Publisher &pub, ros::NodeHandle &nh)
{
  //Копируем расширение файла
  QString extension;
  extension = pathToFile.right(3);

  if(extension == "gpx"){
    QFile file(pathToFile);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
      ROS_INFO("File open error: %s", file.errorString().data());
    }

    QXmlStreamReader inputStream(&file);

    while (!inputStream.atEnd() && !inputStream.hasError()){
      inputStream.readNext();

      if (inputStream.isStartElement()){
        QString name = inputStream.name().toString();

        if (name == "trkpt"){
          coordinates.longitude = inputStream.attributes().value("lon").toFloat();
          coordinates.latitude  = inputStream.attributes().value("lat").toFloat();

          ROS_INFO("[lat, lon] = [%3.3f, %3.3f]", coordinates.latitude, coordinates.longitude);


          //трансформация из сферических координат в картографическую проекцию
          geographic_msgs::GeoPoint geo_pt;
          geo_pt.latitude  = coordinates.latitude;
          geo_pt.longitude = coordinates.longitude;
          geodesy::UTMPoint utm_pt(geo_pt);

          if(x0 == 0.0){
            x0 = utm_pt.easting;
            y0 = utm_pt.northing;
          }

          poseStamped.pose.position.x = utm_pt.easting  - x0;
          poseStamped.pose.position.y = utm_pt.northing - y0;

          path.poses.push_back(poseStamped);

          ROS_INFO("[x,y] = [%3.3f, %3.3f]", poseStamped.pose.position.x, poseStamped.pose.position.y);
          if(flag){
            x_last =  poseStamped.pose.position.x;
            y_last = poseStamped.pose.position.y;
          path_1 = sqrt ((poseStamped.pose.position.x - x_last)*(poseStamped.pose.position.x - x_last)+ (poseStamped.pose.position.y-y_last)*(poseStamped.pose.position.y-y_last));
          flag = false;
          }
          path_2 += sqrt ((poseStamped.pose.position.x - x_last)*(poseStamped.pose.position.x - x_last)+ (poseStamped.pose.position.y-y_last)*(poseStamped.pose.position.y-y_last));
          global_path = path_2 - path_1;
          ROS_INFO("PATH(metets) %f", global_path);

          x_last =  poseStamped.pose.position.x;
          y_last = poseStamped.pose.position.y;

        }
      }
    }

    ros::Rate r(10);

    while(nh.ok()){
      path.header.stamp = ros::Time::now();
      path.header.frame_id = "gps_map";
      pub.publish(path);
      r.sleep();
    }
  }
}

int main(int argc, char **argv){
  // init
  ros::init(argc, argv, "lab7_GPS_node");
  ros::NodeHandle nh;

  ros::Publisher path_pub = nh.advertise < nav_msgs::Path > ("/gps_path", 10);

  sensor_msgs::NavSatFix        coordinates;
  geometry_msgs::PoseStamped    poseStamped;
  nav_msgs::Path                trajectory;

  double x0 = 0.0, y0 = 0.0;

  // path to gpx file
  QString pathToFile = "/home/pinton/20210420-103111.gpx";

  // inf cycle
  readFile (pathToFile, x0, y0, coordinates, poseStamped, trajectory, path_pub, nh);
}
