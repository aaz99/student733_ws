#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

static sensor_msgs::Image depth_map;
static sensor_msgs::CameraInfo cam_info;
static bool cam_info_received = false;
static bool depth_map_received = false;
//sensor_msgs::Image new_map;


static sensor_msgs::PointCloud  pc_msg;
static sensor_msgs::PointCloud2 pc2_msg;

//sensor_msgs::Image convetation(){
//  for (int h = 0; h <= depth_map.height; h++){
//    for (int w = 0;w <= depth_map.width;w+=2) {
//     int z = w/2;
//      new_map.data[z] = depth_map.data[w]+256*depth_map.data[w];
//    }
//  }
//}

void depth_map_callback(const sensor_msgs::Image &map)
{
    depth_map_received = true;
    depth_map = map;
    ROS_INFO("Depth map received");
}
void cam_info_callback(const sensor_msgs::CameraInfo &info)
{
    if(cam_info_received == false)
    {
        cam_info_received = true;
        cam_info = info;

        ROS_INFO("Info about camera accepted");
    }
}


void point_cloud();
void point_cloud2();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_point_cloud_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_depth_map = nh.subscribe("/camera/depth/image_raw", 10, depth_map_callback);
    ros::Subscriber sub_cam_info = nh.subscribe("/camera/depth/camera_info", 10, cam_info_callback);

    ros::Publisher pub_point_cloud = nh.advertise<sensor_msgs::PointCloud>("point_cloud", 10);
    ros::Publisher pub_point_cloud2 = nh.advertise<sensor_msgs::PointCloud2>("point_cloud2", 10);

    ros::Rate r(1);
    while(ros::ok())
    {
        if(depth_map_received == true)
        {

            point_cloud();                      //создание облака точек
            point_cloud2();

            pub_point_cloud.publish(pc_msg);    //вывод облака точек
            pub_point_cloud2.publish(pc2_msg);

            pc_msg.points.clear();              //очистка
            pc2_msg.data.clear();
            depth_map_received = false;
        }

        ros::spinOnce();
        r.sleep();
    }
}

void point_cloud()
{
    float s = 1000.0; // scaling

    for(unsigned int i = 0; i < depth_map.width; i++) {
        for(unsigned int j = 0; j < depth_map.height; j++) {

            geometry_msgs::Point32 point; // float32

            // get value by index
            unsigned short px_value = depth_map.data.at(j * depth_map.width * 2 + i * 2) + 255 * depth_map.data.at(j * depth_map.width * 2 + i*2+1);

            // calculate point parameters
            point.z = (float)px_value / s;
            point.x = ((float(i) - float(cam_info.K[2])) * point.z) / float(cam_info.K[0]);
            point.y = ((float(j) - float(cam_info.K[5])) * point.z) / float(cam_info.K[4]);

            // fill point cloud
            pc_msg.header.frame_id = "cloud_link";
            pc_msg.header.stamp = ros::Time::now();
            pc_msg.points.push_back(point);
        }
    }
}

void point_cloud2()
{
    // init point cloud 2
    pc2_msg.header.frame_id = "cloud_link2";
    pc2_msg.header.stamp = ros::Time::now();
    pc2_msg.width  = depth_map.width;
    pc2_msg.height = depth_map.height;
    pc2_msg.is_bigendian = false;
    pc2_msg.is_dense = false;

    // create pointCloud2 structure
    sensor_msgs::PointCloud2Modifier modifier(pc2_msg);

    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(depth_map.width * depth_map.height);

    // create iterators (ptrs)
    sensor_msgs::PointCloud2Iterator<float> out_x(pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(pc2_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> out_r(pc2_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> out_g(pc2_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> out_b(pc2_msg, "b");

    float s = 1000; // scaling

    for(unsigned int i = 0; i < depth_map.width; i++){
        for(unsigned int j = 0; j < depth_map.height; j++){

            float px_value = depth_map.data.at(j * depth_map.width * 2 + i * 2) + 255 * depth_map.data.at(j * depth_map.width * 2 + i*2+1);
            *out_z = px_value / s;                                                            //координаты точек
            *out_x = ((float(i) - float(cam_info.K[2])) * (*out_z)) / float(cam_info.K[0]);
            *out_y = ((float(j) - float(cam_info.K[5])) * (*out_z)) / float(cam_info.K[4]);

            // point color
            *out_r = 255;
            *out_g = 255;
            *out_b = 255;

            // increment point
            ++out_x; ++out_y; ++out_z;
            ++out_r; ++out_g; ++out_b;
        }
    }
}
