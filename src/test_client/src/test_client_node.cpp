#include <ros/ros.h>
#include "test_service/test_service.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_client_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<test_service::test_service>("geron_triangle");

    // create service object
    test_service::test_service srv;

    // init object's data fields
    srv.request.a = 8;
    srv.request.b = 7;
    srv.request.c = 6;

    if(client.call(srv)) // if true
    {
        ROS_INFO("Triangle square: %f", srv.response.square);
    }
    else
    {
        ROS_ERROR("Failed to call service geron_triangle");
        return 1;
    }

    return 0;
}
