#include <ros/ros.h>
#include <math.h>

#include "test_service/test_service.h"

bool calc_triangle_square(test_service::test_service::Request  &req,
                          test_service::test_service::Response &res)
{
    double p = (req.a + req.b + req.c) / 2.0;
    double expr = p*(p-req.a)*(p-req.b)*(p-req.c);
    res.square = sqrt(expr);

    ROS_INFO("Request: a=%ld, b=%ld, c=%ld", req.a, req.b, req.c);
    ROS_INFO("Response: p=%f, expr=%f, square=%f", p, expr, res.square);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_service_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("geron_triangle", calc_triangle_square);
    ROS_INFO("Ready to calculate triangle square");

    ros::spin();

    return 0;
}
