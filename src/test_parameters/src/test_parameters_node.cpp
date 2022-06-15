#include <ros/ros.h>
#include <math.h>

#define params_global

int main(int argc, char **argv)
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double p = 0.0;
    double expr = 0.0;
    double square = 0.0;

    ros::init(argc, argv, "hw2_glob_params_node");

#ifdef params_global
    ros::NodeHandle nh;

    if(nh.hasParam("/global_a"))
    {
        nh.getParam("/global_a", a);
        ROS_INFO("(Global) a = %f", a);
    }
    else {ROS_ERROR("There is no global parameter: global_a");}

    if(nh.hasParam("/global_b"))
    {
        nh.getParam("/global_b", b);
        ROS_INFO("(Global) b = %f", b);
    }
    else {ROS_ERROR("There is no global parameter: global_b");}

    if(nh.hasParam("/global_c"))
    {
        nh.getParam("/global_c", c);
        ROS_INFO("(Global) c = %f", c);
    }
    else {ROS_ERROR("There is no global parameter: global_c");}
    p = (a + b + c) / 2.0;
    expr = p*(p-a)*(p-b)*(p-c);
    square = sqrt(expr);
    nh.setParam("/global_square", square);
    ROS_INFO("(Global) square = %f", square);

#endif

#ifdef params_private_and_relative

    // ~ means private (inside package)
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_relative("triangle");

    if(nh_private.hasParam("private_a"))
    {
        nh_private.getParam("private_a", a);
        ROS_INFO("(Private) a = %f", a);
    }
    else {ROS_ERROR("There is no private parameter: private_a");}

    if(nh_private.hasParam("private_b"))
    {
        nh_private.getParam("private_b", b);
        ROS_INFO("(Private) b = %f", b);
    }
    else {ROS_ERROR("There is no private parameter: private_b");}

    if(nh_private.hasParam("private_c"))
    {
        nh_private.getParam("private_c", c);
        ROS_INFO("(Private) c = %f", c);
    }
    else {ROS_ERROR("There is no private parameter: private_c");}

    // calculate triangle square

    p = (a + b + c) / 2.0;
    expr = p*(p-a)*(p-b)*(p-c);
    square = sqrt(expr);

    // set new relative parameter

    nh_relative.setParam("relative_square", square);

    ROS_INFO("(Relative) square = %f", square);

#endif

    return 0;
}
