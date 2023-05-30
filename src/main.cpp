#include "ros/ros.h"
#include "std_msgs/String.h"
#include "config.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle ros_node;

    ros::Rate loop_rate(CONTROL_FREQUENCY);

    while (ros::ok())
    {

    }
}


