#include "ros/ros.h"
#include "std_msgs/String.h"
#include "main.h"
#include "pid_controller.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "algorithm.hpp"

PID_Controller x_pos_controller     (0.1, 0.01, 0, -CONTROL_SPEED_MAX, CONTROL_SPEED_MAX);
PID_Controller y_pos_controller     (0.1, 0.01, 0, -CONTROL_SPEED_MAX, CONTROL_SPEED_MAX);
PID_Controller z_pos_controller     (0.1, 0.01, 0, -CONTROL_SPEED_MAX, CONTROL_SPEED_MAX);
PID_Controller yaw_pos_controller   (0.1, 0.01, 0, -CONTROL_SPEED_MAX, CONTROL_SPEED_MAX);

FirstOrderFilter x_feedback_filter(0.15);
FirstOrderFilter y_feedback_filter(0.15);
FirstOrderFilter z_feedback_filter(0.15);
FirstOrderFilter yaw_feedback_filter(0.15);

CartesianPosition cartesian_position_feedback;
CartesianPosition cartesian_position_feedback_filtered;
CartesianPosition cartesian_position_target;
ControlMode control_flag = ControlMode::CARTESIAN;

void cartesian_position_feedback_callback(const geometry_msgs::PoseStamped& msg)
{
    if (ControlMode::CARTESIAN == control_flag)
    {
        cartesian_position_feedback.real_mode = ControlMode::CARTESIAN;
        cartesian_position_feedback.x = msg.pose.position.x;
        cartesian_position_feedback.y = msg.pose.position.y;
    }
    cartesian_position_feedback.z = msg.pose.position.z;
    auto euler = Algorithm::quaternion_to_euler(msg.pose.orientation);
    cartesian_position_feedback.yaw = euler.z;
}

void pixel_position_feedback_callback(const geometry_msgs::Point& msg)
{
    if (ControlMode::PIXEL == control_flag)
    {
        cartesian_position_feedback.real_mode = ControlMode::PIXEL;
        cartesian_position_feedback.x = msg.x / PIXEL_FEEDBACK_REDUCE_SCALE;
        cartesian_position_feedback.y = msg.y / PIXEL_FEEDBACK_REDUCE_SCALE;
    }
}

void control_goal_callback(const geometry_msgs::PoseStamped& msg)
{
    if (static_cast<uint32_t>(ControlMode::CARTESIAN) == msg.header.seq)
        if (ControlMode::CARTESIAN == cartesian_position_feedback.real_mode)
        {
            control_flag = ControlMode::CARTESIAN;
            cartesian_position_target.x = msg.pose.position.x;
            cartesian_position_target.y = msg.pose.position.y;
            cartesian_position_target.z = msg.pose.position.z;
        }
    else if (static_cast<uint32_t>(ControlMode::PIXEL) == msg.header.seq)
        if (ControlMode::PIXEL == cartesian_position_feedback.real_mode)
        {
            control_flag = ControlMode::PIXEL;
            cartesian_position_target.x = 0;
            cartesian_position_target.y = 0;
        }
    cartesian_position_target.yaw = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle ros_node;

    ros::Subscriber cartesian_sub = ros_node.subscribe("/iris_0/mavros/local_position/pose", 1, cartesian_position_feedback_callback);
    ros::Subscriber pixei_sub = ros_node.subscribe("/circle/point", 1, pixel_position_feedback_callback);
    ros::Subscriber control_goal_sub = ros_node.subscribe("temp", 1, control_goal_callback);

    ros::Publisher control_output_pub = ros_node.advertise<geometry_msgs::Twist>("/xtdrone/iris_0/cmd_vel_flu", 1);

    geometry_msgs::Twist control_output;

    ros::Rate loop_rate(CONTROL_FREQUENCY);

    while (ros::ok())
    {
        cartesian_position_feedback_filtered.x = x_feedback_filter.step(cartesian_position_feedback.x);
        cartesian_position_feedback_filtered.y = y_feedback_filter.step(cartesian_position_feedback.y);
        cartesian_position_feedback_filtered.z = z_feedback_filter.step(cartesian_position_feedback.z);
        cartesian_position_feedback_filtered.yaw = yaw_feedback_filter.step(cartesian_position_feedback.yaw);

        control_output.linear.x = x_pos_controller.step(
            cartesian_position_target.x - cartesian_position_feedback_filtered.x);
        control_output.linear.y = y_pos_controller.step(
            cartesian_position_target.y - cartesian_position_feedback_filtered.y);
        control_output.linear.z = z_pos_controller.step(
            cartesian_position_target.z - cartesian_position_feedback_filtered.z);
        
        control_output.angular.x = 0;
        control_output.angular.y = 0;
        control_output.angular.z = yaw_pos_controller.step(
            cartesian_position_target.yaw - cartesian_position_feedback_filtered.yaw);

        control_output_pub.publish(control_output);

        ros::spinOnce();
        loop_rate.sleep();
    }
}


