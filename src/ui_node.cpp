/**
 * \file ui_node.cpp
 * \brief User interface node for controlling turtlesim turtles.
 * \author Sarvenaz Ashoori
 * \version 0.1
 * \date 21/02/2025
 *
 * This node allows the user to manually control two turtles in the turtlesim environment.
 * It provides a menu-based interface for selecting a turtle and specifying movement commands.
 *
 * Services Used:
 * - /spawn (turtlesim::Spawn): Service to spawn a new turtle.
 *
 * Published Topics:
 * - /turtle1/cmd_vel (geometry_msgs::Twist): Velocity command for turtle1.
 * - /turtle2/cmd_vel (geometry_msgs::Twist): Velocity command for turtle2.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <string>

/**
 * \brief Main function to initialize the ROS UI node and provide user controls.
 * \param argc Argument count.
 * \param argv Argument vector.
 * \return int Execution status.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    ROS_INFO("Waiting for turtlesim spawn service...");
    ros::Duration(1.0).sleep();

    // Spawn the second turtle
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.0;
    spawn_srv.request.y = 2.0;
    spawn_srv.request.theta = 0;
    spawn_srv.request.name = "turtle2";

    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Successfully spawned turtle2 at position (5.0, 2.0)");
    } else {
        ROS_ERROR("Failed to spawn turtle2");
        return 1;
    }

    // Publishers for controlling the turtles
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    geometry_msgs::Twist cmd_msg;
    std::string selected_turtle;
    double linear_velocity, angular_velocity;
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        std::cout << "Select the turtle to control (turtle1 or turtle2): ";
        std::cin >> selected_turtle;

        if (selected_turtle == "turtle1" || selected_turtle == "turtle2") {
            std::cout << "Enter the linear velocity (0 to 2): ";
            std::cin >> linear_velocity;
            if (linear_velocity < 0 || linear_velocity > 2) {
                std::cout << "Invalid velocity, enter a value between 0 and 2." << std::endl;
                continue;
            }
            std::cout << "Enter the angular velocity (-2 to 2): ";
            std::cin >> angular_velocity;
            if (angular_velocity < -2 || angular_velocity > 2) {
                std::cout << "Invalid angular velocity, enter a value between -2 and 2." << std::endl;
                continue;
            }
            cmd_msg.linear.x = linear_velocity;
            cmd_msg.angular.z = angular_velocity;
            if (selected_turtle == "turtle1") {
                pub1.publish(cmd_msg);
            } else {
                pub2.publish(cmd_msg);
            }
            ros::Duration(1.0).sleep();
            cmd_msg.linear.x = 0;
            cmd_msg.angular.z = 0;
            if (selected_turtle == "turtle1") {
                pub1.publish(cmd_msg);
            } else {
                pub2.publish(cmd_msg);
            }
            std::cout << "Turtle " << selected_turtle << " has stopped." << std::endl;
        } else {
            std::cout << "Invalid turtle selection. Please choose turtle1 or turtle2." << std::endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

