/**
* \file exercise1.cpp
* \brief Controller for the turtlesim
* \author Sarvenaz Ashoori
* \version 0.1
* \date 25/02/2025
*
 * This node subscribes to the pose topics of two turtles and calculates the distance between them.
 * The calculated distance is published using a custom message type.
 *
 * Subscribed topics :
 * - /turtle1/pose (turtlesim::Pose): Position updates for turtle1.
 * - /turtle2/pose (turtlesim::Pose): Position updates for turtle2.
 *
 * Published Topics :
 * - /turtle_distance_topic (assignment1_rt::distance): Distance between turtles.
 */



#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <turtlesim/Pose.h>
#include <assignment1_rt/distance.h>  // Include the custom message for distance
#include <cmath>

/// Global publisher for turtle1 pose updates
ros::Publisher pub_turtle1;
/// Global publisher for turtle2 pose updates
ros::Publisher pub_turtle2;
/// Global publisher for the calculated distance
ros::Publisher pub_distance;

/// Stores the x-coordinate of turtle1
float turtle1_x;
/// Stores the y-coordinate of turtle1
float turtle1_y;

/// Distance threshold for stopping turtles
float threshold = 1.0;

/**
 * \brief Callback function for turtle1's pose updates.
 * \param msg Pose message from turtlesim.
 *
 * Updates the global variables storing turtle1's position.
 */
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_x = msg->x;
    turtle1_y = msg->y;
}

/**
 * \brief Callback function for turtle2's pose updates and distance calculation.
 * \param msg Pose message from turtlesim.
 *
 * Calculates the distance between turtle1 and turtle2 and publishes it.
 */
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    float x2 = msg->x;
    float y2 = msg->y;
    float distance = std::sqrt(std::pow(x2 - turtle1_x, 2) + std::pow(y2 - turtle1_y, 2));

    assignment1_rt::distance dist_msg;
    dist_msg.distance = distance;
    pub_distance.publish(dist_msg);
}

/**
 * \brief Main function to initialize the ROS node and set up subscribers/publishers.
 * \param argc Argument count.
 * \param argv Argument vector.
 * \return int Execution status.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    pub_turtle1 = nh.advertise<turtlesim::Pose>("/turtle1/pose", 10);
    pub_turtle2 = nh.advertise<turtlesim::Pose>("/turtle2/pose", 10);
    pub_distance = nh.advertise<assignment1_rt::distance>("/turtle_distance_topic", 10);

    ros::Subscriber sub_turtle1 = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber sub_turtle2 = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);

    ros::spin();
    return 0;
}
