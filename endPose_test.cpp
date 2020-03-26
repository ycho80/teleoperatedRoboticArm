#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
#include <ros/xmlrpc_manager.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <sensor_msgs/JointState.h>

ros::Publisher chatter_pub ;
sensor_msgs::JointState joint_msg;
double theta1 = 0;
double theta2 = 0;
double theta3 = 0;

int main(int argc, char ** argv) {
	ros::init(argc, argv, "pose_publisher");
	ros::NodeHandle nh;	
	ros::Publisher chatter_pub = nh.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 100);

	ros::Rate loop_rate(100);
        ros::Duration(1).sleep();
int count = 0;
while (ros::ok())
{

joint_msg.name.clear();
joint_msg.position.clear();
theta1 = 0;
theta2 = 0;
joint_msg.header.stamp = ros::Time::now();
joint_msg.name.push_back("id_1");
joint_msg.position.push_back(theta1);
joint_msg.name.push_back("id_2");
joint_msg.position.push_back(theta2);
chatter_pub.publish(joint_msg);
ROS_FATAL("sens 1");
ros::Duration(2).sleep(); // sleep for 5 second

joint_msg.name.clear();
joint_msg.position.clear();
theta1 = 1;
theta2 = 1.5;
joint_msg.header.stamp = ros::Time::now();
joint_msg.name.push_back("id_1");
joint_msg.position.push_back(theta1);
joint_msg.name.push_back("id_2");
joint_msg.position.push_back(theta2);
chatter_pub.publish(joint_msg);
ROS_FATAL("sens 2");
ros::Duration(2).sleep(); // sleep for 5 second

joint_msg.name.clear();
joint_msg.position.clear();
theta1 = 0;
theta2 = 1;
joint_msg.header.stamp = ros::Time::now();
joint_msg.name.push_back("id_1");
joint_msg.position.push_back(theta1);
joint_msg.name.push_back("id_2");
joint_msg.position.push_back(theta2);
chatter_pub.publish(joint_msg);
ROS_FATAL("sens 3");
ros::Duration(2).sleep(); // sleep for 5 second

ros::spinOnce();

loop_rate.sleep();
++count;
}

return 0;
}


