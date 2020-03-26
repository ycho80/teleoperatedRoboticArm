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

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
sensor_msgs::JointState joint_msg;
ros::Publisher chatter_pub ;
std::vector<float> vec;

double angle = 0;

void po_callback(const sensor_msgs::JointState::ConstPtr& msg) {

ROS_INFO_STREAM("Received pose: " << msg);

angle = msg->position[4];


ROS_INFO_STREAM(angle);


joint_msg.name.clear();
joint_msg.position.clear();
joint_msg.header.stamp = ros::Time::now();
joint_msg.name.push_back("id_4");
joint_msg.position.push_back(angle);

chatter_pub.publish(joint_msg);
ROS_FATAL("sens 1");
}

int main(int argc, char ** argv) {
ros::init(argc, argv, "pose_info");
ros::NodeHandle nh;	
ros::Subscriber bucket_joint = nh.subscribe("/omniEthernet/joint_states/", 100, po_callback);
chatter_pub = nh.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 100, po_callback);

ros::Rate loop_rate(250);
while (ros::ok())
{
ros::spinOnce();

loop_rate.sleep(); //loop_rate 만큼 쉬게 된다. loop_rate(10)인 경우에는 0.1초 쉬고 다시 while loop 돈다

}

return 0;
}


