/*
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
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>

float force_1 = 0;
float force_2 = 0;
float force_3 = 0;

double i=0;

ros::Publisher forcefeed;
geometry_msgs::Vector3 force;

void po_callback(const sensor_msgs::JointState::ConstPtr& msg) {


ROS_INFO_STREAM("Received force: " << msg);
force.z =msg->effort[3];
ROS_INFO_STREAM("Received force: " << force.z);
if (force.z > -2 && force.z < 0.05)
{
force.x =0;
force.y =0;
force.z =msg->effort[3]; 
i=0;
}

else if (force.z > 0.05 && force.z < 2)
{
force.x =0;
force.y =0;
force.z =msg->effort[3]; 
i=0;
}

else if (force.z < 0 && force.z >= -0.05)
{
force.x = 0;
force.y = 0;
force.z = 0;
i=1;
}
else if (force.z < 0.05 && force.z >= 0)
{
force.x = 0;
force.y = 0;
force.z = 0;
i=1;
} 
else if (force.z <=-2 || force.z >=2)
{
force.x = 0;
force.y = 0;
force.z = 0;
i=1;
}
ROS_INFO_STREAM("i: " << i);
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "forcefeedback");
	ros::NodeHandle nh;	
        ros::Subscriber ft_sub = nh.subscribe("/joint_states/", 10, po_callback);
	ros::Publisher forcefeed = nh.advertise<geometry_msgs::Vector3>("/omniEthernet/control", 10);

	ros::Rate loop_rate(250);
      
while(ros::ok()){
    
    ros::spinOnce();
    forcefeed.publish(force);
    loop_rate.sleep();
    
}
return (0);
}

*/




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
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>

float force_1 = 0;
float force_2 = 0;
float force_3 = 0;

double i=0;

ros::Publisher forcefeed;
geometry_msgs::Vector3 force;

void po_callback(const sensor_msgs::JointState::ConstPtr& msg) {


ROS_INFO_STREAM("Received force: " << msg);
force.z =msg->effort[3];
ROS_INFO_STREAM("Received force: " << force.z);
if (force.z >= -2.0 && force.z < 0)
{
force.x =0;
force.y =0;
force.z =msg->effort[3]; //범위를 더 쪼개야 된다. 아니면 범위 밖에 이따가 들어올 때 너무 세다. 1도 너무 세다//테스트 해보면서 조정이 필요
i=0;
}

else if (force.z <-2.0 || force.z >=0)
{
force.x = 0;
force.y = 0;
force.z = 0;
i=1;
}
ROS_INFO_STREAM("i: " << i);
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "forcefeedback");
	ros::NodeHandle nh;	
        ros::Subscriber ft_sub = nh.subscribe("/joint_states/", 10, po_callback);
	ros::Publisher forcefeed = nh.advertise<geometry_msgs::Vector3>("/omniEthernet/control", 10);

	ros::Rate loop_rate(250);
      
while(ros::ok()){
    
    ros::spinOnce();
    forcefeed.publish(force);
    loop_rate.sleep();
    
}
return (0);
}
