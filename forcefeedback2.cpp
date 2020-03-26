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

float force_x = 0;
float force_y = 0;
float force_z = 0;
float torque_x = 0;
float torque_y = 0;
float torque_z = 0;


ros::Publisher forcefeed;
geometry_msgs::Vector3 force;

void po_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {


ROS_INFO_STREAM("Received force/torque: " << msg);
//force.x = msg->wrench.force.y/-3;
//force.y = msg->wrench.force.x/-3;
force.z = msg->wrench.force.z/-10; //줄여야됨
//reading value가 실제값보다 30% 작음. reading value에 1.432 곱해줘야 됨

//ROS_INFO_STREAM(force.x);
//ROS_INFO_STREAM(force.y);
ROS_INFO_STREAM(force.z);


}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "forcefeedback");
	ros::NodeHandle nh;	
        ros::Subscriber ft_sub = nh.subscribe("/netft_data/", 10, po_callback);
	ros::Publisher forcefeed = nh.advertise<geometry_msgs::Vector3>("/omniEthernet/control", 10);

	ros::Rate loop_rate(1000);
      
while(ros::ok()){
    
    ros::spinOnce();
    forcefeed.publish(force);
    loop_rate.sleep();
    
}
return (0);
}






