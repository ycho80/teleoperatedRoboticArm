#include "ros/ros.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/JointState.h>
#include <vector>

std::vector<sensor_msgs::JointState::ConstPtr> pose;
sensor_msgs::JointState joint_msg_omni;
ros::Publisher chatter_pub ;
std::vector<float> vec;

double motor1 = 0;
double motor2 = 0;
double motor3 = 0;

double x = 0;
double x_val = 0;
double y = 0;
double y_val = 0;
double z = 0;
double z_val = 0;
double l1 = 0.13335;
double l2 = 0.13335;
double l3 = 0.13335;

void po_callback(const sensor_msgs::JointState::ConstPtr& msg) {

   ROS_INFO_STREAM("Received pose: " << msg);
    motor1 = msg->position[0];
    motor2 = msg->position[1];
    motor3 = msg->position[2];

    x = cos(motor1)*cos(motor2+motor3)*l3 +cos(motor1)*cos(motor2)*l2;
    y = sin(motor1)*cos(motor2+motor3)*l3 +sin(motor1)*cos(motor2)*l2;
    z = sin(motor2+motor3)*l3 +sin(motor2)*l2 + l1;

joint_msg_omni.name.clear();
joint_msg_omni.position.clear();
joint_msg_omni.header.stamp = ros::Time::now();
joint_msg_omni.position.push_back(x);
joint_msg_omni.position.push_back(y);
joint_msg_omni.position.push_back(z);    

ROS_INFO_STREAM("X: " << x);
ROS_INFO_STREAM("Y: " << y);
ROS_INFO_STREAM("Z: " << z);
ROS_INFO_STREAM("Motor1: " <<motor1);
ROS_INFO_STREAM("Motor2: " <<motor2);
ROS_INFO_STREAM("Motor3: " <<motor3);

   chatter_pub.publish(joint_msg_omni);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_angle");
    ros::NodeHandle nh;
    ros::Subscriber jointAngle_sub = nh.subscribe("/omniEthernet/joint_states", 100, po_callback);
    chatter_pub = nh.advertise<sensor_msgs::JointState>("/fk_omni", 100, po_callback);
    
    ros::Rate loop_rate(250);
while(ros::ok())
{
ros::spinOnce();
loop_rate.sleep();
}
    return(0);
}
