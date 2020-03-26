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
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "omni_sub/OmniButtonEvent.h"
#include "omni_sub/TeleopControl.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
std_msgs::Float64MultiArray arr;
sensor_msgs::JointState joint_msg;
ros::Publisher chatter_pub ;
ros::Publisher chatter_pub1 ;
std::vector<float> vec;

bool button_grey;
bool button_white;

double omni_ang1 = 0;
double omni_ang2 = 0;
double omni_ang3 = 0;
double omni_ang4 = 0;

double pos1 = 0;
double pos2 = 0;
double pos3 = 0;
double pos4 = 0;

float motion_ratio;
double reset;
double set;
double effort_re;
double pos;

ros::Publisher forcefeed;
geometry_msgs::Vector3 force;

void bu_callback(const omni_sub::OmniButtonEvent::ConstPtr &msg1)
{

  button_grey = msg1->grey_button;
  button_white = msg1->white_button;

//  ROS_INFO_STREAM(button_grey);
//  ROS_INFO_STREAM(button_white);

if(button_grey == 0 && button_white ==0)
  {
     motion_ratio = 1;
     reset = 0;
  }
else if(button_grey == 1 && button_white ==0)
  {
     motion_ratio = 2;
     reset = 0;
  }
else if(button_white ==1 && button_grey ==0)
  {
     motion_ratio = 0.5;
     reset = 0;
  }
else if(button_white ==1 && button_grey ==1)
  {
     reset = 1;
  }
}


void po_callback(const sensor_msgs::JointState::ConstPtr& msg) 
{
 if(reset != 1 && set ==0)
 //  if(reset != 1 && pos ==1) //teleoperation test
{


  ROS_INFO_STREAM("motion_ratio: " << motion_ratio);

  omni_ang1 = msg->velocity[0]/motion_ratio;
  omni_ang2 = msg->velocity[1]/motion_ratio;
  omni_ang3 = msg->velocity[2]/motion_ratio;
  omni_ang4 = msg->velocity[4]/motion_ratio;

//  ROS_INFO_STREAM(omni_ang1);
//  ROS_INFO_STREAM(omni_ang2);
//  ROS_INFO_STREAM(omni_ang3);
//  ROS_INFO_STREAM(omni_ang4);

  vec.push_back(omni_ang1);
  vec.push_back(omni_ang2);
  vec.push_back(omni_ang3);
  vec.push_back(omni_ang4);

  arr.data.clear();
  arr.data.insert(arr.data.end(), vec.begin(), vec.end());

  vec.clear();
  chatter_pub.publish(arr);
 
// ROS_FATAL("sens 1");

}
 else if(reset ==1)
{
  omni_ang1 = 0;
  omni_ang2 = 0;
  omni_ang3 = 0;
  omni_ang4 = 0;

  vec.push_back(omni_ang1);
  vec.push_back(-omni_ang2);
  vec.push_back(-omni_ang3);
  vec.push_back(-omni_ang4);

  arr.data.clear();
  arr.data.insert(arr.data.end(), vec.begin(), vec.end());

  vec.clear();
  chatter_pub.publish(arr);
}

 



ROS_FATAL("sens 1");
}

int main(int argc, char ** argv) 
{
  ros::init(argc, argv, "pose_info");
  ros::NodeHandle nh;	
  ros::Subscriber endPose_sub = nh.subscribe("/omniEthernet/joint_states/", 100, po_callback);
  ros::Subscriber button_sub = nh.subscribe("/omniEthernet/button_state/", 100, bu_callback);

  chatter_pub = nh.advertise<std_msgs::Float64MultiArray>("/cmd_joint_vel", 100, po_callback);

  ros::Rate loop_rate(1000); //250
while (ros::ok())
{
  ros::spinOnce();
  loop_rate.sleep(); //loop_rate 만큼 쉬게 된다. loop_rate(10)인 경우에는 0.1초 쉬고 다시 while loop 돈다
}

return 0;
}


