#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
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
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;

float force_1 = 0;
float force_2 = 0;
float force_3 = 0;
int i,j,k;
float parms1 = 0.001;
float parms2 = 0.001;
float parms3 = 0.001;
float parms4 = 0.001;
float parms5 = 0.001;
float parms6 = 0.001;
float parms7 = 0.0475*0.1984;
float parms8 = 0.0475*0.1984;
float parms9 = 0.080815*0.1984;
float parms10 = 0.1984;
float parms11 = 0.01;
float parms12 = 0.01;
float parms13 = 0.001;
float parms14 = 0.001;
float parms15 = 0.001;
float parms16 = 0.001;
float parms17 = 0.001;
float parms18 = 0.001;
float parms19 = 0.06825*0.114;
float parms20 = 0.01425*0.114;
float parms21 = 0.017*0.114;
float parms22 = 0.114;
float parms23 = 0.01;
float parms24 = 0.01;
float parms25 = 0.001;
float parms26 = 0.001;
float parms27 = 0.001;
float parms28 = 0.001;
float parms29 = 0.001;
float parms30 = 0.001;
float parms31 = 0.06825*0.114;
float parms32 = 0.01425*0.114;
float parms33 = 0.017*0.114;
float parms34 = 0.114;
float parms35 = 0.01;
float parms36 = 0.01;
float parms37 = 0.001;
float parms38 = 0.001;
float parms39 = 0.001;
float parms40 = 0.001;
float parms41 = 0.001;
float parms42 = 0.001;
float parms43 = 0.04944*0.029;
float parms44 = 0.031415*0.029;
float parms45 = 0.017*0.029;
float parms46 = 0.029;
float parms47 = 0.01;
float parms48 = 0.01;

float q0 = 0.0;
float q1 = 0.0;
float q2 = 0.0;
float q3 = 0.0;

float dq0 = 0.0;
float dq1 = 0.0;
float dq2 = 0.0;
float dq3 = 0.0;

float ddq0 = 0.0;
float ddq1 = 0.0;
float ddq2 = 0.0;
float ddq3 = 0.0;

float dq0_old = 0.0;
float dq1_old = 0.0;
float dq2_old = 0.0;
float dq3_old = 0.0;

float effort0 = 0.0;
float effort1 = 0.0;
float effort2 = 0.0;
float effort3 = 0.0;

ros::Publisher forcefeed;
geometry_msgs::Vector3 force;

void po_callback(const sensor_msgs::JointState::ConstPtr& msg) {
effort0 = msg->effort[0];
effort1 = msg->effort[1];
effort2 = msg->effort[2];
effort3 = msg->effort[3];
q0 = msg->position[0]+0.16106799244880676;
q1 = msg->position[1]-1.9144080877304077;
q2 = msg->position[2]-0.6672816872596741;
q3 = msg->position[3]-1.8469128608703613;
MatrixXd q(4,1);
q(0,0) = q0; q(1,0) = q1; q(2,0) = q2; q(3,0) = q3; 
  ROS_INFO_STREAM("q:\n"<< q);

dq0 = msg->velocity[0];
dq1 = msg->velocity[1];
dq2 = msg->velocity[2];
dq3 = msg->velocity[3];
MatrixXd dq(4,1);
dq(0,0) = dq0; dq(1,0) = dq1; dq(2,0) = dq2; dq(3,0) = dq3;
  ROS_INFO_STREAM("dq:\n"<< dq);

ddq0 = (msg->velocity[0] - dq0_old) / 0.001 ;
ddq1 = (msg->velocity[1] - dq1_old) / 0.001 ;
ddq2 = (msg->velocity[2] - dq2_old) / 0.001 ;
ddq3 = (msg->velocity[3] - dq3_old) / 0.001 ;
MatrixXd ddq(4,1);
ddq(0,0) = ddq0; ddq(1,0) = ddq1; ddq(2,0) = ddq2; ddq(3,0) = ddq3;
  ROS_INFO_STREAM("ddq:\n"<< ddq);

dq0_old = dq0;
dq1_old = dq1;
dq2_old = dq2;
dq3_old = dq3;
MatrixXd dq_old(4,1);
dq_old(0,0) = dq0_old; dq_old(1,0) = dq1_old; dq_old(2,0) = dq2_old; dq_old(3,0) = dq3_old;
  ROS_INFO_STREAM("dq_old:\n"<< dq_old);


MatrixXd effort(4,1);
effort(0,0) = effort0; effort(1,0) = effort1; effort(2,0) = effort2; effort(3,0) = effort3;


MatrixXd jac(6,4);
jac(0,0) = 0.09888*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*sin(q2)*sin(q3) - 0.09888*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*cos(q2)*cos(q3) - 0.1365*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*cos(q2) - 0.1365*sin(q0)*cos(q1) - 0.1365*sin(q1)*cos(q0);
jac(0,1) = 0.09888*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*sin(q2)*sin(q3) - 0.09888*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*cos(q2)*cos(q3) - 0.1365*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*cos(q2) - 0.1365*sin(q0)*cos(q1) - 0.1365*sin(q1)*cos(q0);
jac(0,2) = (sin(q0)*sin(q1) - cos(q0)*cos(q1))*(0.09888*sin(q2)*cos(q3) + 0.1365*sin(q2) + 0.09888*sin(q3)*cos(q2));
jac(0,3) = (sin(q0)*sin(q1) - cos(q0)*cos(q1))*(0.09888*sin(q2)*cos(q3) + 0.09888*sin(q3)*cos(q2));
jac(1,0) = -0.09888*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*sin(q2)*sin(q3) + 0.09888*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*cos(q2)*cos(q3) + 0.1365*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*cos(q2) - 0.1365*sin(q0)*sin(q1) + 0.1365*cos(q0)*cos(q1);
jac(1,1) = -0.09888*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*sin(q2)*sin(q3) + 0.09888*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*cos(q2)*cos(q3) + 0.1365*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*cos(q2) - 0.1365*sin(q0)*sin(q1) + 0.1365*cos(q0)*cos(q1);
jac(1,2) = -(sin(q0)*cos(q1) + sin(q1)*cos(q0))*(0.09888*sin(q2)*cos(q3) + 0.1365*sin(q2) + 0.09888*sin(q3)*cos(q2));
jac(1,3) = -(sin(q0)*cos(q1) + sin(q1)*cos(q0))*(0.09888*sin(q2)*cos(q3) + 0.09888*sin(q3)*cos(q2));
jac(2,0) = 0;
jac(2,1) = 0;
jac(2,2) = -(sin(q0)*sin(q1) - cos(q0)*cos(q1))*(-0.09888*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*sin(q2)*sin(q3) + 0.09888*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*cos(q2)*cos(q3) + 0.1365*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*cos(q2)) + (sin(q0)*cos(q1) + sin(q1)*cos(q0))*(-0.09888*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*sin(q2)*sin(q3) + 0.09888*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*cos(q2)*cos(q3) + 0.1365*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*cos(q2));
jac(2,3) = -(sin(q0)*sin(q1) - cos(q0)*cos(q1))*(-0.09888*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*sin(q2)*sin(q3) + 0.09888*(-sin(q0)*sin(q1) + cos(q0)*cos(q1))*cos(q2)*cos(q3)) + (sin(q0)*cos(q1) + sin(q1)*cos(q0))*(-0.09888*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*sin(q2)*sin(q3) + 0.09888*(sin(q0)*cos(q1) + sin(q1)*cos(q0))*cos(q2)*cos(q3));
jac(3,0) = 0;
jac(3,1) = 0;
jac(3,2) = sin(q0)*cos(q1) + sin(q1)*cos(q0);
jac(3,3) = sin(q0)*cos(q1) + sin(q1)*cos(q0);
jac(4,0) = 0;
jac(4,1) = 0;
jac(4,2) = sin(q0)*sin(q1) - cos(q0)*cos(q1);
jac(4,3) = sin(q0)*sin(q1) - cos(q0)*cos(q1);
jac(5,0) = 1;
jac(5,1) = 1;
jac(5,2) = 0;
jac(5,3) = 0;

MatrixXd pinv_jac(4,6);
MatrixXd proc_jac(6,6);
MatrixXd pinv_pose_jac(6,6);
proc_jac = jac*jac.transpose();
pinv_jac = jac.transpose() * proc_jac.inverse();
pinv_pose_jac = pinv_jac.transpose();
  ROS_INFO_STREAM("jac:\n"<< jac);
  ROS_INFO_STREAM("pinv_jac:\n"<< pinv_jac);
  ROS_INFO_STREAM("pinv_pose_jac:\n"<< pinv_pose_jac);

  float Mx0 = sin(q2);
  float Mx1 = cos(q2);
  float Mx2 = parms24*Mx0 + parms25*Mx1;
  float Mx3 = Mx1*(-0.1365*pow(Mx0,2) - 0.1365*pow(Mx1,2));
  float Mx4 = -0.1365*pow(sin(q1),2) - 0.1365*pow(cos(q1),2);
  float Mx5 = Mx3 + Mx4;
  float Mx6 = cos(q3);
  float Mx7 = sin(q3);
  float Mx8 = Mx0*Mx6 + Mx1*Mx7;
  float Mx9 = Mx1*Mx6;
  float Mx10 = -Mx7;
  float Mx11 = Mx0*Mx10;
  float Mx12 = Mx11 + Mx9;
  float Mx13 = parms36*Mx8 + parms37*Mx12;
  float Mx14 = pow(Mx6,2);
  float Mx15 = pow(Mx7,2);
  float Mx16 = -0.09888*Mx14 - 0.09888*Mx15;
  float Mx17 = Mx11*Mx16 + Mx16*Mx9;
  float Mx18 = Mx17 + Mx5;
  float Mx19 = parms43*Mx18 + Mx13;
  float Mx20 = parms37*Mx8 + parms39*Mx12;
  float Mx21 = -parms42;
  float Mx22 = Mx18*Mx21 + Mx20;
  float Mx23 = parms43*Mx8 + Mx12*Mx21;
  float Mx24 = parms45*Mx18 + Mx23;
  float Mx25 = Mx16*Mx24;
  float Mx26 = parms25*Mx0 + parms27*Mx1;
  float Mx27 = -parms30;
  float Mx28 = -parms18;
  float Mx29 = parms31*Mx0 + Mx1*Mx27;
  float Mx30 = parms33*Mx5 + Mx24 + Mx29;
  float Mx31 = parms15 + Mx0*(parms31*Mx5 + Mx10*Mx22 + Mx10*Mx25 + Mx19*Mx6 + Mx2) + Mx1*(Mx19*Mx7 + Mx22*Mx6 + Mx25*Mx6 + Mx26 + Mx27*Mx5) + Mx28*Mx4 + Mx3*Mx30;
  float Mx32 = parms21*Mx4 + Mx28 + Mx30;
  float Mx33 = Mx31 - 0.1365*Mx32;
  float Mx34 = parms38*Mx8 + parms40*Mx12;
  float Mx35 = -parms44*Mx8;
  float Mx36 = 0.09888*Mx14 + 0.09888*Mx15;
  float Mx37 = parms26*Mx0 + parms28*Mx1 - 0.1365*parms32*Mx0 + 0.1365*parms44*Mx12*Mx7 + Mx34 + Mx35*Mx36 + 0.1365*Mx35*Mx6;
  float Mx38 = Mx34 + 0.09888*Mx35;
  float Mx39 = Mx3 - 0.1365;
  float Mx40 = Mx17 + Mx39;
  float Mx41 = parms43*Mx40 + Mx13;
  float Mx42 = Mx20 + Mx21*Mx40;
  float Mx43 = parms45*Mx40 + Mx23;
  float Mx44 = Mx16*Mx43;
  float Mx45 = parms33*Mx39 + Mx29 + Mx43;
  float Mx46 = Mx36 + 0.1365*Mx6;
  float Mx47 = 0.1365*Mx7;
  float Mx48 = -parms43;
  float Mx49 = parms41 + parms42*Mx46 + Mx47*Mx48;
  float Mx50 = parms42 + parms45*Mx46;
  float Mx51 = Mx49 + 0.09888*Mx50;

MatrixXd M(4,4);
M(0,0) = parms5 + Mx31 + Mx32*Mx4;
M(0,1) = Mx33;
M(0,2) = Mx37;
M(0,3) = Mx38;
M(1,0) = Mx33;
M(1,1) = parms15 + 0.1365*parms18 + 0.01863225*parms21 + Mx0*(parms31*Mx39 + Mx10*Mx42 + Mx10*Mx44 + Mx2 + Mx41*Mx6) + Mx1*(Mx26 + Mx27*Mx39 + Mx41*Mx7 + Mx42*Mx6 + Mx44*Mx6) - 0.1365*Mx28 + Mx3*Mx45 - 0.1365*Mx45;
M(1,2) = Mx37;
M(1,3) = Mx38;
M(2,0) = Mx37;
M(2,1) = Mx37;
M(2,2) = parms29 + 0.273*parms30 + 0.01863225*parms33 + Mx36*Mx50 + Mx49 + 0.1365*Mx50*Mx6 + 0.1365*Mx7*(parms45*Mx47 + Mx48);
M(2,3) = Mx51;
M(3,0) = Mx38;
M(3,1) = Mx38;
M(3,2) = Mx51;
M(3,3) = parms41 + 0.19776*parms42 + 0.0097772544*parms45;

  
//  ROS_INFO_STREAM("M:\n"<< M);

  float x0 = -0.1365*pow(sin(q1),2) - 0.1365*pow(cos(q1),2);
  float x1 = parms20*x0;
  float x2 = sin(q2);
  float x3 = cos(q2);
  float x4 = parms26*x2 + parms28*x3;
  float x5 = x3*x4;
  float x6 = sin(q3);
  float x7 = -x6;
  float x8 = cos(q3);
  float x9 = pow(x8,2);
  float x10 = pow(x6,2);
  float x11 = -0.09888*x10 - 0.09888*x9;
  float x12 = x3*x6;
  float x13 = x12 + x2*x8;
  float x14 = -parms44;
  float x15 = x3*x8;
  float x16 = x2*x7;
  float x17 = x15 + x16;
  float x18 = pow(x13,2)*x14 + x14*pow(x17,2);
  float x19 = x11*x18;
  float x20 = x19*x7 + x5;
  float x21 = parms38*x13 + parms40*x17;
  float x22 = x17*x21;
  float x23 = pow(x3,2);
  float x24 = pow(x2,2);
  float x25 = x3*(-0.1365*x23 - 0.1365*x24);
  float x26 = x0 + x25;
  float x27 = x11*x15 + x11*x16;
  float x28 = x26 + x27;
  float x29 = parms44*x28;
  float x30 = x13*x29 + x22;
  float x31 = -x13;
  float x32 = x21*x31;
  float x33 = x17*x29 + x32;
  float x34 = parms32*x26;
  float x35 = x2*x34;
  float x36 = -x2;
  float x37 = x36*x4;
  float x38 = x19*x8 + x37;
  float x39 = x3*x34;
  float x40 = -parms32;
  float x41 = x23*x40 + x24*x40;
  float x42 = x18 + x41;
  float x43 = x25*x42;
  float x44 = x1 + x2*(x20 + x30*x8 + x33*x7 + x35) + x3*(x30*x6 + x33*x8 + x38 + x39) + x43;
  float x45 = -parms20;
  float x46 = x42 + x45;
  float x47 = x0*x46;
  float x48 = x44 + x47;
  float x49 = -x0 - 0.1365;
  float x50 = 2*x3;
  float x51 = 2*x2;
  float x52 = parms26*x51 + parms28*x50;
  float x53 = x50*x8;
  float x54 = x51*x7;
  float x55 = x53 + x54;
  float x56 = x50*x6 + x51*x8;
  float x57 = parms38*x56 + parms40*x55;
  float x58 = x36*x49;
  float x59 = x3*x49;
  float x60 = x58*x8 + x59*x7;
  float x61 = x0 - 0.1365;
  float x62 = 2*x25 + x61;
  float x63 = x11*x53 + x11*x54 + x62;
  float x64 = parms44*x63;
  float x65 = x14*x60 + x55*x57 + x56*x64;
  float x66 = x58*x6 + x59*x8;
  float x67 = -x56;
  float x68 = parms44*x66 + x55*x64 + x57*x67;
  float x69 = parms32*x62;
  float x70 = x14*pow(x55,2) + x14*pow(x56,2);
  float x71 = x11*x70;
  float x72 = -x51;
  float x73 = x40*pow(x50,2) + x40*pow(x51,2) + x70;
  float x74 = parms20*x49 + 2*parms20*x61 + x2*(x40*x58 + x50*x52 + x51*x69 + x65*x8 + x68*x7 + x7*x71) + x25*x73 + x3*(parms32*x59 + x50*x69 + x52*x72 + x6*x65 + x68*x8 + x71*x8);
  float x75 = -4*parms20 + x73;
  float x76 = -x48;
  float x77 = -0.1365*parms20;
  float x78 = x25 - 0.1365;
  float x79 = x27 + x78;
  float x80 = parms44*x79;
  float x81 = x13*x80 + x22;
  float x82 = x17*x80 + x32;
  float x83 = parms32*x78;
  float x84 = x2*x83;
  float x85 = x3*x83;
  float x86 = x2*(x20 + x7*x82 + x8*x81 + x84) + x3*(x38 + x6*x81 + x8*x82 + x85) + x43 + x77;
  float x87 = x47 + x86;
  float x88 = -x87;
  float x89 = 0.09888*x10 + 0.09888*x9;
  float x90 = 0.1365*x8 + x89;
  float x91 = parms43*x13;
  float x92 = -parms42;
  float x93 = x17*x92 + x91;
  float x94 = parms45*x28;
  float x95 = x93 + x94;
  float x96 = parms45*x90;
  float x97 = parms42 + x96;
  float x98 = parms44*x31;
  float x99 = x97 + x98;
  float x100 = -x28;
  float x101 = parms37*x13;
  float x102 = parms39*x17;
  float x103 = -x101 - x102;
  float x104 = parms42*x28 + x103;
  float x105 = -parms40;
  float x106 = 0.1365*x6;
  float x107 = x105 + x106*x14;
  float x108 = x15 + x36*x6;
  float x109 = x36*x8;
  float x110 = -x12;
  float x111 = x109 + x110;
  float x112 = 0.1365*x2;
  float x113 = x109*x11 + x11*x110 + x112;
  float x114 = -parms43;
  float x115 = parms41 + x21;
  float x116 = parms42*x90 + x106*x114 + x115;
  float x117 = parms36*x108 + parms37*x111 + parms43*x113 + x107 + x116*x17;
  float x118 = x100*x99 + x104 + x117 + x90*x95;
  float x119 = parms30 + parms32*x36 + 0.1365*parms33;
  float x120 = -x119;
  float x121 = parms36*x13;
  float x122 = parms37*x17;
  float x123 = x121 + x122;
  float x124 = parms43*x28;
  float x125 = x123 + x124;
  float x126 = parms45*x106;
  float x127 = x114 + x126;
  float x128 = parms44*x17;
  float x129 = x127 + x128;
  float x130 = -x106;
  float x131 = parms38 + x14*x90;
  float x132 = parms37*x108 + parms39*x111 + x113*x92 + x116*x31 + x131;
  float x133 = x125 + x129*x28 + x130*x95 + x132;
  float x134 = parms33*x26;
  float x135 = -parms28;
  float x136 = parms29 + 0.1365*parms30 + x4;
  float x137 = parms25*x2;
  float x138 = parms27*x3;
  float x139 = parms31*x2;
  float x140 = -x17;
  float x141 = parms43*x108 + parms45*x113 + x111*x92 + x129*x140 + x13*x99;
  float x142 = x11*x141;
  float x143 = parms24*x3 + parms25*x36 - 0.1365*parms30*x3 + parms31*x112 + x135 + x136*x3 - x137 - x138 + 0.1365*x139 + x142*x7;
  float x144 = -parms31;
  float x145 = parms32*x3 + x144;
  float x146 = -parms30;
  float x147 = parms25*x3;
  float x148 = -0.1365*parms32;
  float x149 = parms26 + x148;
  float x150 = parms24*x2;
  float x151 = parms27*x36 + x112*x146 + x136*x36 + x142*x8 + 2*x147 + x149 + x150;
  float x152 = parms31*x26;
  float x153 = -x3;
  float x154 = parms31*x3 + parms33*x112 + x119*x2 + x141 + x145*x153 + x146*x36;
  float x155 = x154*x25;
  float x156 = x1 + x155 + x2*(parms30*x26 + x118*x8 + x120*x26 + x133*x7 + 0.1365*x134 + x143) + x3*(x118*x6 + x133*x8 + x145*x26 + x151 + x152);
  float x157 = x154 + x45;
  float x158 = x0*x157;
  float x159 = x2*(x107*x8 + x131*x7 + x135) + x3*(x107*x6 + x131*x8 + x149);
  float x160 = -x159;
  float x161 = parms45*x79;
  float x162 = x161 + x93;
  float x163 = -x79;
  float x164 = parms42*x79 + x103;
  float x165 = x117 + x162*x90 + x163*x99 + x164;
  float x166 = parms43*x79;
  float x167 = x123 + x166;
  float x168 = x129*x79 + x130*x162 + x132 + x167;
  float x169 = parms33*x78;
  float x170 = parms31*x78;
  float x171 = x155 + x2*(parms30*x78 + x120*x78 + x143 + x165*x8 + x168*x7 + 0.1365*x169) + x3*(x145*x78 + x151 + x165*x6 + x168*x8 + x170) + x77;
  float x172 = dq2*x159;
  float x173 = parms42 + 0.09888*parms45 + x98;
  float x174 = 0.09888*x13;
  float x175 = 0.09888*parms42 + x115;
  float x176 = parms36*x17 + parms37*x31 - 0.09888*parms42*x17 + parms43*x174 + x105 + x17*x175 + 0.09888*x91;
  float x177 = x100*x173 + x104 + x176 + 0.09888*x94;
  float x178 = x114 + x128;
  float x179 = -0.09888*parms44;
  float x180 = parms38 + x179;
  float x181 = parms39*x31 + x121 + 2*x122 + x174*x92 + x175*x31 + x180;
  float x182 = x124 + x178*x28 + x181;
  float x183 = parms43*x17 + parms45*x174 + x13*x173 + x140*x178 + x31*x92;
  float x184 = x11*x183;
  float x185 = x184*x7 + x5;
  float x186 = x184*x8 + x37;
  float x187 = x183 + x41;
  float x188 = x187*x25;
  float x189 = x1 + x188 + x2*(x177*x8 + x182*x7 + x185 + x35) + x3*(x177*x6 + x182*x8 + x186 + x39);
  float x190 = x187 + x45;
  float x191 = x0*x190;
  float x192 = x2*(x105*x8 + x180*x7) + x3*(x105*x6 + x180*x8);
  float x193 = -x192;
  float x194 = 0.09888*x161 + x163*x173 + x164 + x176;
  float x195 = x166 + x178*x79 + x181;
  float x196 = x188 + x2*(x185 + x194*x8 + x195*x7 + x84) + x3*(x186 + x194*x6 + x195*x8 + x85) + x77;
  float x197 = parms44*x106;
  float x198 = -4*parms40 + x130*x14 - 2*x197;
  float x199 = x90 - 0.09888;
  float x200 = x90 + 0.09888;
  float x201 = 4*parms38 + parms44*x199 - 2*parms44*x200;
  float x202 = dq2*(x160 + x193 + x2*(x135 + x198*x8 + x201*x7) + x3*(x149 + x198*x6 + x201*x8)) + dq3*x192;
  float x203 = -0.1365*x46;
  float x204 = x203 + x44;
  float x205 = -x204;
  float x206 = x203 + x86;
  float x207 = -x206;
  float x208 = -0.1365*x157;
  float x209 = -0.1365*x190;
  float x210 = x101 + x102;
  float x211 = x210 + x28*x92;
  float x212 = x125*x140 + x13*x211;
  float x213 = x147 + x150;
  float x214 = x152 + x213;
  float x215 = x137 + x138;
  float x216 = x146*x26 + x215;
  float x217 = x153*x214 + x2*x216;
  float x218 = x31*x95;
  float x219 = x17*x95;
  float x220 = x139 + x146*x3;
  float x221 = x36*(x134 + x220);
  float x222 = x212 + x217 + 0.1365*x218*x8 + x218*x89 + 0.1365*x219*x6 + 0.1365*x221;
  float x223 = parms42*x60 + x114*x66 - x55*(parms36*x56 + parms37*x55 + parms43*x63) + x56*(parms37*x56 + parms39*x55 + x63*x92);
  float x224 = parms43*x56 + parms45*x63 + x55*x92;
  float x225 = parms45*x60 + x224*x67;
  float x226 = -x222;
  float x227 = x210 + x79*x92;
  float x228 = x13*x227 + x140*x167;
  float x229 = x162*x31;
  float x230 = x146*x78 + x215;
  float x231 = x170 + x213;
  float x232 = x153*x231 + x2*x230;
  float x233 = x162*x17;
  float x234 = x36*(x169 + x220);
  float x235 = x228 + 0.1365*x229*x8 + x229*x89 + x232 + 0.1365*x233*x6 + 0.1365*x234;
  float x236 = -x235;
  float x237 = parms40 + x197;
  float x238 = -x90;
  float x239 = parms38*x108 + parms40*x111 + x106*x99 + x129*x238;
  float x240 = x13*(x211 + x237) + x140*(x125 + x131) + x239;
  float x241 = x108*x14 + x129;
  float x242 = x218 + x241;
  float x243 = 0.1365*parms31;
  float x244 = parms26*x3 + parms28*x36 + x148*x3 + x243;
  float x245 = parms44*x13;
  float x246 = x219 + x245;
  float x247 = x92 - x96;
  float x248 = parms44*x111 + x247;
  float x249 = x106*x97 + x127*x238;
  float x250 = 0.1365*x127*x8 + x127*x89 + 0.1365*x144 + x243 + 0.1365*x247*x6 + x249;
  float x251 = -x250;
  float x252 = x13*(x227 + x237) + x140*(x131 + x167) + x239;
  float x253 = x229 + x241;
  float x254 = x233 + x245;
  float x255 = 0.09888*parms43;
  float x256 = parms38*x17 + parms40*x31 + x17*x179 + x255;
  float x257 = x13*(parms40 + x211) + x140*(x125 + x180) + x256;
  float x258 = x114 + x218;
  float x259 = -0.09888*parms45 + x92;
  float x260 = x259 + x98;
  float x261 = 0.1365*x114*x8 + x114*x89 + x255 + 0.1365*x259*x6;
  float x262 = -x261;
  float x263 = x13*(parms40 + x227) + x140*(x167 + x180) + x256;
  float x264 = x114 + x229;
  float x265 = parms45*x200;
  float x266 = parms42*x130 + x106*(2*parms42 + x265) + x114*x199 - x200*(-2*parms43 + x126);
  float x267 = -4*parms43 + parms45*x130 + 2*x126;
  float x268 = x212 + 0.09888*x218;
  float x269 = x228 + 0.09888*x229;
  float x270 = -x269;
  float x271 = -x268;
  float x272 = 0.09888*x127 + x249;
  float x273 = -x272;
  float x274 = 0.09888*x114 + x255;
  float x275 = -x274;

MatrixXd C(4,4);
C(0,0) = dq0*x48;
C(0,1) = dq0*(x0*x75 + x74 + x76 + x88) + dq1*x87;
C(0,2) = dq0*(x156 + x158 + x160 + x76) + dq1*(x158 + x160 + x171 + x88) + x172;
C(0,3) = dq0*(x189 + x191 + x193 + x76) + dq1*(x191 + x193 + x196 + x88) + x202;
C(1,0) = dq0*x204;
C(1,1) = dq0*(x205 + x207 + x74 - 0.1365*x75) + dq1*x206;
C(1,2) = dq0*(x156 + x160 + x205 + x208) + dq1*(x160 + x171 + x207 + x208) + x172;
C(1,3) = dq0*(x189 + x193 + x205 + x209) + dq1*(x193 + x196 + x207 + x209) + x202;
C(2,0) = dq0*x222;
C(2,1) = dq0*(parms30*x58 + 0.1365*parms33*x58 + x144*x59 + x223 + 0.1365*x225*x8 + x225*x89 + x226 + x236 - x50*(parms24*x51 + parms25*x50 + parms31*x62) + x51*(parms25*x51 + parms27*x50 + x146*x62) + 0.1365*x6*(parms45*x66 + x224*x55) + 0.1365*x72*(parms31*x51 + parms33*x62 + x146*x50)) + dq1*x235;
C(2,2) = dq0*(0.1365*x144 + x153*(x149 + x214) + x2*(parms28 + x216) + 0.1365*x221 + x226 + x240 + 0.1365*x242*x8 + x242*x89 + x244 + x251 + 0.1365*x6*(x246 + x248)) + dq1*(0.1365*x144 + x153*(x149 + x231) + x2*(parms28 + x230) + 0.1365*x234 + x236 + x244 + x251 + x252 + 0.1365*x253*x8 + x253*x89 + 0.1365*x6*(x248 + x254)) + dq2*x250;
C(2,3) = dq0*(x217 + 0.1365*x221 + x226 + x257 + 0.1365*x258*x8 + x258*x89 + x262 + 0.1365*x6*(x246 + x260)) + dq1*(x232 + 0.1365*x234 + x236 + x262 + x263 + 0.1365*x264*x8 + x264*x89 + 0.1365*x6*(x254 + x260)) + dq2*(0.1365*x144 + x243 + x251 + x262 + x266 + 0.1365*x267*x8 + x267*x89 + 0.1365*x6*(-4*parms42 + parms45*x199 - 2*x265)) + dq3*x261;
C(3,0) = dq0*x268;
C(3,1) = dq0*(x223 + 0.09888*x225 + x270 + x271) + dq1*x269;
C(3,2) = dq0*(x240 + 0.09888*x242 + x271 + x273) + dq1*(x252 + 0.09888*x253 + x270 + x273) + dq2*x272;
C(3,3) = dq0*(x257 + 0.09888*x258 + x271 + x275) + dq1*(x263 + 0.09888*x264 + x270 + x275) + dq2*(x266 + 0.09888*x267 + x273 + x275) + dq3*x274;

//  ROS_INFO_STREAM("C:\n" << C);


  float Gx0 = sin(q2);
  float Gx1 = 9.81*Gx0;
  float Gx2 = sin(q3);
  float Gx3 = -Gx2;
  float Gx4 = cos(q2);
  float Gx5 = 9.81*Gx4;
  float Gx6 = cos(q3);
  float Gx7 = Gx1*Gx3 + Gx5*Gx6;
  float Gx8 = -parms44*Gx7;
  float Gx9 = Gx1*Gx6 + Gx2*Gx5;
  float Gx10 = parms44*Gx9;
  float Gx11 = Gx0*(-parms32*Gx5 + Gx10*Gx3 + Gx6*Gx8) + Gx4*(parms32*Gx1 + Gx10*Gx6 + Gx2*Gx8);
  float Gx12 = parms42*Gx7 - parms43*Gx9;
  float Gx13 = parms45*Gx7;

MatrixXd G(4,1);
G(0,0) = Gx11;
G(1,0) = Gx11;
G(2,0) = parms30*Gx5 - parms31*Gx1 + 0.1365*parms33*Gx5 + 0.1365*parms45*Gx2*Gx9 + Gx12 + 0.1365*Gx13*Gx6 + Gx13*(0.09888*pow(Gx2,2) + 0.09888*pow(Gx6,2));
G(3,0) = Gx12 + 0.09888*Gx13;

//  ROS_INFO_STREAM("G:\n" << G);

MatrixXd torque_ext(1,4);
torque_ext = M*ddq + C*dq + G - effort;

//  ROS_INFO_STREAM("torque_ext:\n" << torque_ext);

MatrixXd force_ext(6,1);
force_ext = pinv_pose_jac * torque_ext;
  
  ROS_INFO_STREAM("force_ext:\n" << force_ext);

force.x =force_ext(0,0);
force.y =force_ext(1,0);
force.z =force_ext(2,0); 
}


int main(int argc, char ** argv) {
	ros::init(argc, argv, "inverseDynamics");
	ros::NodeHandle nh;	
        ros::Subscriber ft_sub = nh.subscribe("/joint_states/", 10, po_callback);
	ros::Publisher forcefeed = nh.advertise<geometry_msgs::Vector3>("/omniEthernet/control", 10);

	ros::Rate loop_rate(1000);
      
while(ros::ok()){
    
    ros::spinOnce();
    forcefeed.publish(force);
    loop_rate.sleep();
    
}
return (0);
}





