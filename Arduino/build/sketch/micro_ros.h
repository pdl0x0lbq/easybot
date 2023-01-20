#line 1 "f:\\Desktop\\espa\\micro_ros.h"
#ifndef micro_ros_h
#define micro_ros_h
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <stdint.h>
extern struct nav_msgs__msg__Odometry odom_msg;
extern struct tf2_msgs__msg__TFMessage  tf_msg;
extern float vr,vl;
void  ros_setup(void);
void  ros_loop(float Velocity_Ls,float Velocity_Rs);
#endif