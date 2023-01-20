#include "odom.h"
float last_time=0;
double   th=0;
double   x_=0;
double   y_=0;
const void euler_to_quat(float x, float y, float z, double* q) {
    float c1 = cos(y/2);
    float c2 = cos(z/2);
    float c3 = cos(x/2);

    float s1 = sin(y/2);
    float s2 = sin(z/2);
    float s3 = sin(x/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}
void odom(float Velocity_Ls,float Velocity_Rs){
   float curr_time =millis ()/1000;
   float dt= (curr_time - last_time); 
   double vx=(Velocity_Ls+Velocity_Rs)/2;
   double  vth=(Velocity_Ls-Velocity_Rs)/0.117;
   double  delta_x = (vx * cos(th)) * dt;         //th_弧度
   double  delta_y = (vx * sin(th)) * dt;
   double  delta_th = vth * dt;
      th += delta_th;
      x_ +=delta_x;
      y_ +=delta_y;
   odom_msg.pose.pose.position.x = x_ ;
   odom_msg.pose.pose.position.y = y_;
   odom_msg.pose.pose.position.z =0; 
   odom_msg.twist.twist.linear.x = vx;
   odom_msg.twist.twist.linear.y = 0;
   odom_msg.twist.twist.angular.z = vth;
   tf_msg.transforms.data[0].transform.translation.x = odom_msg.pose.pose.position.x;
   tf_msg.transforms.data[0].transform.translation.y=odom_msg.pose.pose.position.y;
  tf_msg.transforms.data[0].transform.translation.z=0;
   double q[4];
   euler_to_quat(0, 0, th, q);
      odom_msg.pose.pose.orientation.x = (double) q[1]; 
           odom_msg.pose.pose.orientation.y =(double) q[2]; 
                odom_msg.pose.pose.orientation.z = (double) q[3]; 
                     odom_msg.pose.pose.orientation.w =(double) q[0]; 
  tf_msg.transforms.data[0].transform.rotation.x = (double) q[1];
  tf_msg.transforms.data[0].transform.rotation.y = (double) q[2];
  tf_msg.transforms.data[0].transform.rotation.z = (double) q[3];
  tf_msg.transforms.data[0].transform.rotation.w = (double) q[0];
   last_time = curr_time;  
}