#line 1 "f:\\Desktop\\espa\\micro_ros.cpp"
#include <micro_ros_arduino.h>
#include "micro_ros.h"
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "imu.h"
#include "odom.h"
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h> 
#include <geometry_msgs/msg/twist.h>
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)\
//#include <nav_msgs/msg/odometry.h>

rcl_subscription_t vel_subscriber;
rcl_publisher_t imu_publisher;                    
rcl_publisher_t odom_publisher;
rcl_publisher_t tf_publisher;
struct nav_msgs__msg__Odometry odom_msg;
struct tf2_msgs__msg__TFMessage  tf_msg;
sensor_msgs__msg__Imu imu_msg;  
geometry_msgs__msg__Twist vel_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t timer2;
float vr,vl=0;
float oo;
#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
bool micro_ros_init_successful;
enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
  } state;
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  vr= (msg->linear.x+0.0585*msg->angular.z);
  vl= (msg->linear.x-0.0585*msg->angular.z);

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

   odom_msg.header.stamp.nanosec =rmw_uros_epoch_nanos();
   odom_msg.header.stamp.sec =  (rmw_uros_epoch_millis()/1000);
         tf_msg.transforms.data[0].header.stamp.nanosec = rmw_uros_epoch_nanos();
	  tf_msg.transforms.data[0].header.stamp.sec = (rmw_uros_epoch_millis()/1000);
        RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
  }
}
void timer2_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    imu_read(&imu_msg,&oo);

    imu_msg.orientation_covariance[0] = -1;
  imu_msg.header.stamp.sec = (rmw_uros_epoch_millis()/1000);
   imu_msg.header.stamp.nanosec= rmw_uros_epoch_nanos();
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}
  bool create_entities()
  {
   allocator = rcl_get_default_allocator();
   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
// create subscriber
  RCCHECK(rclc_subscription_init_default(
    &vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));
  RCCHECK(rclc_publisher_init_default(
            &imu_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "imu"));
   RCCHECK(rclc_publisher_init_default(
    &tf_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));

  // create timer,
  const unsigned int timer_timeout = 50;
    const unsigned int timer2_timeout = 50;

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  RCCHECK(rclc_timer_init_default(
    &timer2,
    &support,
    RCL_MS_TO_NS(timer2_timeout),
    timer2_callback));
  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &timer2));
  RCCHECK(rclc_executor_add_subscription(&executor, &vel_subscriber, &vel_msg, &subscription_callback, ON_NEW_DATA));

     delay(20);
    return true;
  }
  void destroy_entities()
  {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 200);
    rcl_subscription_fini(&vel_subscriber,&node);
    rcl_publisher_fini(&odom_publisher, &node);
        rcl_publisher_fini(&imu_publisher, &node);
            rcl_publisher_fini(&tf_publisher, &node);
    rcl_timer_fini(&timer);
    rcl_timer_fini(&timer2);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    delay(20);
  }


void ros_setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;
 micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    &odom_msg,
    (micro_ros_utilities_memory_conf_t) {});
    odom_msg.pose.pose.position.x =0;
    odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.position.z = 0; 
    odom_msg.twist.twist.linear.x = 0;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = 0;
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
    odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");
    imu_msg.header.frame_id.data = "IMUXX";
    imu_msg.header.frame_id.size = 5;

  if(!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
      &tf_msg,
      (micro_ros_utilities_memory_conf_t) {})
    )
  {
    error_loop();
  }
  tf_msg.transforms.size = 1;

 tf_msg.transforms.data[0].header.frame_id =
    micro_ros_string_utilities_set(tf_msg.transforms.data[0].header.frame_id, "/odom");
  tf_msg.transforms.data[0].child_frame_id =
    micro_ros_string_utilities_set(tf_msg.transforms.data[0].child_frame_id, "/base_link");
         tf_msg.transforms.data[0].transform.rotation.x = 1;
   tf_msg.transforms.data[0].transform.rotation.y = 0;
   tf_msg.transforms.data[0].transform.rotation.z = 0;
   tf_msg.transforms.data[0].transform.rotation.w = 0;


}

void ros_loop(float Velocity_Ls,float Velocity_Rs) {
  //delay(100);
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        odom(Velocity_Ls,Velocity_Rs);
        RCCHECK(rmw_uros_sync_session(1000));
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

}