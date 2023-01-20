#ifndef imu_h
#define imu_h
#define OUTPUT_READABLE_QUATERNION  // 显示四元数值
#define OUTPUT_READABLE_EULER		// 以度为单位显示欧拉角度
#define OUTPUT_READABLE_YAWPITCHROLL// 以度为单位显示欧拉角度
#define OUTPUT_READABLE_REALACCEL	// 显示真实加速度，调整以消除重力 
#define OUTPUT_READABLE_WORLDACCEL	// 显示初始世界帧加速度，调整以移除重力，并基于四元数的已知方向旋转
#define OUTPUT_TEAPOT				// 显示四元数值
void imu_setup(void);
void imu_read(struct sensor_msgs__msg__Imu *imu,float *yaw_);
#endif