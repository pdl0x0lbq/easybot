#line 1 "f:\\Desktop\\espa\\enc.h"
#ifndef enc_h
#define enc_h
 #include <stdint.h>
#define ENCODER_R 	19  
#define DIRECTION_R 18
#define ENCODER_L   4
#define DIRECTION_L 5
#define interrupt_time 20 // 中断时间
extern int32_t Velocity_L, Velocity_R;   //左右轮编码器数据
extern float  Velocity_Left, Velocity_Right;     //左右轮速度
void READ_ENCODER_L(void);
void READ_ENCODER_R(void);
void read(void);
#endif