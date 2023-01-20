#line 1 "f:\\Desktop\\espa\\control.cpp"
#include <Arduino.h>
#include "control.h"

#define PWMA 25	//控制电机1 PWM控制引脚
#define PWMB 26	//控制电机1 PWM控制引脚
#define PWMC 33 //控制电机2 PWM控制引脚
#define PWMD 32 //控制电机2 PWM控制引脚

#define freq 50000 		 //PWM波形频率5KHZ
#define pwm_Channel_1  0 //使用PWM的通道0
#define pwm_Channel_2  1 //使用PWM的通道1
#define pwm_Channel_3  2 //使用PWM的通道2
#define pwm_Channel_4  3 //使用PWM的通道3

#define resolution  10		//使用PWM占空比的分辨率，占空比最大可写2^10-1=1023

/**************************************************************************
函数功能：赋值给PWM寄存器 
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
#line 22 "f:\\Desktop\\espa\\control.cpp"
void Set_Pwm(int motorr, int motorl);
#line 40 "f:\\Desktop\\espa\\control.cpp"
void contraol_setup();
#line 21 "f:\\Desktop\\espa\\esp32.ino"
void setup();
#line 47 "f:\\Desktop\\espa\\esp32.ino"
void loop();
#line 63 "f:\\Desktop\\espa\\esp32.ino"
void READ_ENCODER_L(void);
#line 78 "f:\\Desktop\\espa\\esp32.ino"
void READ_ENCODER_R(void);
#line 88 "f:\\Desktop\\espa\\esp32.ino"
void read(void);
#line 99 "f:\\Desktop\\espa\\esp32.ino"
void PID(void *ptParam);
#line 110 "f:\\Desktop\\espa\\esp32.ino"
int Incremental_PI_R(int Encoder,int Target);
#line 125 "f:\\Desktop\\espa\\esp32.ino"
int Incremental_PI_l(int Encoder,int Target);
#line 22 "f:\\Desktop\\espa\\control.cpp"
void Set_Pwm(int motorr, int motorl)
{
	int Amplitude = 950;  //===PWM满幅是1024 限制在950
    if (motorr < -Amplitude)  motorr = -Amplitude;
	if (motorr >  Amplitude)  motorr =  Amplitude;
	if (motorl < -Amplitude)  motorl = -Amplitude;
	if (motorl >  Amplitude)  motorl =  Amplitude;
	
	if (motorr > 0)   	ledcWrite(pwm_Channel_1,abs(motorr)),   	ledcWrite(pwm_Channel_2,0); 
	else             ledcWrite(pwm_Channel_1,0),   	ledcWrite(pwm_Channel_2,abs(motorr));
	
	if (motorl > 0)  	ledcWrite(pwm_Channel_3,0),   	ledcWrite(pwm_Channel_4,abs(motorl));
	else             ledcWrite(pwm_Channel_3,abs(motorl)),   	ledcWrite(pwm_Channel_4,0);

}


/*********************************setup**********************************/
void contraol_setup()
{
    pinMode(PWMA, OUTPUT);         
    pinMode(PWMB, OUTPUT);        
    pinMode(PWMC, OUTPUT);         
    pinMode(PWMD, OUTPUT);        
  
    ledcSetup(pwm_Channel_1, freq, resolution);	//PWM通道一开启设置
    ledcAttachPin(PWMA, pwm_Channel_1);			//PWM通道一和引脚PWMA关联
    ledcWrite(pwm_Channel_1, 0);				//PWM通道一占空比设置为零
    
    ledcSetup(pwm_Channel_2, freq, resolution); //PWM通道二开启设置
    ledcAttachPin(PWMB, pwm_Channel_2);			//PWM通道二和引脚PWMB关联
    ledcWrite(pwm_Channel_2, 0);				//PWM通道二占空比设置为零

   ledcSetup(pwm_Channel_3, freq, resolution);	//PWM通道一开启设置
    ledcAttachPin(PWMC, pwm_Channel_3);			//PWM通道一和引脚PWMA关联
    ledcWrite(pwm_Channel_3, 0);				//PWM通道一占空比设置为零
    
    ledcSetup(pwm_Channel_4, freq, resolution); //PWM通道二开启设置
    ledcAttachPin(PWMD, pwm_Channel_4);			//PWM通道二和引脚PWMB关联
    ledcWrite(pwm_Channel_4, 0);				//PWM通道二占空比设置为零
}

#line 1 "f:\\Desktop\\espa\\esp32.ino"
#include <Arduino.h>
#include "enc.h"
#include <Ticker.h>
#include "imu.h"
#include "micro_ros_arduino.h"
#include "micro_ros.h"
#include "odom.h"
#include "control.h"
#include  "PID_v2.h"
#include  "pid.h"
#define RADIUS 0.032
#define WIDTH 0.117
#define PI 3.14
int32_t Velocity_L, Velocity_R;   //左右轮编码器数据
float Velocity_Left, Velocity_Right;     //左右轮速度
Ticker timer_read_encoder;  // 中断函数定时器定义
float Velocity_KP =0.4, Velocity_KI =0.0001,Target=0;//Velocity_KP,Velocity_KI.PI参数  Target目标值
int startPWM=500;                 //初始PWM
int PWM_Restrict=900;            //startPW+PWM_Restric=255<256
int valuer,valuel;                       //用于存储通过PI控制器计算得到的用于调整电机转速的PWM值的整形变量 
void setup()
{




  imu_setup();
  ros_setup();
  contraol_setup();
    Serial.begin(115200);

	pinMode(ENCODER_L, INPUT);       //编码器引脚 输入模式
	pinMode(ENCODER_R, INPUT);       //编码器引脚 输入模式
	pinMode(DIRECTION_L, INPUT);     //编码器引脚 输入模式
	pinMode(DIRECTION_R, INPUT);     //编码器引脚 输入模式
	
	//编码器接口1 开启外部跳边沿中断 
	attachInterrupt(ENCODER_L, READ_ENCODER_L, CHANGE);  //中断函数READ_ENCODER_L
	//编码器接口2 开启外部跳边沿中断 
	attachInterrupt(ENCODER_R, READ_ENCODER_R, CHANGE);  //中断函数READ_ENCODER_R
	
	interrupts();                      //打开外部中断  
  timer_read_encoder.attach_ms(interrupt_time, read);  
  xTaskCreatePinnedToCore(PID, "PID", 1024 * 4, NULL, 1, NULL,0);
}

void loop()
{
 // Serial.println(" Velocity_L");
//Serial.println( Velocity_L);Serial.println(Velocity_Left,12);
  // Serial.println(" Velocity_R");
  //Serial.println(Velocity_Left,12);
  //Serial.println(Velocity_Right,12);
//Serial.println(Velocity_R);Serial.println(Velocity_Right,12);
 	ros_loop(Velocity_Left,Velocity_Right);

}
/**************************************************************************
函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
入口参数：无
返回  值：无
**************************************************************************/
void READ_ENCODER_L(void) {
  if (digitalRead(ENCODER_L) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L--;  //根据另外一相电平判定方向
    else      Velocity_L++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L++; //根据另外一相电平判定方向
    else     Velocity_L--; 
  }
}
/**************************************************************************
函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
入口参数：无
返回  值：无
**************************************************************************/
void READ_ENCODER_R(void) {
  if (digitalRead(ENCODER_R) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R--;//根据另外一相电平判定方向
    else      Velocity_R++;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R++; //根据另外一相电平判定方向
    else     Velocity_R--;
  }
}
void read(void)
{
/**************************************************************************
计算为转速n/s  500线光电编码器数据 定时时间为20MS
 转速 = num/500/20ms
 转速 = num*2/time
**************************************************************************/
    Velocity_Left = ((Velocity_L*50)/24)/90*RADIUS*PI*2;    Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
    Velocity_Right = ((Velocity_R*50)/24)/90*RADIUS*PI*2;    Velocity_R = 0; //读取右轮编码器数据，并清零
}

 void PID(void *ptParam) {
 
  while (1) {

     valuer=Incremental_PI_R(Velocity_Right*1000,200);
     valuel=Incremental_PI_l(Velocity_Left*1000,200);
    Set_Pwm(valuer+500,valuel+500);
    vTaskDelay(5);
  }
}

int Incremental_PI_R (int Encoder,int Target)
{  
   static float Bias,PWM,Last_bias;                      //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   Bias=Target-Encoder;                                  //计算偏差,目标值减去当前值
   PWM+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制计算
   
   if(PWM>PWM_Restrict)
   PWM=PWM_Restrict;                                     //限幅
   
   if(PWM<-PWM_Restrict)
   PWM=-PWM_Restrict;                                    //限幅  
   
   Last_bias=Bias;                                       //保存上一次偏差 
   return PWM;                                           //增量输出
}
int Incremental_PI_l (int Encoder,int Target)
{  
   static float Bias,PWM,Last_bias;                      //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   Bias=Target-Encoder;                                  //计算偏差,目标值减去当前值
   PWM+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制计算
   
   if(PWM>PWM_Restrict)
   PWM=PWM_Restrict;                                     //限幅
   
   if(PWM<-PWM_Restrict)
   PWM=-PWM_Restrict;                                    //限幅  
   
   Last_bias=Bias;                                       //保存上一次偏差 
   return PWM;                                           //增量输出
}
