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
