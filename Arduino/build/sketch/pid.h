#line 1 "f:\\Desktop\\espa\\pid.h"
#define PID_H
 

 
/**
 *  PID controller class
 */
class PIDController
{
public:
    /**
     *  
     * @param P - Proportional gain 
     * @param I - Integral gain
     * @param D - Derivative gain 
     * @param ramp - Maximum speed of change of the output value
     * @param limit - Maximum output value
     */
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;
 
    float operator() (float error);
 
    float P;                //!< 比例增益
    float I;                //!< 积分增益
    float D;                //!< 微分增益
    float output_ramp;      //!< 输出值最大变化率
    float limit;            //!< 输出限制(最大输出绝对值)
 
protected:
    float error_prev;       //!< k-1时刻误差
    float error_prev1;      //!< k-2时刻误差
    unsigned long timestamp_prev; //!< 上次执行计算时的时间戳
    float output_prev;      //!< k-1时刻输出
    float output;           //
};