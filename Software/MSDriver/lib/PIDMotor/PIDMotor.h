#include "../klm/klm.h"
#include "pidCtrl.h"
#include <Arduino.h>

#ifndef PIDMoter_H
#define PIDMoter_H

class PIDMotor
{

public:
    KLM_t KLM_data;           // 卡尔曼滤波——使转速计数平缓
    bool bEnabledPID = false; // 默认不进行PID控制
    bool bBreak = false;      // 是否刹车
    PIDControl _pidCtrl;      // PID控制

    /*
    转速系数——PID控制时必须设置，用于设置目标转速(目标转速 = 转速系数 * 期望速度百分比)
    转速系数计算方法：
        假设编码电机的分辨率(电机转一圈计数)是13，减速比是90(电机转90圈，输出轴转1圈)，额定转速是180rpm，
        采用4倍频(AB引脚的上升下降沿都参与)计数，
        则每转一圈计数13*90*4=4680次，
        每分钟计数为4680*180=842400次，
        即每100ms为842400/60/10=1404次。
        转速系数=1404/255=5.5

        因为 目标转速计数 = 转速系数 * 目标速度(-255 ~ 255)
        所以转速系数越小，目标转速越慢
    */
    float _kStandradPoint;
    float _count100msTar = 0;      // 电机的目标转速——换算成每100ms计数
    float _count100ms = 0;         // 电机的实际转速——换算成每100ms的计数
    unsigned long _sampleTime = 0; // 距离上次采样的微秒数
    int32_t _count = 0;            // 电机的转速计数

    /** 电机是否反向 1：正向 -1：反向**/
    int8_t _offset = 1;
    uint8_t _pinD1;  // 电机的方向引脚1
    uint8_t _pinD2;  // 电机的方向引脚2
    uint8_t _pinPWM; // 电机的PWM引脚

    void enablePID(bool enbable);
    void setParam(int8_t offset, float kp, float ki, float kd, float kStandradPoint);
    void init(uint8_t pinPWM, uint8_t pinD1, uint8_t pinD2);
    void setMotor(int mspeed); // 驱动单个电机 -- mspeed:速度 -255~255
    void setMotorPWM(int pwm); // 驱动单个电机 -- pwm:速度 -255~255
    void execute(bool debug);  // 如需PID控制，每30ms调用此函数
};

#endif