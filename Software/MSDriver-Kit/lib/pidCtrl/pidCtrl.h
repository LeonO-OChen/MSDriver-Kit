#include <Arduino.h>

#ifndef PID_CTRL_H
#define PID_CTRL_H

class PIDControl
{
public:
    // 控制参数
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float target = 0; // 目标值
    float max = 0;    // 最大控制值
    float min = 0;    // 最小控制值

    // 被控对象
    float actual = 0; // 实际值（当前值）

    // 输出值
    float control = 0; // 控制值

    void positionPID();
    void incrementPID();

private:
    float _error[3] = {0}; // 位置式PID：本次误差，上次误差，总误差
                           // 增量式PID：本次误差，本次误差-上次误差,上次误差-上上次误差
};

#endif