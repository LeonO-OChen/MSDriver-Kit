#include "pidCtrl.h"

void PIDControl::positionPID()
{
    // _error[0~2]: 本次误差，上次误差，总误差

    // 计算误差
    _error[0] = target - actual;

    // 积分分离——控制值已经超出最大/最小值时，不累加误差积分项
    if (control >= max)
    {
        if (_error[0] < 0)
        {
            _error[2] += _error[0];
        }
    }
    else if (control <= min)
    {
        if (_error[0] > 0)
        {
            _error[2] += _error[0];
        }
    }
    else
    {
        _error[2] += _error[0];
    }

    // 位置式PID计算
    float pid = kp * _error[0] + ki * _error[2] + kd * (_error[0] - _error[1]);
    _error[1] = _error[0];

    // 输出范围限制
    pid = pid > max ? max : pid;
    pid = pid < min ? min : pid;

    // 输出
    control = pid;
}

void PIDControl::incrementPID()
{
    // _error[0~2]: 本次误差，本次误差-上次误差,上次误差-上上次误差

    // 增量式PID 计算
    float error = target - actual;
    _error[2] = _error[1];         // 上次误差-上上次误差
    _error[1] = error - _error[0]; // 本次误差-上次误差
    _error[0] = error;                   // 本次误差

    // 增量式PID
    float pid = kp * _error[1] + ki * _error[0] + kd * (_error[1] - _error[2]);
    pid += control;

    // 输出范围限制
    pid = pid > max ? max : pid;
    pid = pid < min ? min : pid;

    // 输出
    control = pid;
}