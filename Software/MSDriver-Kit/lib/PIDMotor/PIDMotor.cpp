#include "PIDMotor.h"

void PIDMotor::setParam(float kp, float ki, float kd, float kStandradPoint) {
    // PID参数
    _pidCtrl.kp = kp;
    _pidCtrl.ki = _pidCtrl.ki * _pidCtrl.T;
    _pidCtrl.kd = _pidCtrl.kd / _pidCtrl.T;

    _kStandradPoint = kStandradPoint;
}

void PIDMotor::enablePID(bool enbable) { bEnabledPID = enbable; }

void PIDMotor::init(uint8_t pinPWM, uint8_t pinD1, uint8_t pinD2) {
    // 默认不进行PID控制
    bEnabledPID = false;

    // 引脚
    _pinPWM = pinPWM;
    _pinD1 = pinD1;
    _pinD2 = pinD2;

    _pidCtrl.T = 30000; // 1000=1ms
    _pidCtrl.max = 255;
    _pidCtrl.min = -255;

    // 默认PID
    setParam(0.6, 0.000001, 0, 2);
}

// 驱动电机 --mspeed:速度 -255~255
// 0x0E0E: 刹车
// 0x0E00: 滑行
void PIDMotor::setMotorTar(int16_t speed) {
    bBreak = (speed == 0x0E0E);   // 是否刹车
    bRelease = (speed == 0x0E00); // 是否滑行
    
    if (bRelease) {
        // 滑行
        digitalWrite(_pinD1, LOW);
        digitalWrite(_pinD2, LOW);
        analogWrite(_pinPWM, 0);
    } else if (bBreak) {
        // 刹车
        digitalWrite(_pinD1, HIGH);
        digitalWrite(_pinD2, HIGH);
        analogWrite(_pinPWM, 255);
    } else {
        speed = speed > 255 ? 255 : speed;
        speed = speed < -255 ? -255 : speed;

        if (bEnabledPID) {
            _count100msTar = _kStandradPoint * speed;
        } else {
            // 不进行PID控制的时候，直接设置PWM
            setMotorPWM(speed);
        }
    }
}

void PIDMotor::setMotorPWM(int16_t pwm) {
    // 输出方向
    if (pwm > 0) {
        // 前进
        digitalWrite(_pinD1, HIGH);
        digitalWrite(_pinD2, LOW);
    } else if (pwm < 0) {
        // 后退
        digitalWrite(_pinD1, LOW);
        digitalWrite(_pinD2, HIGH);
    } else {
        // 空档滑行
        digitalWrite(_pinD1, LOW);
        digitalWrite(_pinD2, LOW);
        return;
    }

    //  输出大小
    int val = abs(pwm);
    if (val < 40) {
        // _太小时带不动电机，长时间会烧坏
        analogWrite(_pinPWM, 0);
    } else {
        analogWrite(_pinPWM, val);
    }
}

void PIDMotor::execute(bool debug) {

    unsigned long t1 = micros();
    unsigned long dt = t1 - _sampleTime;

    // 测试计数转换成100ms的计数
    if (dt > 100000) {
        // 超过100ms，作废
        _count100ms = -1;
        _count = 0;
        _sampleTime = micros();
        return;
    } else if (dt < _pidCtrl.T) {
        // 小于采样周期，跳过PID处理
        return;
    } else {
        _count100ms = _count; // 速度大小（含方向）
        _count = 0;
        _sampleTime = micros();

        // 换算成100ms的计数
        _count100ms = _count100ms * 100000 / dt;
        _count100ms = KLM(KLM_data, _count100ms);
    }

    if (!bEnabledPID || bBreak || bRelease) {
        return;
    }

    _pidCtrl.target = _count100msTar;
    _pidCtrl.actual = _count100ms;
    // 位置式PID 计算
    _pidCtrl.positionPID();
    //_pidCtrl.incrementPID();

    // 输出
    setMotorPWM(_pidCtrl.control);

    if (debug) {
        Serial.printf("%f %f %f 255 0\n", _pidCtrl.target, _pidCtrl.actual,
                      _pidCtrl.control);
    }
}
