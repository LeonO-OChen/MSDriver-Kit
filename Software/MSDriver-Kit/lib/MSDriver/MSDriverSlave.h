#include "../PIDMotor/PIDMotor.h"
#include "MSDriverCommon.h"
#include <Arduino.h>
#include <Servo.h>

#ifndef MSDriver_Slave_h
#define MSDriver_Slave_h

// 寄存器
typedef struct {
    uint8_t id;                /* ID*/
    uint8_t cmd;               /* 命令寄存器*/
    uint16_t NA;               /* 占位——由于编译器会结构对齐，使数据长度保持成4的整数倍（32位系统），这里被跳过了2个字节 */
    MSDriverReg_MOD_t mode;    /* 模式寄存器*/
    MSDriverReg_CTL_t ctrl;    /* 控制寄存器 */
    MSDriverReg_FB_t feedback; /* 反馈寄存器  */
} MSDriverReg_t;

class MSDriverSlave
{
    // private:
public:
    // 寄存器
    MSDriverReg_t reg;
    // 影子寄存器——实际使用的寄存器——防止使用过程中被覆盖
    MSDriverReg_MOD_t shadowRegMod;
    MSDriverReg_CTL_t shadowRegCtrl;
    // 当前寄存器地址
    uint8_t regAddr = 0;

    // 4路电机控制
    PIDMotor motor[4];
    Servo servo[8];

    // 根据变更工作模式
    void setModeByReg();
    // 变更电机的工作模式
    void motorSetup(int num, uint8_t mode, float kp, float ki, float kd, float kR, uint8_t pinA, uint8_t pinB, void (*interruptFunA)(void), void (*interruptFunB)(void));
    // 电机及相关引脚工作
    void motorAction(int num, uint8_t mode, int16_t &speed, int32_t &currspeed, uint8_t pinA, uint8_t pinB);
    // 变更舵机的工作模式
    void servoSetup(int num, uint8_t pin, uint8_t mode);
    // 舵机及相关引脚工作
    void servoAction(int num, uint8_t pin, uint8_t mode, uint8_t angle, uint16_t &currValue);

public:
    // 初始化
    void init();                              // 默认模式：电机不测速，AB引脚和舵机引脚都作为输入口
    void init(const MSDriverReg_MOD_t &mode); // 指定模式
    // 必须不断调用此函数对寄存器的设置做出相应
    void execute();

    void getfeedBack(MSDriverReg_FB_t *buf);
    // 收到主机发出的I2C指令（数据）
    static void receiveEvent(int howMany);
    // I2C主机要求读取当前寄存器的值
    static void requestEvent();

    // 电机转动触发外部中断，进行计数
    static void ReadM0ASpeed();
    static void ReadM0BSpeed();
    static void ReadM1ASpeed();
    static void ReadM1BSpeed();
    static void ReadM2ASpeed();
    static void ReadM2BSpeed();
    static void ReadM3ASpeed();
    static void ReadM3BSpeed();
};

extern MSDriverSlave _MSDriverSlave;

#endif