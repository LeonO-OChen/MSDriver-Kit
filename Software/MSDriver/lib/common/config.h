#include <Arduino.h>


#ifndef CONFIG_H
#define CONFIG_H

// 作为I2C从机
#define I2C2_SCL PB10   //I2C2
#define I2C2_SDA PB11
#define I2C_ADD     0x32
#define I2C_ADD1    0x30    

// LED灯
// 同时作为I2C地址切换的输入口，因此只能开漏输出 (OUTPUT_OPEN_DRAIN)
#define PIN_LED PC13    //  


// 编码电机0
#define PIN_M0_PWM PB9 // enA
#define PIN_M0_A PB12  // 接电机信号线1 必须接外部中断
#define PIN_M0_B PB13  // 接电机信号线2
#define PIN_M0_F1 PA13 // IN1
#define PIN_M0_F2 PA12 // IN2

// 编码电机1
#define PIN_M1_PWM PB8 // emB
#define PIN_M1_A PB15  // 接电机信号线1 必须接外部中断
#define PIN_M1_B PB14  // 接电机信号线2
#define PIN_M1_F1 PA11 // IN3
#define PIN_M1_F2 PB2  // IN4

// 编码电机2
#define PIN_M2_PWM PB7 // enC NG
#define PIN_M2_A PB4   // 接电机信号线1 必须接外部中断
#define PIN_M2_B PB3   // 接电机信号线2
#define PIN_M2_F1 PA4  // IN5
#define PIN_M2_F2 PA14 // IN6

// 编码电机3
#define PIN_M3_PWM PB6 // enD
#define PIN_M3_A PA8   // 接电机信号线1 必须接外部中断
#define PIN_M3_B PA5   // 接电机信号线2
#define PIN_M3_F1 PA15 // IN7
#define PIN_M3_F2 PB5  // IN8

// 舵机0~7
#define PIN_S0 PB1
#define PIN_S1 PB0
#define PIN_S2 PA7
#define PIN_S3 PA6
#define PIN_S4 PA3
#define PIN_S5 PA2
#define PIN_S6 PA1
#define PIN_S7 PA0

#endif