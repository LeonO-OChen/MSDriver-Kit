#include "MSDriverSlave.h"
#include "Wire.h"
#include "common.h"
#include "config.h"

MSDriverSlave _MSDriverSlave; // 根据I2C指令进行相应的动作

uint8_t i2cAddr;
String inputString = "";     // 从串口接收到的字符串
bool stringComplete = false; // 判断有没有接收完毕

void initMSDriverSlave();
void printReadme();

void setup()
{
    Serial.begin(115200); // 初始化串口通信

    // ===========================================
    // 引脚输出模式
    // ===========================================
    // LED灯    (低电平点亮)
    // 同时作为I2C地址切换的输入口
    pinMode(PIN_LED, INPUT_PULLUP);
    digitalWrite(PIN_LED, HIGH); // 释放

    // 初始化模块
    _MSDriverSlave.init();
    // initMSDriverSlave();

    // 初始化I2C从机
    // 使用I2C2 (PB10,PB11引脚)
    Wire.setSCL(I2C2_SCL);
    Wire.setSDA(I2C2_SDA);
    Wire.onReceive(MSDriverSlave::receiveEvent);
    Wire.onRequest(MSDriverSlave::requestEvent);

    
    if (digitalRead(PIN_LED)) {
        // 地址跳线断开时
        i2cAddr = I2C_ADD;
    } else {
        // 地址跳线接通时
        i2cAddr = I2C_ADD1;
    }
    Wire.begin(i2cAddr);

    // 在串口输出使用说明
    printReadme();
}

void loop()
{
    static unsigned long t0 = micros();
    static bool blink = false;
    if (timePassed(t0, 200)) {
        // 每隔200ms闪灯
        // 注：当PC13接地时，不会闪烁
        if (blink) {
            digitalWrite(PIN_LED, HIGH);
        } else {
            digitalWrite(PIN_LED, LOW);
        }
        blink = !blink;
    }

    // 重复调用，根据寄存器指令控制电机和舵机
    _MSDriverSlave.execute();

    // 判断是否收到PID调参指令 ==> 对M0进行PID调参
    if (stringComplete) {
        Serial.println(inputString);
        switch (inputString.charAt(0)) {
        case 'p':
            _MSDriverSlave.reg.mode.m0Kp = inputString.substring(1).toFloat();
            break;
        case 'i':
            _MSDriverSlave.reg.mode.m0Ki = inputString.substring(1).toFloat();
            break;
        case 'd':
            _MSDriverSlave.reg.mode.m0Kd = inputString.substring(1).toFloat();
            break;
        }

        inputString = "";
        stringComplete = false;

        Serial.printf("kp: %f ki: %f kd : %f\n", _MSDriverSlave.reg.mode.m0Kp, _MSDriverSlave.reg.mode.m0Ki, _MSDriverSlave.reg.mode.m0Kd);

        // 先停止电机
        _MSDriverSlave.motor[0].setMotorPWM(0);
        delay(2000);

        // 设置发生变更
        _MSDriverSlave.reg.mode.m0Mode = 0b10001001; // 正向，PID控制
        _MSDriverSlave.receivedCmd = true;
    }
}

void initMSDriverSlave()
{
    MSDriverReg_MOD_t mode;
    memset((void *)&mode, 0, sizeof(mode));

    mode.m0Mode = 0b10000000;    // 测速，正向，计数不自动清零，无PID控制
    mode.m1Mode = 0b10000000;    // 测速，正向，计数不自动清零，无PID控制
    mode.m2Mode = 0b10000000;    // 测速，正向，计数不自动清零，无PID控制
    mode.m3Mode = 0b10000000;    // 测速，正向，计数不自动清零，无PID控制
    mode.smode0123 = 0b11111111; // 舵机模式
    mode.smode4567 = 0b11111111; // 舵机模式
    _MSDriverSlave.init(mode);
}

// 接收串口指令
void serialEvent()
{
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        // 收到换行符则结束
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
}

void printReadme(){
    Serial.println("4路电机8路舵机控制模块 MSDriver (ver:2.1)");
    Serial.printf ("I2C address: 0x%02X\n", i2cAddr);
    Serial.println("指令格式：REG地址，数据1，数据2，……");
    Serial.println("");
    Serial.println("你可以向此模块的寄存器发送指令以控制电机转速和舵机角度");
    Serial.println("寄存器主要分\"命令寄存器\"、\"模式寄存器\"、\"控制寄存器\"和\"反馈寄存器\"（具体可参考 \"MSDriverCommon.h\"）。");
    Serial.println("注意：在写入\"模式寄存器\"后，必须再向\"命令寄存器\"发送一个字节的数据，以通知模块改变其工作模式。");
    Serial.println("");
    Serial.println("控制电机转速:");
    Serial.println("    向电机相应的控制寄存器写入:2个字节的速度值（-255~255）。（特殊值0x0E0E表示刹车）");
    Serial.println("控制舵机角度:");
    Serial.println("    向舵机相应的控制寄存器写入:1个字节的角度值（0~180）。");
    Serial.println("引脚作为IO口输出:");
    Serial.println("    电机AB脚，或舵机的相应引脚作为输出时，写入相应的控制寄存器以输出");
    Serial.println("引脚作为IO口输入:");
    Serial.println("    电机AB脚，或舵机的相应引脚作为输入时，读取相应的反馈寄存器为输入");
    Serial.println("PID调参:");
    Serial.println("    M0接上编码电机后，使用Arduino IDE，打开串口监视器，波特率115200。");
    Serial.println("    向模块发送参数名（小写）和参数值，回车后，模块将以此参数对M0进行PID控制，");
    Serial.println("    同时在串口输出目标值、当前值和控制值，在Arduino IDE的\"串口绘图仪\"可观察控制曲线");
    Serial.println("    例：");
    Serial.println("       p2<Enter>");
    Serial.println("       i0.2<Enter>");
    Serial.println("       d0.4<Enter>");

}
