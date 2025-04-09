#include "MSDriverSlave.h"
#include "Wire.h"
#include "common.h"
#include "config.h"

MSDriverSlave _MSDriverSlave; // 根据I2C指令进行相应的动作

uint8_t i2cAddr;
String inputString = "";     // 从串口接收到的字符串
bool stringComplete = false; // 判断有没有接收完毕

void printReadme();

void setup() {
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

    // 在串口输出使用说明
    printReadme();

    // 初始化I2C从机
    // 使用I2C2 (PB10,PB11引脚)
    Wire.setSCL(I2C2_SCL);
    Wire.setSDA(I2C2_SDA);
    // 打开I2C
    if (digitalRead(PIN_LED)) {
        // 地址跳线断开时
        i2cAddr = I2C_ADD;
    } else {
        // 地址跳线接通时
        i2cAddr = I2C_ADD1;
    }
    Wire.begin(i2cAddr);
    Wire.onReceive(MSDriverSlave::receiveEvent);
    // 这行一定要方在Wire.begin()后面
    Wire.onRequest(MSDriverSlave::requestEvent);
}

void loop() {
    static unsigned long t0 = micros();
    static bool blink = false;

    static int motorNum = 0; // 进行PID调参的电机（0~3）
    static float kp = 0;
    static float ki = 0;
    static float kd = 0;
    static float kr = 0;
    static bool tunePID = false;

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
        // 是否改变了参数
        bool isChanged = true;
        Serial.println(inputString);
        switch (inputString.charAt(0)) {
        case '?':
            printReadme();
            Serial.println();
            isChanged = false;
            break;
        case 'e':
            tunePID = !tunePID;
            _MSDriverSlave.motor[motorNum]._tunePID = tunePID;
            break;
        case 'm':
            motorNum = inputString.substring(1).toInt();
            if (motorNum < 0 || motorNum > 3) {
                motorNum = 0;
            }
            _MSDriverSlave.reg.mode.mMode[0] = 0b11001001; // PID控制(电机禁用)
            _MSDriverSlave.reg.mode.mMode[1] = 0b11001001; // PID控制(电机禁用)
            _MSDriverSlave.reg.mode.mMode[2] = 0b11001001; // PID控制(电机禁用)
            _MSDriverSlave.reg.mode.mMode[3] = 0b11001001; // PID控制(电机禁用)
            _MSDriverSlave.reg.cmd = APPLY;
            break;
        case 'r':
            kr = inputString.substring(1).toFloat();
            switch (motorNum) {
            case 0:
                _MSDriverSlave.reg.mode.m0KR = kr;
                break;
            case 1:
                _MSDriverSlave.reg.mode.m1KR = kr;
                break;
            case 2:
                _MSDriverSlave.reg.mode.m2KR = kr;
                break;
            case 3:
                _MSDriverSlave.reg.mode.m3KR = kr;
                break;
            }
            break;
        case 't':
            _MSDriverSlave.reg.ctrl.speedM[motorNum] = inputString.substring(1).toInt();
            break;
        case 'p':
            kp = inputString.substring(1).toFloat();
            switch (motorNum) {
            case 0:
                _MSDriverSlave.reg.mode.m0Kp = kp;
                break;
            case 1:
                _MSDriverSlave.reg.mode.m1Kp = kp;
                break;
            case 2:
                _MSDriverSlave.reg.mode.m2Kp = kp;
                break;
            case 3:
                _MSDriverSlave.reg.mode.m3Kp = kp;
                break;
            }
            break;
        case 'i':
            ki = inputString.substring(1).toFloat();
            switch (motorNum) {
            case 0:
                _MSDriverSlave.reg.mode.m0Ki = ki;
                break;
            case 1:
                _MSDriverSlave.reg.mode.m1Ki = ki;
                break;
            case 2:
                _MSDriverSlave.reg.mode.m2Ki = ki;
                break;
            case 3:
                _MSDriverSlave.reg.mode.m3Ki = ki;
                break;
            }
            break;
        case 'd':
            kd = inputString.substring(1).toFloat();
            switch (motorNum) {
            case 0:
                _MSDriverSlave.reg.mode.m0Kd = kd;
                break;
            case 1:
                _MSDriverSlave.reg.mode.m1Kd = kd;
                break;
            case 2:
                _MSDriverSlave.reg.mode.m2Kd = kd;
                break;
            case 3:
                _MSDriverSlave.reg.mode.m3Kd = kd;
                break;
            }
            break;
        default:
            isChanged = false;
        }

        inputString = "";
        stringComplete = false;

        Serial.printf("转速系数(r): %f 目标速度(t): %d \n", kr, _MSDriverSlave.reg.ctrl.speedM[motorNum]);
        Serial.printf("kp: %f ki: %f kd : %f\n", kp, ki, kd);

        if (isChanged) {
            if (tunePID) {
                Serial.println("PID调参：ON");
            } else {
                Serial.println("PID调参：OFF");
            }

            // 先停止电机
            _MSDriverSlave.motor[0].setMotorPWM(0);
            _MSDriverSlave.motor[1].setMotorPWM(0);
            _MSDriverSlave.motor[2].setMotorPWM(0);
            _MSDriverSlave.motor[3].setMotorPWM(0);

            // 等待电机速度归零
            delay(200);

            if (tunePID) {
                // 等待电机速度归零
                for (int i = 0; i < 10; i++) {
                    delay(180);
                    // 数据归零
                    Serial.printf("%f 0 0 255 \n", kr * _MSDriverSlave.reg.ctrl.speedM[motorNum]);
                }

                // 设置发生变更
                _MSDriverSlave.reg.mode.mMode[motorNum] = 0b11000001; // PID控制(电机使能)
                _MSDriverSlave.reg.cmd = APPLY;
            } else {
                // 设置发生变更
                _MSDriverSlave.reg.mode.mMode[motorNum] = 0b11001001; // PID控制(电机禁用)
                _MSDriverSlave.reg.cmd = APPLY;
            }
        }
    }
}

// 接收串口指令
void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        // 收到换行符则结束
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
}

void printReadme() {
    Serial.println("4路电机8路舵机控制模块 MSDriver (ver:2.2)");
    Serial.printf("I2C address: 0x%02X\n", i2cAddr);
    Serial.println("指令格式：REG地址，数据1，数据2，……");
    Serial.println("");
    Serial.println("你可以向此模块的寄存器发送指令以控制电机转速和舵机角度");
    Serial.println("寄存器主要分\"命令寄存器\"、\"模式寄存器\"、\"控制寄存器\""
                   "和\"反馈寄存器\"（具体可参考 \"MSDriverCommon.h\"）。");
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
    Serial.println("    接上编码电机后，使用Arduino IDE，打开串口监视器，波特率115200。");
    Serial.println("    向模块发送参数名（小写）和参数值，回车后，模块将以此参数对电机进行PID控制，");
    Serial.println("    同时在串口输出目标值、当前值和控制值，在Arduino IDE的\"串口绘图仪\"可观察控制曲线");
    Serial.println("    例：");
    Serial.println("       ?<Enter>     -- 帮助");
    Serial.println("       e<Enter>     -- 开始/停止调参");
    Serial.println("       m0<Enter>    -- 选择0号电机进行调参");
    Serial.println("       r2.5<Enter>  -- 设置转速系数");
    Serial.println("       t180<Enter>  -- 设置目标值（-255~255）");
    Serial.println("       p2<Enter>    -- 设置P参数");
    Serial.println("       i0.0002<Enter>  -- 设置I参数");
    Serial.println("       d10<Enter>  -- 设置D参数");
    Serial.println("开源地址：https://github.com/LeonO-OChen/MSDriver");
}
