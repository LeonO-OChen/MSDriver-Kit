/*
  CPU:  ESP32-S3
*/

#include "MSDriverMaster.h"
#include "i2cMaster.h" // I2C通信
#include <Adafruit_SSD1306.h>  // OLED显示屏

#define PIN_LED 2

// I2C
#define I2C_SCL 22
#define I2C_SDA 21
#define I2C_FREQ 200000

I2C_Master _i2c;

// 电机驱动模块
MSDriverMaster _MSDriverMaster;

// OLED显示屏
Adafruit_SSD1306 oled(128, 32, &Wire, -1);

void initMSDriver();

void setup()
{
    Serial.begin(115200); // 初始化串口，波特率设置为9600
    _i2c.init(I2C_SDA, I2C_SCL, I2C_FREQ);
    pinMode(PIN_LED, OUTPUT);

    initMSDriver();

    oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    oled.clearDisplay(); // 清屏
    oled.setTextSize(2);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(40, 16);
    oled.println("Hello!");
    oled.display(); // 显示
    oled.setTextSize(1);
}

// 主任务
void loop()
{
    // 每隔2S切换电机0的速度
    delay(2000);
    digitalWrite(PIN_LED, LOW);
    _MSDriverMaster.motor(0, 100);
    _MSDriverMaster.motor(1, 100);
    _MSDriverMaster.motor(2, 100);
    _MSDriverMaster.motor(3, 100);
    _MSDriverMaster.servo(0, 180);
    
    delay(2000);
    digitalWrite(PIN_LED, HIGH);
    _MSDriverMaster.motor(0, -100);
    _MSDriverMaster.motor(1, -100);
    _MSDriverMaster.motor(2, -100);
    _MSDriverMaster.motor(3, -100);
    _MSDriverMaster.servo(0, 0);
}

void initMSDriver()
{
    uint8_t motorMode1 = 0b10000000; // 测速，正向，计数不自动清零，无PID控制
    uint8_t motorMode2 = 0b10010000; // 测速，反向，计数不自动清零，无PID控制
    uint8_t smode = 0b11;  // 舵机模式
    _MSDriverMaster.init(0x32);
    _MSDriverMaster.setMotorMode(-1, motorMode1);            // 设置所有电机工作模式
    _MSDriverMaster.setMotorPID(-1, 0.6, 0.000001, 0, 7.02); // 设置所有电机PID参数
    _MSDriverMaster.setServoMode(-1, smode);             // 设置所有舵机工作模式
    _MSDriverMaster.sendCmd();
}