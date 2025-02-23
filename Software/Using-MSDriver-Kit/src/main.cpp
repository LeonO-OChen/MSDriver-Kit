/*
  CPU:  ESP32-S3
*/

#include "MSDriverMaster.h"
#include "i2cMaster.h"        // I2C通信
#include <Adafruit_SSD1306.h> // OLED显示屏

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
void initMSDriver2();

void setup()
{
    Serial.begin(115200); // 初始化串口，波特率设置为9600
    _i2c.init(I2C_SDA, I2C_SCL, I2C_FREQ);
    pinMode(PIN_LED, OUTPUT);

    // 等待其它设备上电完毕
    delay(500);

    initMSDriver();

    oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    oled.clearDisplay(); // 清屏
    oled.setTextSize(2);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(40, 16);
    oled.println("init...");
    oled.display(); // 显示

    delay(1000);
    oled.clearDisplay(); // 清屏
    // oled.setCursor(40, 16);
    // oled.println("Hello!");
    oled.display(); // 显示
    oled.setTextSize(1);
    oled.setCursor(10, 20);
}

// 主任务
void loop()
{
    static char str[20];
    int32_t speed0 = 0;
    int32_t speed2 = 0;
    int count = 0;
    for (int i = 0; i < 20; i++) {
        // 每隔200ms切换电机0的速度
        delay(200);

        //_MSDriverMaster.getValueM(0, &speed0);
        //_MSDriverMaster.getValueM(2, &speed2);
        //_i2c.ReadDataArray(0x28, M0_SPEED_RA, (uint8_t*)&speed0, 4);
        _i2c.ReadDataArray(0x32, M0_SPEED_RA, (uint8_t *)&speed0, 4);

        sprintf(str, "%7d %7d\n", speed0, speed2);
        oled.println(str);
        oled.display(); // 显示

        if (++count == 10) {
            // 每隔2s切换电机的速度
            digitalWrite(PIN_LED, LOW);
            _MSDriverMaster.motor(0, 100);
            _MSDriverMaster.motor(1, 100);
            _MSDriverMaster.motor(2, 100);
            _MSDriverMaster.motor(3, 100);
            _MSDriverMaster.servo(0, 180);
        }
    }
    // 每隔2s切换电机的速度
    digitalWrite(PIN_LED, HIGH);
    _MSDriverMaster.motor(0, -100);
    _MSDriverMaster.motor(1, -100);
    _MSDriverMaster.motor(2, -100);
    _MSDriverMaster.motor(3, -100);
    _MSDriverMaster.servo(0, 0);
}

// 4电机开环控制，舵机全开
void initMSDriver()
{
    uint8_t motorMode01 = 0b10000000; // 测速，正向，计数不自动清零，无PID控制
    uint8_t motorMode23 = 0b10010000; // 测速，反向，计数不自动清零，无PID控制
    uint8_t smode = 0b11;             // 舵机模式
    _MSDriverMaster.init(0x32);
    _MSDriverMaster.setMotorMode(0, motorMode01); // 设置M0工作模式
    _MSDriverMaster.setMotorMode(1, motorMode01); // 设置M1工作模式
    _MSDriverMaster.setMotorMode(2, motorMode23); // 设置M2工作模式
    _MSDriverMaster.setMotorMode(3, motorMode23); // 设置M3工作模式
    _MSDriverMaster.setServoMode(-1, smode);      // 设置所有舵机工作模式
    _MSDriverMaster.sendCmd();
}

// M0,M2 闭环控制
void initMSDriver2()
{
    uint8_t motorMode0 = 0b10000000; // 测速，正向，计数不自动清零，无PID控制
    uint8_t motorMode2 = 0b10010000; // 测速，反向，计数不自动清零，无PID控制
    uint8_t smode = 0b11;            // 舵机模式
    _MSDriverMaster.init(0x32);
    _MSDriverMaster.setMotorMode(0, motorMode0);            // 设置M0工作模式
    _MSDriverMaster.setMotorMode(2, motorMode2);            // 设置M2工作模式
    _MSDriverMaster.setMotorPID(-1, 0.6, 0.000001, 0, 2.0); // 设置所有电机PID参数
    _MSDriverMaster.setServoMode(-1, smode);                // 设置所有舵机工作模式
    _MSDriverMaster.sendCmd();
}
