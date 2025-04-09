#include "MSDriverSlave.h"
#include "Wire.h"
#include "common.h"
#include "config.h"

void MSDriverSlave::init() {
    // 编码电机0
    pinMode(PIN_M0_PWM, OUTPUT);
    pinMode(PIN_M0_F1, OUTPUT);
    pinMode(PIN_M0_F2, OUTPUT);
    pinMode(PIN_M0_A, INPUT);
    pinMode(PIN_M0_B, INPUT);

    // 编码电机1
    pinMode(PIN_M1_PWM, OUTPUT);
    pinMode(PIN_M1_F1, OUTPUT);
    pinMode(PIN_M1_F2, OUTPUT);
    pinMode(PIN_M1_A, INPUT);
    pinMode(PIN_M1_B, INPUT);

    // 编码电机2
    pinMode(PIN_M2_PWM, OUTPUT);
    pinMode(PIN_M2_F1, OUTPUT);
    pinMode(PIN_M2_F2, OUTPUT);
    pinMode(PIN_M2_A, INPUT);
    pinMode(PIN_M2_B, INPUT);

    // 编码电机3
    pinMode(PIN_M3_PWM, OUTPUT);
    pinMode(PIN_M3_F1, OUTPUT);
    pinMode(PIN_M3_F2, OUTPUT);
    pinMode(PIN_M3_A, INPUT);
    pinMode(PIN_M3_B, INPUT);

    // 初始化寄存器
    // 默认电机AB引脚不测速，舵机不使能(引脚作为输入口)
    memset((void *)&reg, 0, sizeof(reg));
    memset((void *)&shadowRegMod, 0, sizeof(shadowRegMod));
    memset((void *)&shadowRegCtrl, 0, sizeof(shadowRegCtrl));
    reg.id = 0x11; // 可以任意修改

    // 电机引脚
    motor[0].init(PIN_M0_PWM, PIN_M0_F1, PIN_M0_F2);
    motor[1].init(PIN_M1_PWM, PIN_M1_F1, PIN_M1_F2);
    motor[2].init(PIN_M2_PWM, PIN_M2_F1, PIN_M2_F2);
    motor[3].init(PIN_M3_PWM, PIN_M3_F1, PIN_M3_F2);

    setModeByReg();
}

void MSDriverSlave::init(const MSDriverReg_MOD_t &mod) {
    init();

    memcpy(&reg.mode, &(mod), sizeof(MSDriverReg_MOD_t));
    memcpy(&shadowRegMod, &(reg.mode), sizeof(MSDriverReg_MOD_t));

    // 根据寄存器配置工作模式
    setModeByReg();
}

void MSDriverSlave::execute() {
    static unsigned long t0 = micros();
    if (reg.cmd) {
        // 收到变更工作模式的指令
        // 把寄存器信息复制到影子寄存器——防止被随时覆盖后影响正在进行的操作
        memcpy(&shadowRegMod, &(reg.mode), sizeof(MSDriverReg_MOD_t));
        if (reg.cmd == APPLY) {
            // 根据寄存器配置工作模式
            setModeByReg();
        } else if (reg.cmd == INIT) {
            init();
        }

        reg.cmd = 0;
    }

    // M0 ~ M3
    motorAction(0, PIN_M0_A, PIN_M0_B);
    motorAction(1, PIN_M1_A, PIN_M1_B);
    motorAction(2, PIN_M2_A, PIN_M2_B);
    motorAction(3, PIN_M3_A, PIN_M3_B);

    // S0 ~ S7
    servoAction(0, PIN_S0, shadowRegMod.smode0123);
    servoAction(1, PIN_S1, shadowRegMod.smode0123 >> 2);
    servoAction(2, PIN_S2, shadowRegMod.smode0123 >> 4);
    servoAction(3, PIN_S3, shadowRegMod.smode0123 >> 6);
    servoAction(4, PIN_S4, shadowRegMod.smode4567);
    servoAction(5, PIN_S5, shadowRegMod.smode4567 >> 2);
    servoAction(6, PIN_S6, shadowRegMod.smode4567 >> 4);
    servoAction(7, PIN_S7, shadowRegMod.smode4567 >> 6);
}

void MSDriverSlave::setModeByReg() {

    // M0 ~ M3
    motorSetup(0, shadowRegMod.mMode[0], shadowRegMod.m0Kp, shadowRegMod.m0Ki,
               shadowRegMod.m0Kd, shadowRegMod.m0KR, PIN_M0_A, PIN_M0_B,
               ReadM0ASpeed, ReadM0BSpeed);
    motorSetup(1, shadowRegMod.mMode[1], shadowRegMod.m1Kp, shadowRegMod.m1Ki,
               shadowRegMod.m1Kd, shadowRegMod.m1KR, PIN_M1_A, PIN_M1_B,
               ReadM1ASpeed, ReadM1BSpeed);
    motorSetup(2, shadowRegMod.mMode[2], shadowRegMod.m2Kp, shadowRegMod.m2Ki,
               shadowRegMod.m2Kd, shadowRegMod.m2KR, PIN_M2_A, PIN_M2_B,
               ReadM2ASpeed, ReadM2BSpeed);
    motorSetup(3, shadowRegMod.mMode[3], shadowRegMod.m3Kp, shadowRegMod.m3Ki,
               shadowRegMod.m3Kd, shadowRegMod.m3KR, PIN_M3_A, PIN_M3_B,
               ReadM3ASpeed, ReadM3BSpeed);

    // S0 ~ S7
    servoSetup(0, PIN_S0, shadowRegMod.smode0123);
    servoSetup(1, PIN_S1, shadowRegMod.smode0123 >> 2);
    servoSetup(2, PIN_S2, shadowRegMod.smode0123 >> 4);
    servoSetup(3, PIN_S3, shadowRegMod.smode0123 >> 6);
    servoSetup(4, PIN_S4, shadowRegMod.smode4567);
    servoSetup(5, PIN_S5, shadowRegMod.smode4567 >> 2);
    servoSetup(6, PIN_S6, shadowRegMod.smode4567 >> 4);
    servoSetup(7, PIN_S7, shadowRegMod.smode4567 >> 6);
}

void MSDriverSlave::motorSetup(int num, uint8_t mode, float kp, float ki,
                               float kd, float kR, uint8_t pinA, uint8_t pinB,
                               void (*interruptFunA)(void),
                               void (*interruptFunB)(void)) {

    if (mode & 0x80) {
        // 需要测速
        pinMode(pinA, INPUT);
        pinMode(pinB, INPUT);

        motor[num].setParam(kp, ki, kd, kR);
        if (mode & 0b1000001 == 0b1000001) {
            // PID控制
            motor[num].enablePID(true);
        } else {
            motor[num].enablePID(false);
        }

        attachInterrupt(pinA, interruptFunA, CHANGE);
        attachInterrupt(pinB, interruptFunB, CHANGE);
    } else {
        // 无需测速
        detachInterrupt(pinA);
        detachInterrupt(pinB);
        motor[num].enablePID(false);

        // A脚
        switch (mode & 0x07) {
        case 0b0000: // 相应的A脚作为数字输入 INPUT
        case 0b0001: // 相应的A脚作为模拟输入 INPUT
            pinMode(pinA, INPUT);
            break;
        case 0b0010: // 相应的A脚作为输入 INPUT_PLLDOWN
            pinMode(pinA, INPUT_PULLDOWN);
            break;
        case 0b0011: // 相应的A脚作为输入 INPUT_PULLUP
            pinMode(pinA, INPUT_PULLUP);
            break;
        case 0b0100: // 相应的A脚作为输出 OUTPUT LOW
        case 0b0101: // 相应的A脚作为输出 OUTPUT HIGH
            pinMode(pinA, OUTPUT);
            break;
        default:
            // 0b0110: 相应的A脚作为输出 OUTPUT_OPEN_DRAIN LOW
            // 0b0111: 相应的A脚作为输出 OUTPUT_OPEN_DRAIN HIGH
            pinMode(pinA, OUTPUT_OPEN_DRAIN);
            break;
        }

        // B脚
        switch ((mode >> 4) & 0x07) {
        case 0b0000: // 相应的B脚作为数字输入 INPUT
        case 0b0001: // 相应的B脚作为模拟输入 INPUT
            pinMode(pinB, INPUT);
            break;
        case 0b0010: // 相应的B脚作为输入 INPUT_PLLDOWN
            pinMode(pinB, INPUT_PULLDOWN);
            break;
        case 0b0011: // 相应的B脚作为输入 INPUT_PULLUP
            pinMode(pinB, INPUT_PULLUP);
            break;
        case 0b0100: // 相应的B脚作为输出 OUTPUT LOW
        case 0b0101: // 相应的B脚作为输出 OUTPUT HIGH
            pinMode(pinB, OUTPUT);
            break;
        default:
            // 0b0110: 相应的B脚作为输出 OUTPUT_OPEN_DRAIN LOW
            // 0b0111: 相应的B脚作为输出 OUTPUT_OPEN_DRAIN HIGH
            pinMode(pinB, OUTPUT_OPEN_DRAIN);
            break;
        }
    }
}

void MSDriverSlave::motorAction(int num, uint8_t pinA, uint8_t pinB) {

    uint8_t mode = shadowRegMod.mMode[num];
    int16_t inSpeed = reg.ctrl.speedM[num];
    int32_t &currspeed = reg.feedback.currspeedM[num];

    if (mode & 0x08) {
        // 禁用电机
        motor[num].setMotorTar(0x0E00);
    } else {
        // 电机有效时设置目标速度
        // 两次读到相同内容时才改变速度——防止寄存器写到一半就执行
        if (shadowRegCtrl.speedM[num] == inSpeed) {
            motor[num].setMotorTar(inSpeed);
        }
    }
    shadowRegCtrl.speedM[num] = inSpeed;

    if (mode & 0x80) {
        // 需要测速 —— AB引脚已通过外部中断自动计数
        if (mode & 0x40) {
            // 需要转换成100ms的计数或PID
            motor[num].execute();
            currspeed = motor[num]._count100ms;
        } else {
            // 需要测速 保留总数
            currspeed = motor[num]._count;
        }
    } else {
        // 无需测速 —— AB引脚可作为IO口
        uint16_t *ptr = (uint16_t *)&currspeed;
        // A脚
        switch (mode & 0x07) {
        case 0b0000: // 相应的A脚作为数字输入 INPUT
        case 0b0010: // 相应的A脚作为输入 INPUT_PLLDOWN
        case 0b0011: // 相应的A脚作为输入 INPUT_PULLUP
            *ptr = digitalRead(pinA);
            break;
        case 0b0001: // 相应的A脚作为模拟输入 INPUT
            *ptr = analogRead(pinA);
            break;
        case 0b0100: // 相应的A脚作为输出 OUTPUT LOW
        case 0b0110: // 相应的A脚作为输出 OUTPUT_OPEN_DRAIN LOW
            digitalWrite(pinA, LOW);
            break;
        default:
            // 0b0101: 相应的A脚作为输出 OUTPUT HIGH
            // 0b0111: 相应的A脚作为输出 OUTPUT_OPEN_DRAIN HIGH
            digitalWrite(pinA, HIGH);
            break;
        }

        // B脚
        ptr++;
        switch ((mode >> 4) & 0x07) {
        case 0b0000: // 相应的B脚作为数字输入 INPUT
        case 0b0010: // 相应的B脚作为输入 INPUT_PLLDOWN
        case 0b0011: // 相应的B脚作为输入 INPUT_PULLUP
            *ptr = digitalRead(pinB);
            break;
        case 0b0001: // 相应的B脚作为模拟输入 INPUT
            *ptr = analogRead(pinB);
            break;
        case 0b0100: // 相应的B脚作为输出 OUTPUT LOW
        case 0b0110: // 相应的B脚作为输出 OUTPUT_OPEN_DRAIN LOW
            digitalWrite(pinB, LOW);
            break;
        default:
            // 0b0101: 相应的B脚作为输出 OUTPUT HIGH
            // 0b0111: 相应的B脚作为输出 OUTPUT_OPEN_DRAIN HIGH
            digitalWrite(pinB, HIGH);
            break;
        }
    }
}

void MSDriverSlave::servoSetup(int num, uint8_t pin, uint8_t mode) {
    if (mode & 0b0011) {
        pinMode(pin, OUTPUT);
        servo[num].attach(pin);
    } else {
        servo[num].detach();
        // INPUT
        pinMode(pin, INPUT);
    }
}

void MSDriverSlave::servoAction(int num, uint8_t pin, uint8_t mode) {

    uint8_t angle = reg.ctrl.angleS[num];
    uint16_t &currValue = reg.feedback.currValueS[num];

    switch (mode & 0b0011) {
    case 0b001:
        // OUTPUT 高低电平
        digitalWrite(pin, angle);
        break;
    case 0b010:
        // PWM
        analogWrite(pin, angle);
        break;
    case 0b011:
        // 舵机
        angle = angle > 180 ? 180 : angle;
        servo[num].write(angle);
        break;
    default:
        // INPUT
        currValue = analogRead(pin);
    }
}

void MSDriverSlave::receiveEvent(int howMany) {
    uint8_t *regAddrPtr = &_MSDriverSlave.regAddr;
    uint8_t *regPtr = (uint8_t *)&_MSDriverSlave.reg;

    if (Wire.available()) {
        // 第一字节为寄存器地址
        *regAddrPtr = Wire.read();
    }

    while (Wire.available()) { // 读取所有接收到的
        uint8_t c = Wire.read();
        // 如果是可写寄存器
        if (*regAddrPtr >= MSD_REG_ADDR::CMD &&
            *regAddrPtr < MSD_REG_ADDR::S0_ANGLE_R) {
            // 写入寄存器
            regPtr[*regAddrPtr] = c;
        }
        (*regAddrPtr)++;
    }
}

// 主机要求读取寄存器内容
void MSDriverSlave::requestEvent() {
    // 要求的寄存器地址超出范围
    if (_MSDriverSlave.regAddr > MSD_REG_ADDR::END) {
        return;
    }

    // 最多一次传送32字节
    size_t len = MSD_REG_ADDR::END - _MSDriverSlave.regAddr + 1;
    len = len > 32 ? 32 : len;

    uint8_t *regPtr = (uint8_t *)&_MSDriverSlave.reg;
    Wire.write(&regPtr[_MSDriverSlave.regAddr], len);
}

/*
    对M0 进行转速计数
*/
void MSDriverSlave::ReadM0ASpeed() {
    int32_t *ptr = &_MSDriverSlave.motor[0]._count;
    if (digitalRead(PIN_M0_A) == HIGH) {
        if (digitalRead(PIN_M0_B) == LOW) {
            (*ptr)++;
        } else {
            (*ptr)--;
        }
    } else {
        if (digitalRead(PIN_M0_B) == LOW) {
            (*ptr)--;
        } else {
            (*ptr)++;
        }
    }

    // 防止溢出
    if ((*ptr) > 1000000000L) {
        (*ptr) = 0;
    } else if ((*ptr) < -1000000000L) {
        (*ptr) = 0;
    }
}

/*
    对M0 进行转速计数
*/
void MSDriverSlave::ReadM0BSpeed() {
    int32_t *ptr = &_MSDriverSlave.motor[0]._count;
    if (digitalRead(PIN_M0_B) == HIGH) {
        if (digitalRead(PIN_M0_A) == LOW) {
            (*ptr)--;
        } else {
            (*ptr)++;
        }
    } else {
        if (digitalRead(PIN_M0_A) == LOW) {
            (*ptr)++;
        } else {
            (*ptr)--;
        }
    }

    // 防止溢出
    if ((*ptr) > 1000000000L) {
        (*ptr) = 0;
    } else if ((*ptr) < -1000000000L) {
        (*ptr) = 0;
    }
}

/*
    对M1 进行转速计数
*/
void MSDriverSlave::ReadM1ASpeed() {
    int32_t *ptr = &_MSDriverSlave.motor[1]._count;

    if (digitalRead(PIN_M1_A) == HIGH) {
        if (digitalRead(PIN_M1_B) == LOW) {
            (*ptr)++;
        } else {
            (*ptr)--;
        }
    } else {
        if (digitalRead(PIN_M1_B) == LOW) {
            (*ptr)--;
        } else {
            (*ptr)++;
        }
    }

    // 防止溢出
    if ((*ptr) > 1000000000L) {
        (*ptr) = 0;
    } else if ((*ptr) < -1000000000L) {
        (*ptr) = 0;
    }
}

/*
    对M1 进行转速计数
*/
void MSDriverSlave::ReadM1BSpeed() {
    int32_t *ptr = &_MSDriverSlave.motor[1]._count;
    if (digitalRead(PIN_M1_B) == HIGH) {
        if (digitalRead(PIN_M1_A) == LOW) {
            (*ptr)--;
        } else {
            (*ptr)++;
        }
    } else {
        if (digitalRead(PIN_M1_A) == LOW) {
            (*ptr)++;
        } else {
            (*ptr)--;
        }
    }

    // 防止溢出
    if ((*ptr) > 1000000000L) {
        (*ptr) = 0;
    } else if ((*ptr) < -1000000000L) {
        (*ptr) = 0;
    }
}

/*
    对M2 进行转速计数
    使用中断引脚PB4
*/
void MSDriverSlave::ReadM2ASpeed() {
    int32_t *ptr = &_MSDriverSlave.motor[2]._count;
    if (digitalRead(PIN_M2_A) == HIGH) {
        if (digitalRead(PIN_M2_B) == LOW) {
            (*ptr)++;
        } else {
            (*ptr)--;
        }
    } else {
        if (digitalRead(PIN_M2_B) == LOW) {
            (*ptr)--;
        } else {
            (*ptr)++;
        }
    }

    // 防止溢出
    if ((*ptr) > 1000000000L) {
        (*ptr) = 0;
    } else if ((*ptr) < -1000000000L) {
        (*ptr) = 0;
    }
}

/*
    对M2 进行转速计数
    使用中断引脚PB3
*/
void MSDriverSlave::ReadM2BSpeed() {
    int32_t *ptr = &_MSDriverSlave.motor[2]._count;
    if (digitalRead(PIN_M2_B) == HIGH) {
        if (digitalRead(PIN_M2_A) == LOW) {
            (*ptr)--;
        } else {
            (*ptr)++;
        }
    } else {
        if (digitalRead(PIN_M2_A) == LOW) {
            (*ptr)++;
        } else {
            (*ptr)--;
        }
    }

    // 防止溢出
    if ((*ptr) > 1000000000L) {
        (*ptr) = 0;
    } else if ((*ptr) < -1000000000L) {
        (*ptr) = 0;
    }
}

/*
    对M3 进行转速计数
*/
void MSDriverSlave::ReadM3ASpeed() {
    int32_t *ptr = &_MSDriverSlave.motor[3]._count;
    if (digitalRead(PIN_M3_A) == HIGH) {
        if (digitalRead(PIN_M3_B) == LOW) {
            (*ptr)++;
        } else {
            (*ptr)--;
        }
    } else {
        if (digitalRead(PIN_M3_B) == LOW) {
            (*ptr)--;
        } else {
            (*ptr)++;
        }
    }

    // 防止溢出
    if ((*ptr) > 1000000000L) {
        (*ptr) = 0;
    } else if ((*ptr) < -1000000000L) {
        (*ptr) = 0;
    }
}

/*
    对M3 进行转速计数
*/
void MSDriverSlave::ReadM3BSpeed() {
    int32_t *ptr = &_MSDriverSlave.motor[3]._count;
    if (digitalRead(PIN_M3_B) == HIGH) {
        if (digitalRead(PIN_M3_A) == LOW) {
            (*ptr)--;
        } else {
            (*ptr)++;
        }
    } else {
        if (digitalRead(PIN_M3_A) == LOW) {
            (*ptr)++;
        } else {
            (*ptr)--;
        }
    }

    // 防止溢出
    if ((*ptr) > 1000000000L) {
        (*ptr) = 0;
    } else if ((*ptr) < -1000000000L) {
        (*ptr) = 0;
    }
}

void MSDriverSlave::getfeedBack(MSDriverReg_FB_t *buf) {
    memcpy(buf, &(reg.feedback), sizeof(MSDriverReg_FB_t));
}