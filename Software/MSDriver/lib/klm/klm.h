#include <Arduino.h>

#ifndef KLM_H
#define KLM_H

struct KLM_t {
    float P = 1;
    float P_;
    float X = 0;
    float X_;
    float K = 0;
    float Q = 0.01;
    float R = 0.2;
};

float KLM(KLM_t &data, float z);

#endif