#include "klm.h"


float KLM(KLM_t &data, float z) {
    data.X_ = data.X;
    data.P_= data.P + data.Q;
    data.K = data.P_ / (data.P_ + data.R);
    data.X = data.X_ + data.K * (z - data.X_);
    data.P = data.P_ - data.K * data.P_;
    return data.X;
}