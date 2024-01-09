#include "HashFunc.h"

uint32_t FloatToRaw32Bit(float x) {
    union Data_32
    {
        uint32_t x_i;
        float x_f;
    };
    Data_32 tmp;
    tmp.x_f = x;
    return tmp.x_i;
}

uint64_t DoubleToRaw64Bit(double x) {
    union Data_64
    {
        uint64_t x_l;
        double x_d;
    };
    Data_64 tmp;
    tmp.x_d = x;
    return tmp.x_l;
}
