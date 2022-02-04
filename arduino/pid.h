#ifndef _PID_H
#define _PID_H

#include <stdint.h>

#define _P	(0.2)
#define _I	(0.02)
#define _D	(0.002)

class pid
{
private:
    /* data */
    float Kp;                       //比例系数Proportional
    float Ki;                       //积分系数Integral
    float Kd;                       //微分系数Derivative

    int Ek;                       //当前误差
    int Ek1;                      //前一次误差 e(k-1)
    int Ek2;                      //再前一次误差 e(k-2)
    int LocSum;                   //累计积分位置

public:
    pid(/* args */);
    ~pid();
    
    int run(uint16_t SetValue, uint32_t ActualValue);
};

#endif
