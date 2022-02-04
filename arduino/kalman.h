#ifndef _M_KALMAN_H
#define _M_KALMAN_H

#include <Arduino.h>

class Kalman {
    //卡尔曼滤波参数与函数  
    float angle, angle_dot;//角度和角速度  
    float angle_0, angle_dot_0;//采集来的角度和角速度  
    float dt=50*0.001;//注意：dt的取值为kalman滤波器采样时间  
    //一下为运算中间变量  
    float P[2][2] = {{ 1, 0 },  
                { 0, 1 }};  
    float Pdot[4] ={ 0,0,0,0};  
    float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度,角速度数据置信度  
    float R_angle=0.5 ,C_0 = 0.1;   
    float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

public:
    Kalman(uint32_t sample_interval){
        angle = .0;
        angle_dot = .0;
        angle_0 = .0;
        angle_dot_0 = .0;
        
        q_bias = .0;
        angle_err = .0;
        PCt_0 = .0;
        PCt_1 = .0;
        E = .0;
        K_0 = .0;
        K_1 = .0;
        t_0 = .0;
        t_1 = .0;
        
        dt = sample_interval*0.001;
    }

    float run(double angle_m, double gyro_m); 
};

#endif