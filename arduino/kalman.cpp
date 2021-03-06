#include "kalman.h"

float Kalman::run(double angle_m,double gyro_m) {  
  angle+=(gyro_m-q_bias) * dt;  
  angle_err = angle_m - angle;  
  Pdot[0]=Q_angle - P[0][1] - P[1][0];  
  Pdot[1]=- P[1][1];  
  Pdot[2]=- P[1][1];  
  Pdot[3]=Q_gyro;  
  P[0][0] += Pdot[0] * dt;  
  P[0][1] += Pdot[1] * dt;  
  P[1][0] += Pdot[2] * dt;  
  P[1][1] += Pdot[3] * dt;  
  PCt_0 = C_0 * P[0][0];  
  PCt_1 = C_0 * P[1][0];  
  E = R_angle + C_0 * PCt_0;  
  K_0 = PCt_0 / E;  
  K_1 = PCt_1 / E;  
  t_0 = PCt_0;  
  t_1 = C_0 * P[0][1];  
  P[0][0] -= K_0 * t_0;  
  P[0][1] -= K_0 * t_1;  
  P[1][0] -= K_1 * t_0;  
  P[1][1] -= K_1 * t_1;  
  angle += K_0 * angle_err; //最优角度  
  q_bias += K_1 * angle_err;  
  angle_dot = gyro_m-q_bias;//最优角速度
  return angle;
}