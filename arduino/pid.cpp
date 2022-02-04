#include "pid.h"

pid::pid(/* args */) {
	LocSum = 0;         	//误差累计
	Ek1 = 0;        		//上次偏差值
	Ek2 = 0;        		//上上次偏差值

	Kp = _P;                //比例常数
	Ki = _I;                //积分常数
	Kd = _D;                //微分常数
}

pid::~pid() {
}

/************************************************
函数名称 ： run
功    能 ： PID位置(Location)计算
参    数 ： SetValue ------ 设置值(期望值)
            ActualValue --- 实际值(反馈值)
返 回 值 ： PIDLoc -------- PID位置
*************************************************/
int pid::run(uint16_t SetValue, uint32_t ActualValue){

    int PIDLoc;                                  //位置

    Ek = SetValue - ActualValue;
    LocSum += Ek;                         //累计误差
    if(LocSum>65535) LocSum = 65535;
    if(LocSum<-65535) LocSum = -65535;

    PIDLoc = Kp * Ek + (Ki * LocSum) + Kd * (Ek1 - Ek);
    Ek1 = Ek;

    return PIDLoc;
}
