#include "eso.h"

ESO::ESO()
{
    //参数初始化
    w0=5;
    h=0.1;//采样间隔0.1秒
    z1=0;
    z2=0;
    z3=0;
    psi_last = 0;//上一时刻的角度
}

// 目前是一维ESO（仅psi), 后续轨迹跟踪可以加入x,y
void ESO::eso_calculate(float psi,float m_r,float tau_n, float r)
{
    float Beita_01 = 3 * w0;
    float Beita_02 = 3 * w0 * w0;
    float Beita_03 = w0 * w0 * w0;

    psi = psi * M_PI/180; // 转为弧度
    //临界角处理，防止psi突变时，ESO跟不上
    if (psi - psi_last > M_PI)
        psi = psi - 2*M_PI;
    else if(psi - psi_last < -M_PI)
        psi = psi + 2*M_PI;
    //推力去死区
//    if(tau_n > 0)
//        tau_n = tau_n - 10;
//    else if(tau_n < 0)
//        tau_n = tau_n + 10;

    //离散化一维ESO
    float e = z1 - psi;
    z1 += (z2 - Beita_01 * e) * h;
    z2 += (z3 - Beita_02 * e + tau_n/m_r) * h;// ESO
    z3 += - Beita_03 * e * h;

    // 扰动观测z1 限幅
    if (z1 >= 2*M_PI)
        z1 = 2*M_PI;
    else if (z1 < -M_PI)
        z1 = -M_PI;
    // 扰动观测z2 限幅
    if (z2 > 1)
        z2 = 1;
    else if (z2 < -1)
        z2 = -1;
    // 扰动观测z3 限幅
    if (z3 > 20)
        z3 = 20;
    else if (z3 < -20)
        z3 = -20;

    //更新角度
    psi_last = psi;
}
