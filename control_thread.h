#ifndef CONTROL_THREAD_H
#define CONTROL_THREAD_H

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QMutex>
#include <QMutexLocker>
#include <QTimer>
#include <QTime>
#include <QtMath>
#include "eso.h"
#include <cmath>

class ControlThread : public QObject
{
    Q_OBJECT
public:
    explicit ControlThread(QObject *parent = nullptr);
    ~ControlThread();
signals:
    //ROV运动状态
    void SigToMove_Forward();//前进
    void SigToMove_Backward();//后退
    void SigToMove_Left();//左平移
    void SigToMove_Right();//右平移
    void SigToMove_TurnLeft();//左转，转动指定角度的控制还未实现。
    void SigToMove_TurnRight();//右转
    //云台状态
    void SigToPtz1_TurnLeft();//单目云台左转
    void SigToPtz1_TurnRight();//单目云台右转
    void SigToPtz2_TurnLeft();//双目云台左转
    void SigToPtz2_TurnRight();//双目云台右转
    //电机状态
    void SigToMotor_Run();//收集电机运转
    void SigToMotor_Stop();//收集电机停转
    //复位状态
    void SigToReset();//停止运动，云台复位
public slots:
    void doProcessControl();//运动控制算法
    void doProcessTimerStop();//暂停定时器
    //void doProcessTimerRun();
private:
    ESO *my_eso;
    QTimer *timer_control;//控制定时器，用于推进器启动
    QTimer *timer_turn_left;//转向定时器
    QTimer *timer_turn_right;
    QTimer *timer_right;//平移定时器
    QTimer *timer_left;
    QTimer *timer_motor_run;//电机启停定时器
    QTimer *timer_motor_stop;
    //void doProcessTimerRun();


    void sleep(unsigned int msec);
    float sat(double S, double sigma);
    float psi(float x,float y);
    void psi_solve();
    void LOS_Solve();
    void PID();
    void SMC();
    void ST();

    //APF函数
    void APF_Generate();
    void APF_Calculate();



};

#endif // CONTROL_THREAD_H
