#ifndef TCP_THREAD_H
#define TCP_THREAD_H

#include <QObject>

#include <QTcpSocket>
#include <QHostAddress>
#include <QAbstractSocket>
#include <QThread>
#include <QDebug>
#include <QMutex>
#include <QMutexLocker>
#include <QDataStream>
#include <QTime>
#include <QTimer>
#include <QApplication>

class TcpThread : public QObject
{
    Q_OBJECT
public:
    explicit TcpThread(QObject *parent = nullptr);
    ~TcpThread();
signals:
    void SigToConnected();
    void SigDisConnected();
    void SigRecvFinished();
    void SigToControl();//给control线程
    void SigToStopTimer_control();//暂停control的定时器
public slots:
    void doProcessConnectToServer(QString,QString);
    void doConnected();
    void doDisConnected();
    void doProcessRecvData();
    void doProcessManual();//手动控制模式
    void doProcessAuto();//自动控制模式
    void Manual_DisConnected();//手动断开连接
    void Manual_Send(QString);//发送文本

private:
    QTcpSocket *myClient;
    QTimer *timer_tcp;
    QTimer *timer_manual;
    QTimer *timer_auto;
    //发送指令处理
    void Renew_Cmd(quint8* cmd);//发送最新的cmd数组
    void Initial_Manual(quint8* cmd);//手动模式初始化
    void Initial_Auto(quint8* cmd);//自动模式初始化
    void sleep(unsigned int msec);
    void delayMs(unsigned int ms);
    quint16 crc16(const quint8*data, quint16 len);//生成CRC16位校验码
};

#endif // TCP_THREAD_H
