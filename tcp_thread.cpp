#include "tcp_thread.h"
#include "mainwindow.h"
//#include "control.h"

QMutex myMutex;
extern QSemaphore Sem;//定义只含一个信号灯的信号量
extern  QByteArray array;
extern quint8 cmd[42];//控制指令
extern ControlThread* control;//声明mainwindow定义的control对象
extern float Ts;

TcpThread::TcpThread(QObject *parent)
    : QObject{parent}
{
    myClient = nullptr;
    myClient = new QTcpSocket(this);//资源由父对象this回收
    timer_tcp = new QTimer(this);//定时器实例化
    timer_auto = new QTimer(this);
    timer_manual = new QTimer(this);
}
//析构函数
TcpThread::~TcpThread()
{
    delete timer_tcp;//删除定时器对象
    delete timer_auto;
    delete timer_manual;
    myClient->close();//关闭通信套接字
}

void TcpThread::sleep(unsigned int msec)
{
    //currnentTime 返回当前时间 用当前时间加上我们要延时的时间msec得到一个新的时刻
    QTime reachTime = QTime::currentTime().addMSecs(msec);
    //用while循环不断比对当前时间与我们设定的时间
    while(QTime::currentTime()<reachTime)
    {
        //如果当前的系统时间尚未达到我们设定的时刻，就让Qt的应用程序类执行默认的处理，
        //以使程序仍处于响应状态。一旦到达了我们设定的时刻，就跳出该循环，继续执行后面的语句。
        QApplication::processEvents(QEventLoop::AllEvents,100);
    }
}

void TcpThread::delayMs(unsigned int ms)
{
    QEventLoop loop;//定义一个新的事件循环
    QTimer::singleShot(ms, &loop, SLOT(quit()));//创建单次定时器，槽函数为事件循环的退出函数
    loop.exec();//事件循环开始执行，程序会卡在这里，直到定时时间到，本循环被退出
}

/*
 * *****************************************************************************************
  @ TCP处理
 * *****************************************************************************************
*/
//连接服务器，并建立槽
void TcpThread::doProcessConnectToServer(QString serverIP,QString serverPort)
{
    myClient->connectToHost(QHostAddress(serverIP),serverPort.toUInt());//建立TCP链接

    connect(myClient,SIGNAL(connected()),this,SLOT(doConnected()));//连接成功

    connect(myClient,SIGNAL(disconnected()),this,SLOT(doDisConnected()));//连接中断

    connect(myClient,SIGNAL(readyRead()),this,SLOT(doProcessRecvData()));//接收成功

    connect(this,SIGNAL(SigToControl()),control,SLOT(doProcessControl()));//control线程连接槽

    connect(this,SIGNAL(SigToStopTimer_control()),control,SLOT(doProcessTimerStop()));//切换到手动模式时，暂停control的定时器

    //tcp定时中断处理
    connect(timer_tcp,&QTimer::timeout, this, [=]()
    {
        timer_tcp->stop();//停止计时
        if(nullptr != myClient)
            Renew_Cmd(cmd);//发送最新的cmd指令
        timer_tcp->start(Ts*1000);//重启计时100ms
    });
    //manual中断处理
    connect(timer_manual,&QTimer::timeout, this, [=]()
    {
        timer_manual->stop();//停止计时
        Initial_Manual(cmd);//初始化帧1
    });
    //auto中断处理
    connect(timer_auto,&QTimer::timeout, this, [=]()
    {
        timer_auto->stop();//停止计时
        Initial_Auto(cmd);//初始化帧2
        emit SigToControl();//发送自动控制信号给control线程
    });
}

//连接成功
void TcpThread::doConnected(){
    emit SigToConnected();//发送连接成功信号，给主线程
    sleep(2000);
    Initial_Manual(cmd);//默认为手动模式初始化.也可改为自动模式
    Renew_Cmd(cmd);//发送一次最新的cmd指令
    timer_tcp->start(Ts*1000);//启动定时器，100ms进入一次定时中断

//    while(nullptr != myClient)//循环发送方式，耗费资源
//    {
//        Renew_Cmd(cmd);
//        sleep(200);
//    }
}

//连接中断
void TcpThread::doDisConnected(){
    emit SigDisConnected();//发送连接断开信号，给主线程
}

//手动断开（默认在手动控制模式下）
void TcpThread::Manual_DisConnected()
{
    if(nullptr == myClient)//通信套接字已为空时不能再断开，直接返回
        return;
    Initial_Manual(cmd);//指令数组复位
    sleep(200);//等待复位cmd发送完成
    timer_tcp->stop();//暂停计时
    myClient->disconnectFromHost();//断开与服务器的连接
    myClient->close();//关闭通信套接字
}

//手动发送
void TcpThread::Manual_Send(QString str)
{
    if(nullptr == myClient)//连接失败则不发送
        return;
    //将信息写入到通信套接字
    myClient->write(str.toUtf8().data());//先转 QByteArrat,再转const* char
}

//手动控制模式
void TcpThread::doProcessManual()
{
    if(nullptr == myClient)//连接失败则不发送
        return;
    emit SigToStopTimer_control();//通知control，推进器设为静止态，且暂停其定时器
    timer_manual->start(1000);//等待eb91格式的控制位0发送完成
}

//自动控制模式
void TcpThread::doProcessAuto()
{
    if(nullptr == myClient)//连接失败则不发送
        return;
    timer_auto->start(3000);//等待eb90格式的控制位1发送完成
    //Initial_Auto(cmd);//直接在control线程中初始化，防止信号槽之间反复横跳
}

//数据接收
void TcpThread::doProcessRecvData()
{
    if(nullptr == myClient)
        return;
    //Sem.acquire();  //获取二值信号量。可确保读取完当前帧，才接收下一帧。
    QMutexLocker locker(&myMutex);//加锁,利用QMutexLocker管理,在该函数结束时自动析构解锁

    array = (QByteArray)myClient->readAll();//读取缓冲区数据
    emit SigRecvFinished();//发送读取完成信号，给主线程
    //Sem.release();  //释放信号量
}

/*
 * *****************************************************************************************
  @ 发送指令数组
 * *****************************************************************************************
*/
//生成CRC16位校验码
quint16 TcpThread::crc16(const quint8*data, quint16 len)
{
    quint16 crc = 0xFFFF;
    for (quint8 i = 0; i < len; i++)
    {
        crc ^= data[i];
        for(int j = 0; j < 8; j++)
        {
            quint16 carry_flag = crc & 0x0001;
            crc >>= 1;
            if(carry_flag)
            {
                crc ^= 0xA001;
            }
        }
    }
    return crc;
}

//手动模式cmd数组初始化
void TcpThread::Initial_Manual(quint8* cmd)
{
    if(nullptr == myClient)//连接失败则不发送
        return;
    Sem.acquire();
    cmd[0] = 0xeb;
    cmd[1] = 0x90;
    cmd[2] = 0x2a;//帧长42字节，若只看数据位，则需修改
    for(int i = 3;i <= 6;i ++)//摇杆默认值128 - 0x80
        cmd[i] = 0x80;
    for(int i = 7;i <= 22;i++)//默认值0
        cmd[i] = 0x00;
    cmd[23] = 0x00;//机器人控制方式：手动
    for(int i = 24;i <= 25;i ++)//云台默认90度，128 - 0x80
        cmd[i] = 0x80;
    for(int i = 26;i <= 29;i ++)//推进器默认值128 - 0x80
        cmd[i] = 0x80;
    for(int i = 30;i <= 37;i ++)//预留位0
        cmd[i] = 0;
    quint16 crc16_out = crc16(cmd,38);//生成0-37数据字节的校验码,2字节
    cmd[38] = (quint8)((0xff00&crc16_out)>>8);//高8位
    cmd[39] = (quint8)(0x00ff&crc16_out);//低8位
    cmd[40] = 0x0d;//固定帧尾
    cmd[41] = 0x0a;
    Sem.release();
}
//自动模式cmd数组初始化
void TcpThread::Initial_Auto(quint8* cmd)
{
    if(nullptr == myClient)//连接失败则不发送
        return;
    Sem.acquire();
    cmd[0] = 0xeb;//帧头不一样
    cmd[1] = 0x91;
    cmd[2] = 0x2a;//帧长42字节，若只看数据位，则需修改
    for(int i = 3;i <= 22;i++)//默认值0
        cmd[i] = 0x00;
    cmd[23] = 0x01;//机器人控制方式：自动
    for(int i = 24;i <= 25;i ++)//云台默认90度，128 - 0x80
        cmd[i] = 0x80;
    for(int i = 26;i <= 29;i ++)//推进器默认值128 - 0x80
        cmd[i] = 0x80;
    for(int i = 30;i <= 37;i ++)//预留位0
        cmd[i] = 0;
    quint16 crc16_out = crc16(cmd,38);//生成0-37数据字节的校验码,2字节
    cmd[38] = (quint8)((0xff00&crc16_out)>>8);//高8位
    cmd[39] = (quint8)(0x00ff&crc16_out);//低8位
    cmd[40] = 0x0d;//固定帧尾
    cmd[41] = 0x0a;
    Sem.release();
}
//发送最新的cmd数组
void TcpThread::Renew_Cmd(quint8* cmd)
{
    Sem.acquire();  //获取二值信号量。
    quint16 crc16_out = crc16(cmd,38);//生成0-37数据字节的校验码,2字节
    cmd[38] = (quint8)((0xff00&crc16_out)>>8);//高8位
    cmd[39] = (quint8)(0x00ff&crc16_out);//低8位
    QByteArray cmd_send;
    cmd_send.resize(42);
    for(int i = 0;i < 42;i ++)
        cmd_send[i] = cmd[i];
    myClient->write(cmd_send);//发送指令
    Sem.release();  //释放信号量
    //qDebug()<<cmd_send.toHex();
}

//【PC接收第59帧】
//eb90240500010004002d004d3b0000020e0f0c27ff29010074ffac430105001bcdca0d0a
