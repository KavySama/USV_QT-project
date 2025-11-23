#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qobject.h>

TcpThread* tcp = new TcpThread;  //创建TcpThread类对象tcp。必须new出来，否则主线程信号无法传达到次线程中！
ControlThread* control = new ControlThread;//创建ControlThread类对象control

QByteArray array;//全局QByteArray数组,用来存储接收数据
QSemaphore Sem(1);//定义只含一个信号灯的信号量
extern QMutex myMutex;
extern QMutex myMutex_xy;
extern QVector<QVector<double>> Rov_location;
extern QVector<QVector<double>> P_location;
bool ok;
int clear_rsv = 0;//接收计数器
int button_camera_flag = 0;//相机按钮的标志位
int button_control_flag = 0;//控制模式按钮的标志位
float distance[4];
quint8 yaw_signed = 0;//姿态角符号位
quint8 roll_signed = 0;
quint8 pitch_signed = 0;
quint8 angle_signed = 0;
quint8 cmd[42];//控制指令数组
QString rov_lat;
QString rov_lon;

// xy坐标及速度
float Rov_x = 0;//机器人坐标, 同 x,y
float Rov_y = 0;
float v_x = 0;
float v_y = 0;
float Rov_u = 0;//机器人速度u
float Rov_r = 0;//机器人速度r, 同 Rov_psi_diff
// psi角度 滤波
float Rov_psi = 0;
float Rov_psi_last = 0;
#define FILTER_N 16
int coe[FILTER_N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};    // 加权系数表
float filter_buf[FILTER_N + 1];
// r角速度差分计算
float Rov_psi_diff = 0; // r
float Rov_psi_diff_last = 0;
float Rov_psi_diff_2 = 0; // r'
float Ts = 0.1;// 控制周期
// serverwidget同步标记
float matlab_send_flag = 0;
// 侧向速度
float Rov_v = 0; // 机器人坐标系侧向速度 v

MainWindow::MainWindow(QWidget *parent)//构造函数
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    configWindow = new map_set;
    ekf = new EKFLocalization();
    EKFState initSt{0,0,0,0,0};
    ekf->reset(initSt);

    m_webView = new QWebEngineView(this);
    m_webView->setGeometry(20,10,971,571);//设置位置和大小
    //m_webView->resize(800,500);
    array.resize(510);
    thread_init();//子线程初始化
    connect_init();//连接初始化


//    QFile f("E:\\control_data.txt");
//    QTextStream out(&f);
//    if(!f.open(QIODevice::WriteOnly | QIODevice::Text))
//    {
//        qDebug() << ("打开文件失败");
//    }
//    float delt=21.6;
//    QString y1 = QString::number(delt,'f',1);
//    QString y2 = QString::number(0x03,10);
//    QString y3 = QString::number(0xff,10);
//    //写入文本文件
//    out << y1<<" "<<y2<<" "<<y3<< endl;
//    f.close();
//    qDebug() << ("文件已关闭");
}

MainWindow::~MainWindow()
{
    tcp_thread.quit();//等待tcp线程的结束
    tcp_thread.wait();
    control_thread.quit();//等待控制线程结束
    control_thread.wait();
    delete ui;
}

//===========================================function==================================================
void MainWindow::thread_init()
{
    tcp->moveToThread(&tcp_thread);
    tcp_thread.start();//开启tcp线程

    control->moveToThread(&control_thread);
    control_thread.start();//开启control线程
}

void MainWindow::connect_init(){

    //tcp线程连接槽
    connect(this,SIGNAL(SigToConnect(QString,QString)),tcp,SLOT(doProcessConnectToServer(QString,QString)));  //建立连接槽
    connect(tcp,SIGNAL(SigToConnected()),this,SLOT(doProcessConnected()));      //连接成功槽
    connect(tcp,SIGNAL(SigDisConnected()),this,SLOT(doProcessDisconnected()));  //连接断开槽
    connect(tcp,SIGNAL(SigRecvFinished()),this,SLOT(doProcessShow()));  //数据显示槽
    connect(ui->button_close,&QPushButton::clicked,[=]()
    {
        emit SigToClose();
    });

    connect(this,SIGNAL(SigToClose()),tcp,SLOT(Manual_DisConnected()));  //手动断开连接槽
    connect(this,SIGNAL(SigToSend(QString)),tcp,SLOT(Manual_Send(QString)));  //tcp发送槽
    connect(this,SIGNAL(SigToManual()),tcp,SLOT(doProcessManual()));  //手动控制模式
    connect(this,SIGNAL(SigToAuto()),tcp,SLOT(doProcessAuto()));  //自动控制模式
    connect(&tcp_thread,SIGNAL(finished()),tcp,SLOT(deleteLater()));    //资源回收槽，线程结束后自动销毁。
    //control线程连接槽
    connect(control,SIGNAL(SigToMove_Forward()),this,SLOT(doProcessMove_Forward()));
    connect(control,SIGNAL(SigToMove_Backward()),this,SLOT(doProcessMove_Backward()));
    connect(control,SIGNAL(SigToMove_Left()),this,SLOT(doProcessMove_Left()));
    connect(control,SIGNAL(SigToMove_Right()),this,SLOT(doProcessMove_Right()));
    connect(control,SIGNAL(SigToMove_TurnLeft()),this,SLOT(doProcessMove_TurnLeft()));
    connect(control,SIGNAL(SigToMove_TurnRight()),this,SLOT(doProcessMove_TurnRight()));
    connect(control,SIGNAL(SigToPtz1_TurnLeft()),this,SLOT(doProcessPtz1_TurnLeft()));
    connect(control,SIGNAL(SigToPtz1_TurnRight()),this,SLOT(doProcessPtz1_TurnRight()));
    connect(control,SIGNAL(SigToPtz2_TurnLeft()),this,SLOT(doProcessPtz2_TurnLeft()));
    connect(control,SIGNAL(SigToPtz2_TurnRight()),this,SLOT(doProcessPtz2_TurnRight()));
    connect(control,SIGNAL(SigToMotor_Run()),this,SLOT(doProcessMotor_Run()));
    connect(control,SIGNAL(SigToMotor_Stop()),this,SLOT(doProcessMotor_Stop()));
    connect(control,SIGNAL(SigToReset()),this,SLOT(doProcessReset()));
    connect(&control_thread,SIGNAL(finished()),control,SLOT(deleteLater()));    //资源回收槽，线程结束后自动销毁。
    //map_set
    connect(this,SIGNAL(rov_ll(QString,QString)),configWindow,SLOT(doProcessRov_ll(QString,QString)));
    connect(configWindow,SIGNAL(rov_location(double,double)),this,SLOT(doProcessRov_location(double,double)));

    //“连接机器人”按钮映射
    connect(ui->button_robot,&QPushButton::clicked,[=]()
    {
        on_button_connect_clicked();//映射tcp连接
        ui->button_robot->setText("机器人已连接");
    });
    //“连接双目摄像头”按钮映射
    connect(ui->button_camera,&QPushButton::clicked,[=]()
    {
        if(button_camera_flag == 0)
        {
            on_button_connect_2_clicked();//映射http连接
            ui->button_camera->setText("摄像头已连接");
            button_camera_flag ++;
        }
        else if(button_camera_flag == 1)
        {
            on_button_close_2_clicked();//映射http断开
            ui->button_camera->setText("摄像头断开");
            button_camera_flag = 0;//标志复位
        }
    });
    //“控制模式”按钮映射
    connect(ui->button_convert,&QPushButton::clicked,[=]()
    {
        if(button_control_flag == 0)
        {
            cmd[23] = 0x00;//控制标志位置0，通知下位机准备转换模式
            //初始化帧1并发送
            ui->button_convert->setText("手动控制模式");
            ui->textEdit_Write->append("已切换【手动控制模式】：");
            button_control_flag ++;
            emit SigToManual();//发送手动控制信号
            //qDebug()<<"hand";
        }
        else if(button_control_flag == 1)
        {
            cmd[23] = 0x01;//控制标志位置1，通知下位机准备转换模式
            //初始化帧2并发送
            ui->button_convert->setText("自动控制模式");
            ui->textEdit_Write->setText("已切换【自动控制模式】：");
            button_control_flag = 0;//标志复位
            emit SigToAuto();//发送自动控制信号，tcp线程会发送3s（约30次）的过渡帧
            //qDebug()<<"auto";
        }
        //QMessageBox::information(this,"注意","切换成功！");
    });
}

void MainWindow::sleep(unsigned int msec)
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

/*
 * *****************************************************************************************
  @ 键盘事件处理，用于手动控制模式
 * *****************************************************************************************
*/

// 键盘按下事件
void MainWindow::keyPressEvent(QKeyEvent * event)
{
    //emit keyPressed(event->key());
    Sem.acquire();  //获取信号量。修改cmd完成后，renew函数才能发送
    // 普通键
    switch (event->key())
    {
    //前进
    case Qt::Key_W:
        cmd[3] = 128 - 35;//模拟左右摇杆前推，均设为0x70--112
        cmd[5] = 128 - 35;
        qDebug() <<"ROV前进";
        ui->textEdit_Write->append("ROV前进");
        break;
    //后退
    case Qt::Key_S:
        cmd[3] = 180;//模拟左右摇杆后推，均设为0x90--144
        cmd[5] = 180;
        qDebug() <<"ROV后退";
        ui->textEdit_Write->append("ROV后退");
        break;
    //左旋转
    case Qt::Key_Q:
        cmd[3] = 128 - 35;//模拟右摇杆前推，左摇杆后推
        cmd[5] = 128 + 35;
        qDebug() <<"ROV左旋转";
        ui->textEdit_Write->append("ROV左旋转");
        break;
    //右旋转
    case Qt::Key_E:
        cmd[3] = 128 + 35;//模拟右摇杆后推，左摇杆前推
        cmd[5] = 128 - 35;
        qDebug() <<"ROV右旋转";
        ui->textEdit_Write->append("ROV右旋转");
        break;
    //云台left
    case Qt::Key_A:
        cmd[4] = 0x70;//模拟左右摇杆左推，均设为0x70--112
        cmd[6] = 0x70;
        qDebug() <<"云台左移";
        ui->textEdit_Write->append("云台左移");
        break;
    //云台right
    case Qt::Key_D:
        cmd[4] = 0x90;//模拟左右摇杆右推，均设为0x90--144
        cmd[6] = 0x90;
        qDebug() <<"云台右移";
        ui->textEdit_Write->append("云台右移");
        break;
    //ESC键
    case Qt::Key_Escape:
        qDebug() <<"ESC";
        cmd[3] = 0x80;//摇杆，云台复位
        cmd[4] = 0x80;
        cmd[5] = 0x80;
        cmd[6] = 0x80;
        cmd[17] = 0x00;
        qDebug() <<"ROV停止";
        ui->textEdit_Write->append("ROV停止");
        break;
    //回车键
    case Qt::Key_Return:
        cmd[17] = 0x01;
        qDebug() <<"收集电机运转！";
        ui->textEdit_Write->append("收集电机运转！");
        break;
    //Shift键
    case Qt::Key_Shift:
        cmd[17] = 0x00;
        qDebug() <<"收集电机停止！";
        ui->textEdit_Write->append("收集电机停止！");
        break;
    }
    Sem.release();  //释放信号量
}

// 键盘释放事件
void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    //emit keyReleased(event->key());

//    if(event->key() == Qt::Key_Up)
//    {
//        ui->textEdit_Write->append("已停止前进！");
//        cmd[3] = 0x80;//推进器复位
//        cmd[5] = 0x80;
//    }
//    if(event->key() == Qt::Key_Down)
//    {
//        ui->textEdit_Write->append("已停止后退！");
//        cmd[3] = 0x80;//推进器复位
//        cmd[5] = 0x80;
//    }
//    if(event->key() == Qt::Key_Left)
//    {
//        ui->textEdit_Write->append("已停止左移！");
//        cmd[4] = 0x80;//推进器复位
//        cmd[6] = 0x80;
//    }
//    if(event->key() == Qt::Key_Right)
//    {
//        ui->textEdit_Write->append("已停止右移！");
//        cmd[4] = 0x80;//推进器复位
//        cmd[6] = 0x80;
//    }
}

/*
 * *****************************************************************************************
  @ 自动模式下，运动状态显示
 * *****************************************************************************************
*/
void MainWindow::doProcessMove_Forward()
{
    ui->textEdit_Write->append("ROV正在前进");
}
void MainWindow::doProcessMove_Backward()
{
    ui->textEdit_Write->append("ROV正在后退");
}
void MainWindow::doProcessMove_Left()
{
    ui->textEdit_Write->append("ROV正在向左平移");
}
void MainWindow::doProcessMove_Right()
{
    ui->textEdit_Write->append("ROV正在向右平移");
}
void MainWindow::doProcessMove_TurnLeft()
{
    ui->textEdit_Write->append("ROV正在左转");
}
void MainWindow::doProcessMove_TurnRight()
{
    ui->textEdit_Write->append("ROV正在右转");
}
void MainWindow::doProcessPtz1_TurnLeft()
{
    ui->textEdit_Write->append("单目云台正在左转");
}
void MainWindow::doProcessPtz1_TurnRight()
{
    ui->textEdit_Write->append("单目云台正在右转");
}
void MainWindow::doProcessPtz2_TurnLeft()
{
    ui->textEdit_Write->append("双目云台正在左转");
}
void MainWindow::doProcessPtz2_TurnRight()
{
    ui->textEdit_Write->append("双目云台正在右转");
}
void MainWindow::doProcessMotor_Run()
{
    ui->textEdit_Write->append("电机正在运转");
}
void MainWindow::doProcessMotor_Stop()
{
    ui->textEdit_Write->append("电机停转");
}
void MainWindow::doProcessReset()
{
    ui->textEdit_Write->append("ROV复位");
}
/*
 * *****************************************************************************************
  @ ROV客户端
 * *****************************************************************************************
*/

//发起tcp连接
void MainWindow::on_button_connect_clicked()
{
    if(nullptr==ui->lineEdit_IP || nullptr==ui->lineEdit_Port)
        return ;
    serverIP = ui->lineEdit_IP->text();
    serverPort = ui->lineEdit_Port->text();

    emit SigToConnect(serverIP,serverPort);//发送连接信号，给tcp线程
}

//tcp连接成功显示
void MainWindow::doProcessConnected()
{
    ui->textEdit_Read->setText("机器人连接成功！");
    ui->textEdit_Read->append("通知机器人回传数据....");
    //sleep(1000);
    ui->textEdit_Read->append("机器人数据开始回传");
    //ui->button_connect->setEnabled(false);//禁用连接按钮
}

//tcp连接中断显示
void MainWindow::doProcessDisconnected()
{
    QMessageBox::warning(this,"提示","与机器人连接中断！");
    ui->textEdit_Read->setText("与服务器连接已断开！");
    ui->button_robot->setText("机器人断开");
    ui->button_convert->setText("控制模式已关闭");
    ui->textEdit_Write->setText("");//清空发送框
    clear_rsv = 0;//接收计数复位
    button_control_flag = 0;//控制标志复位
}

//tcp接收显示
void MainWindow::doProcessShow()
{
    QMutexLocker locker(&myMutex);//加锁

    QString hexString = array.toHex();//转16进制字符串
    quint8 data[255];//接收待处理数据。用QByteArray易错！！
    for(int i=0;i<255;i++)
        data[i]=hexString.mid(i*2,2).toInt(&ok,16);//依次取hexString两个字符拼成一个整型变量（16进制表示）
    if((data[2]==0xff)&&(data[253]==0x0d)&&(data[254]==0x0a))//帧长和帧尾吻合
    {
        //CRC校验
        quint16 crc16_out = crc16(data,251);//校验范围 0-250字节
        //quint8 crc_H = (0xff00&crc16_out)>>8;//高8位
        quint8 crc_L = 0x00ff&crc16_out;//低8位
        //qDebug()<<"crc_H is"<<QString::number(crc_H,16);
        //qDebug()<<"crc_L is"<<QString::number(crc_L,16);
        if(data[252]==crc_L)//crc校验也吻合，只看低位，高位一直有问题
        {
            display_ROV(data);//将ROV回传数据显示在UI窗口
            QString clear_rsv_num = QString::number(clear_rsv,10);
            clear_rsv_num = QString("【PC接收第%1帧】").arg(clear_rsv_num);
            ui->textEdit_Read->append(clear_rsv_num);
            ui->textEdit_Read->append(hexString);
            clear_rsv++;
            if(clear_rsv % 500 == 0)//每成功接收500条信息清空一次窗口
            {
                ui->textEdit_Read->setText("");
            }
        }else
            return;//丢弃
    }
    else
        return;//丢弃
    array.clear();//清空全局数组
}

//发送文本数据
void MainWindow::on_button_send_clicked()
{
    str0 = ui->textEdit_Write->toPlainText();//从写文本框获取信息
    emit SigToSend(str0);//发送文本信号，给tcp线程
}

//清空显示框
void MainWindow::on_button_clear_clicked()
{
    ui->textEdit_Read->setText("");
}

//清空发送框
void MainWindow::on_button_clear_send_clicked()
{
    ui->textEdit_Write->setText("");
}


/*
 * *****************************************************************************************
  @ 双目摄像头客户端
 * *****************************************************************************************
*/
void MainWindow::on_button_connect_2_clicked()
{
    QString url = ui->lineEdit_IP_2->text();
    if (!url.isEmpty())
    {
        m_webView->load(url);
        m_webView->show();
        //m_webView->setMaximumSize(971, 571);
        //m_webView->setMinimumSize(971, 571);
        //m_webView->showMaximized();
    }
}

void MainWindow::on_button_close_2_clicked()
{
    QString url = "";
    m_webView->load(url);
    m_webView->show();
    //m_webView->showMaximized();
}

/*
 * *****************************************************************************************
  @ 传感器数据显示处理
 * *****************************************************************************************
*/
//ROV回传数据综合处理
void MainWindow::display_ROV(quint8 *data)
{        
    //姿态角
    qint16 yaw = (data[3]<<8)+data[4];//高低位合并
    qint16 roll = (data[5]<<8)+data[6];
    qint16 pitch = (data[7]<<8)+data[8];
    display_angle(yaw,roll,pitch);
    //距离传感器
    qint8 front = data[9];
    qint8 after = data[10];
    qint8 left = data[11];
    qint8 right = data[12];
    display_distance(front,after,left,right);
    //云台
    qint8 doub = data[13];
    qint8 single = data[14];
    display_PTZ(doub,single);
    //挡位
    qint8 handle = data[15];
    display_handle(handle);
    //gps时间
    quint8 hour = data[16];
    quint8 min = data[17];
    quint8 sec = data[18];
    display_gps_time(hour,min,sec);

    //gps纬度
    quint8 integer = data[19];
    int decimal = (data[20]<<16)+(data[21]<<8)+data[22];//小数高位在前
    quint8 n_s = data[23];
    int k = 0;//找到$GNGGA原始语句
    int i =33;
    while(k<3)
    {
        if(data[i]==0x24)k++;
        i++;
    }
    float lat_m1 = (data[i+19]-48)*10+ (data[i+20]-48)*1;//纬度 分的整数部分
    //float lat_m2 = (data[i+22]-48)/10+ (data[i+23]-48)/100+ (data[i+24]-48)/10^3+ (data[i+25]-48)/10^4+ (data[i+26]-48)/10^5+ (data[i+27]-48)/10^6;//纬度 分的小数部分
    float lat_m21 = (data[i+22]-48);lat_m21 /= 10;
    float lat_m22 = (data[i+23]-48);lat_m22 /= 100;
    float lat_m23 = (data[i+24]-48);lat_m23 /= 1000;
    float lat_m24 = (data[i+25]-48);lat_m24 /= 10000;
    float lat_m25 = (data[i+26]-48);lat_m25 /= 100000;
    float lat_m26 = (data[i+27]-48);lat_m26 /= 1000000;
    float lat_m2 = lat_m21 + lat_m22 + lat_m23 + lat_m24 + lat_m25 + lat_m26;
    double lat_m = lat_m1+lat_m2;//纬度 分
    lat_m = lat_m/60 * 10000000;//转为度
    //qDebug()<<lat_m;
    display_gps_latitude(integer,decimal,n_s,lat_m);

    //gps经度
    quint8 integer2 = data[24];
    int decimal2 = (data[25]<<16)+(data[26]<<8)+data[27];//小数高位在前
    quint8 w_e = data[28];
    //qDebug()<<decimal2;
    float lon_m1 = (data[i+34]-48)*10+ (data[i+35]-48)*1;//经度 分的整数部分
    float lon_m21 = (data[i+37]-48);lon_m21 /= 10;
    float lon_m22 = (data[i+38]-48);lon_m22 /= 100;
    float lon_m23 = (data[i+39]-48);lon_m23 /= 1000;
    float lon_m24 = (data[i+40]-48);lon_m24 /= 10000;
    float lon_m25 = (data[i+41]-48);lon_m25 /= 100000;
    float lon_m26 = (data[i+42]-48);lon_m26 /= 1000000;
    float lon_m2 = lon_m21 + lon_m22 + lon_m23 + lon_m24 + lon_m25 + lon_m26;//经度 分的小数部分
    float lon_m = lon_m1+lon_m2;//经度 分
    lon_m = lon_m/60 * 10000000;//转为度
    display_gps_longitude(integer2,decimal2,w_e,lon_m);
    emit rov_ll(rov_lat,rov_lon);//经纬度传送给map_set，转为xy坐标
    //qDebug()<<rov_lat<<" "<<rov_lon;

    //计算线速度，更新到全局变量
    uvr_solve();

    // EKF 融合：使用差分速度作为测量，使用当前 yaw rate 做预测
    if(ekf) {
        ekf->predict(Ts, Rov_psi_diff); // 预测
        ekf->updateGPS(Rov_x, Rov_y);   // GPS位置
        ekf->updateYaw(Rov_psi * M_PI/180.0); // 航向 (rad)
        ekf->updateVel(Rov_u, Rov_v);   // 速度测量
        const EKFState &st = ekf->state();
        Rov_x = st.x;
        Rov_y = st.y;
        Rov_u = st.u;
        Rov_v = st.v;
        // 航向保持原 UI 度制显示，不覆盖 Rov_psi (保持传感器角度)，可选：Rov_psi = st.psi*180.0/M_PI;
    }

    //gps模式
    quint8 mode = data[29];
    display_gps_mode(mode);
    //gps卫星数量
    quint8 count = data[30];
    display_gps_satellite_count(count);
    //gps水平经度因子
    quint8 factor = data[31];
    display_gps_longitude_factor(factor);

    // 通知servewidget进程，可以发送状态量给matlab
    matlab_send_flag = 1;
}

//计算差分速度u,r
void MainWindow::uvr_solve()
{
    QMutexLocker locker(&myMutex_xy);// 与 map_set 互斥
    Rov_x = Rov_location[0][0];
    Rov_y = Rov_location[0][1] / 10.0; // 原始代码做过 /10 处理
    v_x = (Rov_location[0][0] - Rov_location[1][0]) / Ts; // ENU坐标 x方向速度
    v_y = (Rov_location[0][1] - Rov_location[1][1]) / (10.0 * Ts); // ENU坐标 y方向速度
    // 使用中点航向降低瞬时旋转误差
    double psi_mid_rad = (Rov_psi + Rov_psi_last) * 0.5 * M_PI / 180.0;
    // 速度坐标变换：全局(ENU) -> 机器人坐标系 (surge u, sway v)
    Rov_u = v_x * cos(psi_mid_rad) + v_y * sin(psi_mid_rad);
    Rov_v = -v_x * sin(psi_mid_rad) + v_y * cos(psi_mid_rad);
    Rov_r = Rov_psi_diff; // yaw rate (rad/s) 已由滤波更新
}
//姿态角
void MainWindow::display_angle(quint16 yaw,quint16 roll,quint16 pitch)
{
    yaw = hextoInt(yaw);//去掉符号位，返回绝对值数据
    QString yaw_num = QString::number(yaw,10);//HEX数转10进制字符串
    if(angle_signed != 0)//hextoInt内识别为负数
        yaw_signed = 1;
    roll = hextoInt(roll);
    QString roll_num = QString::number(roll,10);
    if(angle_signed != 0)
        roll_signed = 1;
    pitch = hextoInt(pitch);
    QString pitch_num = QString::number(pitch,10);
    if(angle_signed != 0)
        pitch_signed = 1;
    float yaw_dec = yaw_num.toFloat();//字符串转浮点数，用来计算
    float roll_dec = roll_num.toFloat();
    float pitch_dec = pitch_num.toFloat();
    yaw_dec /= 10;//计算真实值
    roll_dec /= 10;
    pitch_dec /= 10;
    //转为 x轴正半轴开始逆时针(0,180), x负半轴开始逆时针(-180,0).
    //跳变点为x负半轴，对于正西方向。实验中尽量避免朝这个方向前进
    float psi = 0;
    if (yaw_dec> 270 && yaw_dec<= 360)
        psi = 450 - yaw_dec;
    else if(yaw_dec>= 0 && yaw_dec<= 270)
        psi = 90 - yaw_dec;
//    //滤波参数
//    float filter_sum = 0;
//    int sum_coe = 0; // 加权系数和
//    int k = 3;//滑动窗口，不大于 FILTER_N=12 即可
//    filter_buf[k] = psi;//角度放入滤波队列
//    for(int i = 0; i < k; i++)
//    {
//        filter_buf[i] = filter_buf[i + 1];//所有数据左移，低位仍掉
//        filter_sum += filter_buf[i] * coe[i];//加权滑动滤波
//        sum_coe = sum_coe + i + 1;
//        //  filter_sum += filter_buf[i] * 1;//滑动滤波
//        //  sum_coe = sum_coe + i + 1;
//    }
//    //判断是否越过X-轴
//    if(psi<170 && psi>-170)
//    {
//        //Rov_psi = filter_sum/sum_coe;//加权滑动滤波
//        Rov_psi = filter_sum/k;//均值滤波
//    }
//    else
//    {
//        Rov_psi = psi;//无滤波
//        qDebug()<<"=================================";
//    }
    //更新上一步
    Rov_psi_last = Rov_psi;
    Rov_psi_diff_last = Rov_psi_diff;
    //更新角度，无加权滤波
    Rov_psi = psi;
    //更新角速度
    psi_diff_solve();

    yaw_num = QString::number(Rov_psi);//浮点数转回字符串
    roll_num = QString::number(roll_dec);
    pitch_num = QString::number(pitch_dec);
    if(yaw_signed != 0)
        ui->lineEdit_yaw->setText("-"+yaw_num+u8"°");//负数
    else
        ui->lineEdit_yaw->setText(yaw_num+u8"°");//正数
    if(roll_signed != 0)
        ui->lineEdit_roll->setText("-"+roll_num+u8"°");
    else
        ui->lineEdit_roll->setText(roll_num+u8"°");
    if(pitch_signed != 0)
        ui->lineEdit_pitch->setText("-"+pitch_num+u8"°");
    else
        ui->lineEdit_pitch->setText(pitch_num+u8"°");
}

//角速度r更新
void MainWindow::psi_diff_solve()
{
    // Kalman 滤波：状态 x=[psi_rad, r_rad]^T，测量 z=psi_rad (展开后的航向弧度)
    static bool kf_inited = false;
    static double kf_psi = 0.0; // 航向弧度
    static double kf_r   = 0.0; // 航向角速度 rad/s
    static double P11 = 1.0, P12 = 0.0, P21 = 0.0, P22 = 1.0; // 协方差矩阵
    static const double q_psi = 1e-5;      // 航向过程噪声
    static const double q_r   = 5e-4;      // 角速度过程噪声
    static const double r_meas = (0.5 * M_PI/180.0) * (0.5 * M_PI/180.0); // 测量噪声方差 (0.5°)
    // 角度展开避免跨±180°跳变
    static double unwrap_last_deg = 0.0;
    static double unwrap_accum_deg = 0.0;
    if(!kf_inited) {
        unwrap_last_deg = Rov_psi;
        unwrap_accum_deg = Rov_psi;
        kf_psi = Rov_psi * M_PI/180.0;
        kf_r = 0.0;
        kf_inited = true;
        Rov_psi_diff_last = 0.0;
        Rov_psi_diff = 0.0;
        Rov_psi_diff_2 = 0.0;
        return;
    }
    double raw_deg = Rov_psi;
    double delta_deg = raw_deg - unwrap_last_deg;
    while(delta_deg > 180.0) delta_deg -= 360.0;
    while(delta_deg < -180.0) delta_deg += 360.0;
    unwrap_accum_deg += delta_deg;
    unwrap_last_deg = raw_deg;
    double z_rad = unwrap_accum_deg * M_PI/180.0;
    double dt = Ts;
    // 预测
    double psi_pred = kf_psi + kf_r * dt;
    double r_pred   = kf_r;
    double P11_pred = P11 + dt*(P21 + P12) + dt*dt*P22 + q_psi;
    double P12_pred = P12 + dt*P22;
    double P21_pred = P21 + dt*P22;
    double P22_pred = P22 + q_r;
    // 更新
    double y = z_rad - psi_pred;
    double S = P11_pred + r_meas;
    double K1 = P11_pred / S;
    double K2 = P21_pred / S;
    kf_psi = psi_pred + K1 * y;
    kf_r   = r_pred   + K2 * y;
    P11 = (1.0 - K1) * P11_pred;
    P12 = (1.0 - K1) * P12_pred;
    P21 = P21_pred - K2 * P11_pred;
    P22 = P22_pred - K2 * P12_pred;
    // 输出角速度及二阶导
    Rov_psi_diff_last = Rov_psi_diff;
    Rov_psi_diff = kf_r;
    Rov_psi_diff_2 = (Rov_psi_diff - Rov_psi_diff_last) / dt;
}
//距离传感器
void MainWindow::display_distance(quint8 front,quint8 after,quint8 left,quint8 right)
{
    QString front_num = QString::number(front,10);//HEX数转10进制字符串
    QString after_num = QString::number(after,10);
    QString left_num = QString::number(left,10);
    QString right_num = QString::number(right,10);
    float front_dec = front_num.toFloat();//字符串转浮点数，用来计算
    float after_dec = after_num.toFloat();
    float left_dec = left_num.toFloat();
    float right_dec = right_num.toFloat();
    front_dec /= 10;//计算真实值
    after_dec /= 10;
    left_dec /= 10;
    right_dec /= 10;
    distance[0] = front_dec;
    distance[1] = after_dec;
    distance[2] = left_dec;
    distance[3] = right_dec;
    front_num = QString::number(front_dec);//浮点数转回字符串
    after_num = QString::number(after_dec);
    left_num = QString::number(left_dec);
    right_num = QString::number(right_dec);
    ui->lineEdit_front->setText(front_num+u8"米");//UI显示
    ui->lineEdit_after->setText(after_num+u8"米");
    ui->lineEdit_left->setText(left_num+u8"米");
    ui->lineEdit_right->setText(right_num+u8"米");
}
//云台
void MainWindow::display_PTZ(quint8 doub,quint8 single)
{
    QString doub_num = QString::number(doub,10);//HEX数转10进制字符串
    QString single_num = QString::number(single,10);
    float doub_dec = doub_num.toFloat();//字符串转浮点数，用来计算
    float single_dec = single_num.toFloat();
//    doub_dec = doub_dec*180/255;//存在比例计算时选择这种
//    single_dec = single_dec*180/255;
    doub_dec = doub_dec*1;//计算真实值
    single_dec = single_dec*1;
    doub_num = QString::number(doub_dec);//浮点数转回字符串
    single_num = QString::number(single_dec);
    ui->lineEdit_doublePTZ->setText(doub_num+u8"°");//UI显示
    ui->lineEdit_singlePTZ->setText(single_num+u8"°");
}
//挡位
void MainWindow::display_handle(quint8 handle)
{
    QString handle_num = QString::number(handle,10);//HEX数转10进制字符串
    float handle_dec = handle_num.toFloat();//字符串转浮点数，用来计算
    handle_dec = handle_dec*1;//计算真实值
    handle_num = QString::number(handle_dec);//浮点数转回字符串
    ui->lineEdit_motor->setText(handle_num+u8" 挡");//UI显示
}
//gps时间
void MainWindow::display_gps_time(quint8 hour,quint8 min,quint8 sec)
{
    QString hour_num = QString::number(hour,10);//HEX数转10进制字符串
    QString min_num = QString::number(min,10);
    QString sec_num = QString::number(sec,10);
    float hour_dec = hour_num.toFloat();//字符串转浮点数，用来计算
    float min_dec = min_num.toFloat();
    float sec_dec = sec_num.toFloat();
    hour_dec = hour_dec*1 + 8;//计算真实值 UTC时间转北京时间
    min_dec = min_dec*1;
    sec_dec = sec_dec*1;
    hour_num = QString::number(hour_dec);//浮点数转回字符串
    min_num = QString::number(min_dec);
    sec_num = QString::number(sec_dec);
    ui->lineEdit_GPS_time->setText(hour_num+u8":"+min_num+u8":"+sec_num);//UI显示
}
//gps纬度
void MainWindow::display_gps_latitude(quint16 integer,int decimal,quint16 n_s,float lat_m)
{
    // lat_m 是十进制浮点数，代表纬度的小数，由GPS原始数据得出
    QString integer_num = QString::number(integer,10);//HEX数转10进制字符串
    QString decimal_num = QString::number(decimal,10);
    QString n_s_num = QString::number(n_s,10);
    float integer_dec = integer_num.toFloat();//字符串转浮点数，用来计算
    float decimal_dec = decimal_num.toFloat();
    float n_s_dec = n_s_num.toFloat();
    integer_dec = integer_dec*1;//计算真实值
    decimal_dec = decimal_dec*1;
    n_s_dec = n_s_dec*1;
    integer_num = QString::number(integer_dec);//浮点数转回字符串
    decimal_num = QString::number(decimal_dec,'f',0);//非科学计数显示，保留0位小数
    QString lat_m_num = QString::number(lat_m,'f',0);//保留0位小数
    while(decimal_num.length()<8)//末位补0，直到长8位
        decimal_num += "0";
    decimal_num = decimal_num.left(8);

    //qDebug()<<integer_num<<" "<<decimal_num;

    if(n_s_dec)
        n_s_num = 'S';//1南纬
    else
        n_s_num = 'N';//0北纬
    //ui->lineEdit_latitude->setText(integer_num+"."+decimal_num);//UI显示
    ui->lineEdit_latitude->setText(integer_num+"."+lat_m_num);//原始数据显示
    rov_lat = integer_num + "." + lat_m_num;
    //rov_lat = "39.99337600";
}
//gps经度
void MainWindow::display_gps_longitude(quint16 integer,int decimal,quint16 e_w,float lon_m)
{
    QString integer_num = QString::number(integer,10);//HEX数转10进制字符串
    QString decimal_num = QString::number(decimal,10);
    QString e_w_num = QString::number(e_w,10);
    float integer_dec = integer_num.toFloat();//字符串转浮点数，用来计算
    float decimal_dec = decimal_num.toFloat();
    float e_w_dec = e_w_num.toFloat();
    integer_dec = integer_dec*1;//计算真实值
    decimal_dec = decimal_dec*1;
    e_w_dec = e_w_dec*1;
    integer_num = QString::number(integer_dec);//浮点数转回字符串
    decimal_num = QString::number(decimal_dec,'f',0);//非科学计数显示，保留0位小数
    QString lon_m_num = QString::number(lon_m,'f',0);//保留0位小数
    while(decimal_num.length()<8)//末位补0，直到长8位
        decimal_num += "0";
    if(e_w_dec)
        e_w_num = 'E';//1东经
    else
        e_w_num = 'W';//0西经
    //ui->lineEdit_longitude->setText(integer_num+"."+decimal_num);//UI显示
    ui->lineEdit_longitude->setText(integer_num+"."+lon_m_num);//原始数据显示
    rov_lon = integer_num + "." + lon_m_num;
    //rov_lon = "116.30881700";
}
//gps模式
void MainWindow::display_gps_mode(quint16 mode)
{
    QString mode_num = QString::number(mode,10);//HEX数转10进制字符串
    float mode_dec = mode_num.toFloat();//字符串转浮点数，用来计算
    mode_dec = mode_dec*1;//计算真实值
    if(mode_dec == 0)//浮点数转回字符串
        mode_num = "定位数据不可用!";
    else if(mode_dec == 1)
        mode_num = "SPS模式,定位有效";
    else if(mode_dec == 2)
        mode_num = "差分GPS模式,定位有效";
    else if(mode_dec == 3)
        mode_num = "SPS模式定位有效";//通信协议未给0x03
    else if(mode_dec == 4)
        mode_num = "RTK固定解";
    else if(mode_dec == 5)
        mode_num = "RTK浮点解";
    ui->lineEdit_GPS_mode->setText(mode_num);//UI显示
}
//gps卫星数量
void MainWindow::display_gps_satellite_count(quint16 count)
{
    QString count_num = QString::number(count,10);//HEX数转10进制字符串
    float count_dec = count_num.toFloat();//字符串转浮点数，用来计算
    count_dec = count_dec*1;//计算真实值
    count_num = QString::number(count_dec);//浮点数转回字符串
    ui->lineEdit_satellite_count->setText(count_num+u8"个");//UI显示
}
//gps经度因子
void MainWindow::display_gps_longitude_factor(quint16 factor)
{
    QString factor_num = QString::number(factor,10);//HEX数转10进制字符串
    float factor_dec = factor_num.toFloat();//字符串转浮点数，用来计算
    factor_dec = factor_dec/100;//计算真实值
    factor_num = QString::number(factor_dec);//浮点数转回字符串
    ui->lineEdit_longitude_factor->setText(factor_num);//UI显示
}
//去掉负数的符号位
qint16 MainWindow::hextoInt(quint16 angle)
{
    angle_signed = 0;//复位

    QString angle_str = QString::number(angle,16);
    //qDebug()<<"yaw_str.length()="<<QString().asprintf("%d",yaw_str.length());
    if(angle_str.length()==4)//QString字符串默认高位空，字符串长度为4时才可能为负
    {
        if(!(angle_str.at(0)>='0'&&angle_str.at(0)<='7'))//第一位1表示负数
        {
            angle = angle & 0x7fff;//最高位置0
            angle_signed =1;//通知外层函数，本次结果为负数
//            angle = ~angle;   //若数据是补码形式，则需要以下几行转原码
//            angle = angle & 0x7fff;
//            angle = angle + 1;
//            angle = angle * -1;
        }
    }
    return angle;
}
//生成CRC16位校验码
quint16 MainWindow::crc16(const quint8*data, quint16 len)
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


/*
 * *****************************************************************************************
  # map_set
 * *****************************************************************************************
*/

void MainWindow::on_button_map_set_clicked()
{
    //map_set* configWindow = new map_set;
    configWindow->show();
    configWindow->setWindowTitle("Map_Config");
}

//显示rov 坐标
void MainWindow::doProcessRov_location(double e,double n)
{
    QString x = QString::number(e,'f',2);
    ui->lineEdit_rov_e->setText(x);
    QString y = QString::number(n/10,'f',2);
    ui->lineEdit_rov_n->setText(y);
}


//按下“绘制曲线”按钮。
void MainWindow::on_button_handle_clicked()
{
    if(!plotWin) {
        plotWin = new PlotWindow(this, ekf);
    }
    // 引入正弦参考更新函数声明
    //（头文件若未包含 tracking_params.h 则添加）
    plotWin->show();
    if(!plotStarted) {
        plotElapsed.start();
        plotTimer = new QTimer(this);
        connect(plotTimer, &QTimer::timeout, this, [this]() {
            double t_sec = plotElapsed.elapsed()/1000.0;
            // Rov_psi 当前为度，转换为弧度
            double psi_rad = Rov_psi * M_PI / 180.0;
            // Rov_psi_diff 已按代码逻辑为 rad/s；若未来改为度/s，这里需再乘 M_PI/180
            double r_rad = Rov_psi_diff;
            // 确保参考正弦在未启动控制线程时也随时间更新
            update_tracking_refs(plotTimer->interval()/1000.0);
            plotWin->addSample(t_sec, psi_rad, r_rad, Rov_x, Rov_y, Rov_u, Rov_v);
        });
        plotTimer->start(100); // 100 ms 更新
        plotStarted = true;
    }
}


