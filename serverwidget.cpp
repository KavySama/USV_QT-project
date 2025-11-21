#include "serverwidget.h"
#include "ui_serverwidget.h"
#include "stdio.h"
#include "mainwindow.h"

quint8 rsv_data[18];//接收待处理数据。用QByteArray易错！！
int clear_send = 0;//发送计数器
bool ok2;
float mpc_u; // MPC回传的最优速度
float mpc_r;
float mpc_tau_u; // MPC回传的推力（由预设模型计算出）
float mpc_tau_r;
float psi = 0;
extern float Rov_x;
extern float Rov_y;
extern float Rov_psi;
extern float Rov_u;
extern float Rov_r;
extern float Ts;
extern float matlab_send_flag;
extern float matlab_rsv_flag;

serverwidget::serverwidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::serverwidget)
{
    ui->setupUi(this);
    timer_matlab = new QTimer(this);//matlab定时器初始化

    tcpserver = nullptr;
    tcpsocket = nullptr;
    //创建监听套接字对象
    tcpserver=new QTcpServer(this);//指定父对象 便于回收空间
    //bind+listen
    tcpserver->listen(QHostAddress::Any,8080);//设置服务器地址和端口号，并监听

    //服务器建立连接
    connect(tcpserver,&QTcpServer::newConnection,[=]()
    {
        tcpsocket = tcpserver->nextPendingConnection();//取出连接好的套接字
        TcpSockets_List.append(tcpsocket);//把新进来的客户端存入列表
        Client_num++;

        QString ip = tcpsocket->peerAddress().toString();// 获取连接的 ip地址
        quint16 port = tcpsocket->peerPort();// 获取连接的 端口号
        QString temp = QString("[%1:%2] 客户端%3连接成功").arg(ip).arg(port).arg(Client_num);
        ui->textEdit_Read->append(temp);// 显示客户端信息
        ui->lineEdit_client_count->setText(QString::number(Client_num));// 显示连接数量
        //qDebug()<<"Client_num="<<QString().asprintf("%d",Client_num);

        connect(timer_matlab,&QTimer::timeout, [=]()
        {
            // 先暂停定时器
            timer_matlab->stop();
            //等mainwindow处理完传感器数据，再将数据发送给matlab
            if(matlab_send_flag == 1)
            {
                // 将ROV状态帧传给MATLAB
                quint8 data[19];
                data[0] = 0x01;
                data[1] = 0x02;
                data[2] = sign(Rov_x);//符号位,0正，1负
                data[3] = floor(abs(Rov_x)); //坐标x的整数部分, (x,y)已限制在[-200,200]方形范围内
                data[4] = floor((abs(Rov_x) - data[3])*100);//取两位小数
                data[5] = sign(Rov_y);//符号位,0正，1负
                data[6] = floor(abs(Rov_y)); //坐标y的整数部分
                data[7] = floor((abs(Rov_y) - data[6])*100);//取两位小数
                psi = Rov_psi * M_PI/180; // 转为弧度
                data[8] = sign(psi);//符号位,0正，1负
                data[9] = floor(abs(psi)); // 角度psi的整数部分, 逆时针 上半部 0~pi 下半部 -pi~0
                data[10] = floor((abs(psi) - data[9])*100);// 取两位小数
                data[11] = sign(Rov_u);//符号位,0正，1负
                data[12] = floor(abs(Rov_u)); // 速度u的整数部分
                data[13] = floor((abs(Rov_u) - data[12])*100);// 取两位小数
                data[14] = sign(Rov_r);//符号位,0正，1负
                data[15] = floor(abs(Rov_r)); // 速度r的整数部分
                data[16] = floor((abs(Rov_r) - data[15])*100);//取两位小数
                data[17] = 0x03;
                data[18] = 0x04;
                QByteArray data_send;//发送缓冲区
                data_send.resize(19);
                for(int j = 0;j < 19;j ++)//转成QByteArray数组，方便发送
                {
                    data_send[j] = data[j];
                }
                //qDebug()<<"Send to Matlab: "<<data_send;
                tcpsocket->write(data_send);

                matlab_send_flag = 0;//清空同步标记
            }

            timer_matlab->start(10);//重启定时器,每10ms尝试向MATLAB发送一次ROV状态
        });

        //接收信息
        connect(tcpsocket,&QTcpSocket::readyRead,[=]()
        {
            //接受MATLAB回传帧
            QByteArray buffer;
            buffer.resize(17);
            buffer = (QByteArray)tcpsocket->read(17);
            //hexstring
            QString hexString = buffer.toHex();
            for(int i = 0;i < 17;i ++)
                rsv_data[i] = hexString.mid(i*2,2).toInt(&ok2,16); // 依次取hexString两个字符拼成一个整型变量(16进制表示)
            //qDebug()<<rsv_data[0]<<" "<<rsv_data[1]<<" "<<rsv_data[2]<<" "<<rsv_data[3]<<" "<<rsv_data[4]<<" "<<rsv_data[5]<<" "<<rsv_data[6]<<" "<<rsv_data[7];
            if((rsv_data[0]==0x01)&&(rsv_data[1]==0x02)&&(rsv_data[15]==0x03)&&(rsv_data[16]==0x04)) // 帧头和帧尾吻合, 是否再添加一个顺序标志位？增加同步性
            {
                //ui->textEdit_Read->append("【接收Matlab帧：】");
                //ui->textEdit_Read->append(hexString);
                mpc_u = rsv_data[2] + float(rsv_data[3])/100;
                if(rsv_data[4] == 0)
                    {mpc_r = rsv_data[5] + float(rsv_data[6])/100;}
                else
                    {mpc_r = -rsv_data[5] - float(rsv_data[6])/100;}
                if(rsv_data[7] == 0)
                {mpc_tau_u = rsv_data[8] + float(rsv_data[9])/100 + float(rsv_data[10])/10000;}
                else
                {mpc_tau_u = -rsv_data[8] - float(rsv_data[9])/100 - float(rsv_data[10])/10000;}
                if(rsv_data[11] == 0)
                {mpc_tau_r = rsv_data[12] + float(rsv_data[13])/100 + float(rsv_data[14])/10000;}
                else
                {mpc_tau_r = -rsv_data[12] - float(rsv_data[13])/100 - float(rsv_data[14])/10000;}
                qDebug() << "Recevied from Matlab: "<< "mpc_u = " << mpc_u << " mpc_r = " << mpc_r<< " mpc_tau_u = " << mpc_tau_u << " mpc_tau_r = " << mpc_tau_r;

                matlab_rsv_flag = 1; // 提示control_thread接收完毕
            }
            else
                return; // 丢弃
            //接受图像处理帧
            //            QByteArray buffer = (QByteArray)tcpsocket->read(18);
            //            //hexstring
            //            QString hexString = buffer.toHex();
            //            //qDebug()<<hexString;
            //            for(int i = 0;i < 18;i ++)
            //                rsv_data[i] = hexString.mid(i*2,2).toInt(&ok2,16);//依次取hexString两个字符拼成一个整型变量(16进制表示)
            //            if((rsv_data[0]==0x01)&&(rsv_data[1]==0x02)&&(rsv_data[16]==0x03)&&(rsv_data[17]==0x04))//帧和帧尾吻合
            //            {
            //                ui->textEdit_Read->append("【客户端发送】");
            //                ui->textEdit_Read->append(hexString);
            //            }
            //            else
            //                return;//丢弃
        });

        //连接断开时
        connect(tcpsocket,&QTcpSocket::disconnected,[this]()
        {
            Client_num = 0;//客户端连接数量置0
            clear_send = 0;//发送计数置0
            tcpsocket->close();//关闭 通信套接字
            tcpsocket=nullptr;//赋空指针，等待下次建立通信 
            ui->textEdit_Read->setText("与客户端连接已断开！");
            ui->textEdit_Write->setText("");//清空发送区
            ui->lineEdit_client_count->setText(QString::number(Client_num));//刷新连接数量
        });

    });

}

serverwidget::~serverwidget()
{
    delete ui;

    delete timer_matlab;//删除matlab定时器
}

/*
 ******************************************************************************************
  @ 模拟下位机发送的包
 ******************************************************************************************
*/

//发送数据包随机生成函数,传递指针
void serverwidget::Create_array(quint8* data){
    int i = 0;

    data[0] = 0xeb;//固定帧头
    data[1] = 0x90;
    data[2] = 0x24;//数据长度36

    //姿态角  0x0000-0x0708  0-1800
    data[3] = rand_hex(7);
    if(data[3]==7)
        data[4] = rand_hex(8);
    else
        data[4] = rand_hex(255);
    data[5] = rand_hex(7);
    if(data[5]==7)
        data[6] = rand_hex(8);
    else
        data[6] = rand_hex(255);
    data[7] = rand_hex(7);
    if(data[7]==7)
        data[8] = rand_hex(8);
    else
        data[8] = rand_hex(255);
    //距离传感器  0x00-0x78
    for(i=9;i<=12;i++)
    {
        data[i] = rand_hex(120);
    }
    //云台角度  0x00-0xff/b4 0-180
    for(i=13;i<=14;i++)
    {
        data[i] = rand_hex(180);
    }
    //挡位  0x01-0x03
    data[15] = rand_hex(3);
    //GPS  时分秒
    data[16] = 14;
    data[17] = 15;
    data[18] = 12;
    //纬度
//    data[19] = rand_hex(90);//整数部分，0-90度
//    data[20] = rand_hex(14);//00-0e,小数部分0-999999对应0x00-0x0f423f
//    data[21] = rand_hex(255);//实际模拟要加多个if分类赋值，这里仅简便设置0x00-0x0effff
//    data[22] = rand_hex(255);//
//    data[23] = rand_hex(1);//南北纬
    data[19] = 39;//整数部分，0-90度
    data[20] = 0xff;//00-0e,小数部分0-999999对应0x00-0x0f423f
    data[21] = 0x29;//实际模拟要加多个if分类赋值，这里仅简便设置0x00-0x0effff
    data[22] = 0x01;//
    data[23] = 0x00;//北纬
    //经度
//    data[24] = rand_hex(180);//整数部分，0-180度
//    data[25] = rand_hex(14);//00-0e,小数部分0-999999对应0x00-0x0f423f
//    data[26] = rand_hex(255);//实际模拟要加多个if分类赋值，这里仅简便设置0x00-0x0effff
//    data[27] = rand_hex(255);//
//    data[28] = rand_hex(1);//东西经
    data[24] = 116;//整数部分，0-180度
    data[25] = 0xff;//00-0e,小数部分0-999999对应0x00-0x0f423f
    data[26] = 0xac;//实际模拟要加多个if分类赋值，这里仅简便设置0x00-0x0effff
    data[27] = 0x43;//
    data[28] = 0x01;//东经
    //GPS模式指示  0x00-0x05
    data[29] = rand_hex(5);
    //卫星数量和水平经度因子  0x00-0xff
    for(i=30;i<=31;i++)
    {
        data[i] = rand_hex(255);
    }
    //CRC校验码  是否需要校验帧头数据待定！！！！
    quint16 crc16_out = crc16(data,32);//生成0-31数据字节的校验码,2字节
    data[32] = (quint8)((0xff00&crc16_out)>>8);//高8位
    data[33] = (quint8)(0x00ff&crc16_out);//低8位

    data[34] = 0x0d;//固定帧尾
    data[35] = 0x0a;
}
//生成CRC16位校验码
quint16 serverwidget::crc16(const quint8*data, quint16 len)
{
    quint16 crc = 0xFFFF;
    for (quint8 i = 0; i < len; i++)//若只校验数据位，则从i=3开始
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
//生成随机16进制数
quint8 serverwidget::rand_hex(quint8 max)
{
    QRandomGenerator generator = QRandomGenerator::securelySeeded();// 生成随机数种子，可以使用当前时间作为种子
    quint8 random_dec = generator.bounded(max+1);//生成0-max范围的随机数
    //将随机数转为16进制
    QString num;
    num = QString::number(random_dec,16);
    quint8 a= num.toInt();
    //qDebug()<<"send_data="<<QString().asprintf("%x",a);
    return a;
}
//延时功能 非阻塞型
void serverwidget::sleep(unsigned int msec)
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
 ******************************************************************************************
  @ 服务端
 ******************************************************************************************
*/

//发送传感器数据
void serverwidget::on_button_send_clicked()
{
    if(tcpsocket==nullptr)//未建立连接，则不发送
    {
        return ;
    }

    timer_matlab->start(100);//开启matlab定时器

    //qDebug()<<data_send;
//    while(tcpsocket!=nullptr)//建立连接期间持续发送
//    {
//        Create_array(data);//对data进行一次随机生成，包括校验码，帧头帧尾不变
//        for(int j = 0;j < 6;j ++)//转成QByteArray数组，方便发送
//        {
//            data_send[j] = data[j];
//        }
//        clear_send++;
//        if(clear_send % 50 == 0)//每发送50条信息清空一次窗口
//        {
//            ui->textEdit_Read->setText("");
//        }
//        sleep(100);//每0.1s发送一次
//    }

}

//清空显示区
void serverwidget::on_button_clear_clicked()
{
    ui->textEdit_Read->setText("");
}

//清空发送区
void serverwidget::on_button_clear_send_clicked()
{
    ui->textEdit_Write->setText("");
}

//主动断开TCP连接
void serverwidget::on_button_exit_clicked()
{
    timer_matlab->stop();
    tcpsocket->close();//关闭 通信套接字
    tcpsocket=nullptr;//赋空指针，等待下次建立通信
}

//符号函数
float serverwidget::sign(double S)
{
    float s = 0;
    if (S >= 0)
        s = 0;
    else if (S < 0)
        s = 1;
    return s;
}
