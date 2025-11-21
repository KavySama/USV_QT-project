#ifndef SERVERWIDGET_H
#define SERVERWIDGET_H

#include <QWidget>
#include<QTcpServer>//监听套接字
#include<QTcpSocket>//通信套接字
#include <QMessageBox>
#include <QDebug>
#include <QRandomGenerator>
#include <QTime>


namespace Ui {
class serverwidget;
}

class serverwidget : public QWidget
{
    Q_OBJECT

public:
    explicit serverwidget(QWidget *parent = nullptr);
    ~serverwidget();

private slots:
//发送数据
    void sleep(unsigned int msec);
    void Create_array(quint8* data);
    quint8 rand_hex(quint8 max);
    quint16 crc16(const quint8*data, quint16 len);//生成CRC16位校验码
//TCP服务端按钮
    void on_button_send_clicked();
    void on_button_clear_clicked();
    void on_button_clear_send_clicked();
    void on_button_exit_clicked();

private:
    Ui::serverwidget *ui;
    QTcpServer *tcpserver;//监听套接字
    QTcpSocket *tcpsocket;//通信套接字
    QList<QTcpSocket*> TcpSockets_List;//soket列表
    int Client_num = 0;//客户端连接数量

    QTimer *timer_matlab;//matlab通信定时器，用于传输数据
    float sign(double S);
};

#endif // SERVERWIDGET_H
