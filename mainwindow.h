#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "tcp_thread.h"
#include "control_thread.h"
#include "map_set.h"
#include "serverwidget.h"


//需要添加的库
#include <QProcess>
#include <QMessageBox>
#include <QTcpSocket>//通信套接字
#include <QHostAddress>
#include <QDebug>
#include <QString>
#include <QRandomGenerator>
#include <QTime>
#include <QWebEngineView>
#include <QStackedLayout>
#include <QKeyEvent>
#include <QSemaphore>
#include <QThread>
#include <QTimer>
#include <QElapsedTimer>
#include "plotwindow.h"
#include "ekf_localization.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
protected:
    void keyPressEvent(QKeyEvent *event)override; //重写键盘按下事件
    void keyReleaseEvent(QKeyEvent *event)override; //重写键盘松开事件
signals:
    void SigToConnect(QString,QString);
    void SigToSend(QString);
    void SigToClose();
    void SigToManual();
    void SigToAuto();
    void keyPressed(int key);//键盘信号
    void keyReleased(int key);
    void rov_ll(QString,QString);//to map_set
public slots:

private slots:
    //tcp线程回传处理
    void doProcessConnected();
    void doProcessDisconnected();
    void doProcessShow();
    //control线程回传处理
    void doProcessMove_Forward();
    void doProcessMove_Backward();
    void doProcessMove_Left();
    void doProcessMove_Right();
    void doProcessMove_TurnLeft();
    void doProcessMove_TurnRight();
    void doProcessPtz1_TurnLeft();
    void doProcessPtz1_TurnRight();
    void doProcessPtz2_TurnLeft();
    void doProcessPtz2_TurnRight();
    void doProcessMotor_Run();
    void doProcessMotor_Stop();
    void doProcessReset();
    //ROV客户端按钮
    void on_button_connect_clicked();
    void on_button_send_clicked();
    void on_button_clear_clicked();
    void on_button_clear_send_clicked();
    //双目摄像头客户端按钮
    void on_button_connect_2_clicked();
    void on_button_close_2_clicked();
    //控制按钮
    void on_button_handle_clicked();
    //map_set
    void on_button_map_set_clicked();
    void doProcessRov_location(double e,double n);//map_set

private:
    Ui::MainWindow *ui;
    QTcpSocket *tcpsocket;//ROV通信套接字
    QWebEngineView *m_webView; //页面显示对象指针
    QString serverIP;
    QString serverPort;
    QString str0;

    QThread tcp_thread;//tcp线程类
    QThread control_thread;//控制线程类

    map_set *configWindow;//map_set

    void thread_init();
    void connect_init();
    //void startObjthread();
    void sleep(unsigned int msec);
    //接收数据处理
    void display_ROV(quint8* data);//ROV回传数据显示
    void display_angle(quint16 yaw,quint16 roll,quint16 pitch);
    void display_distance(quint8 front,quint8 after,quint8 left,quint8 right);
    void display_PTZ(quint8 doub,quint8 single);
    void display_handle(quint8 handle);
    void display_gps_time(quint8 hour,quint8 min,quint8 sec);
    void display_gps_latitude(quint16 integer,int decimal,quint16 n_s,float lat_m);
    void display_gps_longitude(quint16 integer,int decimal,quint16 e_w,float lon_m);
    void display_gps_mode(quint16 mode);
    void display_gps_satellite_count(quint16 count);
    void display_gps_longitude_factor(quint16 factor);
    qint16 hextoInt(quint16 s);//转为有符号数
    quint16 crc16(const quint8*data, quint16 len);//生成CRC16位校验码

    void psi_diff_solve();
    void uvr_solve();

    // 绘图相关
    PlotWindow *plotWin = nullptr;
    QTimer *plotTimer = nullptr;
    QElapsedTimer plotElapsed;
    bool plotStarted = false;
    EKFLocalization *ekf = nullptr; // 定位与速度 EKF
};
#endif // MAINWINDOW_H
