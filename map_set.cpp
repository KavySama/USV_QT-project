#include "map_set.h"
#include "ui_map_set.h"
#include "mainwindow.h"

quint8 flag_save = 0;
int R = 6371393;//m
double lat0 = 0;
double lon0 = 0;
QSemaphore Sem_2(1);
QMutex myMutex_xy;
QVector<QVector<double>> Rov_location;
QVector<QVector<double>> P_location;
QVector<double> xy_mid;

map_set::map_set(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::map_set)
{
    ui->setupUi(this);

    //Initialize 2D QVector
    Rov_location.resize(3);//3 2
    for(int i = 0;i < Rov_location.size();i++)
    {
        Rov_location[i].resize(2);
        for(int j = 0;j < Rov_location[i].size();j ++)
        {
            Rov_location[i][j] = 0;
        }
    }
    P_location.resize(11);//11 2
    for(int i = 0;i < P_location.size();i++)
    {
        P_location[i].resize(2);
        for(int j = 0;j < P_location[i].size();j ++)
        {
            P_location[i][j] = 0;
        }
    }
}

map_set::~map_set()
{
    delete ui;
}

void map_set::on_button_save_clicked()
{
    flag_save = 1;
    //read data
    QString b_lat = ui->lineEdit_base_lat->text();
    QString b_lon = ui->lineEdit_base_lon->text();
    QString p1_lat = ui->lineEdit_p1_lat->text();
    QString p1_lon = ui->lineEdit_p1_lon->text();
    QString p2_lat = ui->lineEdit_p2_lat->text();
    QString p2_lon = ui->lineEdit_p2_lon->text();
    QString p3_lat = ui->lineEdit_p3_lat->text();
    QString p3_lon = ui->lineEdit_p3_lon->text();
    QString p4_lat = ui->lineEdit_p4_lat->text();
    QString p4_lon = ui->lineEdit_p4_lon->text();
    QString p5_lat = ui->lineEdit_p5_lat->text();
    QString p5_lon = ui->lineEdit_p5_lon->text();
    QString p6_lat = ui->lineEdit_p6_lat->text();
    QString p6_lon = ui->lineEdit_p6_lon->text();
    QString p7_lat = ui->lineEdit_p7_lat->text();
    QString p7_lon = ui->lineEdit_p7_lon->text();
    QString p8_lat = ui->lineEdit_p8_lat->text();
    QString p8_lon = ui->lineEdit_p8_lon->text();
    QString p9_lat = ui->lineEdit_p9_lat->text();
    QString p9_lon = ui->lineEdit_p9_lon->text();
    QString p10_lat = ui->lineEdit_p10_lat->text();
    QString p10_lon = ui->lineEdit_p10_lon->text();

    //trans to double
    P_location[0] = llToFloat(b_lat,b_lon);
    lat0 = P_location[0][0];
    lon0 = P_location[0][1];
    P_location[1] = llToFloat(p1_lat,p1_lon);
    P_location[2] = llToFloat(p2_lat,p2_lon);
    P_location[3] = llToFloat(p3_lat,p3_lon);
    P_location[4] = llToFloat(p4_lat,p4_lon);
    P_location[5] = llToFloat(p5_lat,p5_lon);
    P_location[6] = llToFloat(p6_lat,p6_lon);
    P_location[7] = llToFloat(p7_lat,p7_lon);
    P_location[8] = llToFloat(p8_lat,p8_lon);
    P_location[9] = llToFloat(p9_lat,p9_lon);
    P_location[10] = llToFloat(p10_lat,p10_lon);

    //calculate ned
    for(int j=1;j<=10;j++)
    {
        P_location[j] = ned_calculate(P_location[j][0],P_location[j][1]);
        qDebug()<<"("<<P_location[j][0]<<","<<P_location[j][1]<<")";
    }
}


void map_set::on_button_clear_clicked()
{
    ui->lineEdit_base_lat->setText("");
    ui->lineEdit_base_lon->setText("");
    ui->lineEdit_p1_lat->setText("");
    ui->lineEdit_p1_lon->setText("");
    ui->lineEdit_p2_lat->setText("");
    ui->lineEdit_p2_lon->setText("");
    ui->lineEdit_p3_lat->setText("");
    ui->lineEdit_p3_lon->setText("");
    ui->lineEdit_p4_lat->setText("");
    ui->lineEdit_p4_lon->setText("");
    ui->lineEdit_p5_lat->setText("");
    ui->lineEdit_p5_lon->setText("");
    ui->lineEdit_p6_lat->setText("");
    ui->lineEdit_p6_lon->setText("");
    ui->lineEdit_p7_lat->setText("");
    ui->lineEdit_p7_lon->setText("");
    ui->lineEdit_p8_lat->setText("");
    ui->lineEdit_p8_lon->setText("");
    ui->lineEdit_p9_lat->setText("");
    ui->lineEdit_p9_lon->setText("");
    ui->lineEdit_p10_lat->setText("");
    ui->lineEdit_p10_lon->setText("");
}

void map_set::doProcessRov_ll(QString rov_lat,QString rov_lon)
{
    QMutexLocker locker(&myMutex_xy);//加锁
    //qDebug()<<Rov_location[0][0]<<" "<<Rov_location[0][1];
    //更新上一步坐标
    Rov_location[2] = Rov_location[1];
    Rov_location[1] = Rov_location[0];
    //Renew ROV's position
    xy_mid = ned_calculate(rov_lat.toDouble(),rov_lon.toDouble());//trans to ned (x,y)
    //防止经纬度突变，限定x,y在(-20,20)内
    if(abs(xy_mid[0])>=20||abs(xy_mid[1])>=120)
        Rov_location[0] = Rov_location[1];// 数据异常时，维持原坐标不变
    else
        Rov_location[0] = xy_mid;// 正常时，取ned解算结果
    //emit to mainwindow
    emit rov_location(Rov_location[0][0],Rov_location[0][1]);
}

QVector<double> map_set::llToFloat(QString lat,QString lon)
{
    //split
//    QStringList lat_list = lat.split(".");
//    QString lat_integer = lat_list[0];
//    QString lat_decimal = lat_list[1];
//    QStringList lon_list = lon.split(".");
//    QString lon_integer = lon_list[0];
//    QString lon_decimal = lon_list[1];
//    qDebug() << lat_integer << lat_decimal;
//    //trans to float
//    double latitude = lat_integer.toDouble() + lat_decimal.toDouble()/1000000;
//    double longitude = lon_integer.toDouble() + lon_decimal.toDouble()/1000000;
    double latitude = lat.toDouble();
    double longitude = lon.toDouble();
    return {latitude,longitude};
}

QVector<double> map_set::ned_calculate(double latitude,double longitude)
{
//    double X0=0;double X=0;double dX=0;
//    double Y0=0;double Y=0;double dY=0;
//    double Z0=0;double Z=0;double dZ=0;
//    double H=0;
//    double a=6378137;
//    double b=6356752.3142;
//    double e2=(a*a-b*b)/(a*a);//e为第一离心率，e2为其平方
//    double N=a/sqrt(1-e2*sin(latitude)*sin(latitude));
//    X0=N*cos(lat0)*cos(lon0);
//    Y0=N*cos(lat0)*sin(lon0);
//    Z0=(N*(1-e2)+H)*sin(lat0);
//    X=N*cos(latitude)*cos(longitude);
//    Y=N*cos(latitude)*sin(longitude);
//    Z=(N*(1-e2)+H)*sin(latitude);
//    dX=X-X0;
//    dY=Y-Y0;
//    dZ=Z-Z0;
//    double x = -sin(latitude)*dX+cos(latitude)*dY;
//    double y = -cos(latitude)*sin(longitude)*dX-sin(latitude)*sin(longitude)*dY+cos(latitude)*dZ;
//    double z = cos(longitude)*cos(latitude)*dX+cos(longitude)*sin(latitude)*dY+sin(latitude)*dZ;
//    return {y,x};

    //calculate neda
    double e = R * cos(lat0 * M_PI / 180) * (longitude - lon0) * M_PI / 180;//精度为0.086m
    double n = R * (latitude - lat0) * M_PI / 180;//精度为0.112m，分米级
    return {e,n};
}








