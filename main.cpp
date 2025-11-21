#include "mainwindow.h"
#include "serverwidget.h"
#include "tcp_thread.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w.setWindowTitle("水面清洁机器人控制系统");

    serverwidget w2;
    w2.show();
    w2.setWindowTitle("服务端");

    return a.exec();
}
