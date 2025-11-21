QT       += core gui network webenginewidgets charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
#CSDN找的用于解决中文乱码问题
msvc {
    QMAKE_CFLAGS += /utf-8
    QMAKE_CXXFLAGS += /utf-8
}
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    control_thread.cpp \
    eso.cpp \
    main.cpp \
    mainwindow.cpp \
    map_set.cpp \
    serverwidget.cpp \
    plotwindow.cpp \
    ekf_localization.cpp \
    ekf_config_dialog.cpp \
    control_alg_config_dialog.cpp \
    control_alg_params.cpp \
    tracking_config_dialog.cpp \
    tracking_params.cpp \
    stereo_method.cpp \
    tcp_thread.cpp

HEADERS += \
    control_thread.h \
    eso.h \
    mainwindow.h \
    map_set.h \
    myopencv.h \
    plotwindow.h \
    ekf_localization.h \
    ekf_config_dialog.h \
    control_alg_config_dialog.h \
    control_alg_params.h \
    tracking_config_dialog.h \
    tracking_params.h \
    serverwidget.h \
    stereo_method.h \
    tcp_thread.h

FORMS += \
    mainwindow.ui \
    map_set.ui \
    serverwidget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target



# 以前的相对路径设置，已失效，找不到库
#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../Opencv/opencv/build/x64/vc15/lib/ -lopencv_world3413
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../Opencv/opencv/build/x64/vc15/lib/ -lopencv_world3413d
#else:unix: LIBS += -L$$PWD/../../../../Opencv/opencv/build/x64/vc15/lib/ -lopencv_world3413

#INCLUDEPATH += $$PWD/../../../../Opencv/opencv/build/include
#DEPENDPATH += $$PWD/../../../../Opencv/opencv/build/include

# 项目右键，选择添加外部库，选择C:/Opencv/opencv/build/x64/vc15/lib，会自动生成下面的代码，注意最后两行的路径要改成include（默认是x86/vc15)
win32:CONFIG(release, debug|release): LIBS += -LC:/Opencv/opencv/build/x64/vc15/lib/ -lopencv_world3413
else:win32:CONFIG(debug, debug|release): LIBS += -LC:/Opencv/opencv/build/x64/vc15/lib/ -lopencv_world3413d
else:unix: LIBS += -LC:/Opencv/opencv/build/x64/vc15/lib/ -lopencv_world3413

INCLUDEPATH += C:/Opencv/opencv/build/include
DEPENDPATH += C:/Opencv/opencv/build/include
