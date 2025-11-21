#ifndef PLOTWINDOW_H
#define PLOTWINDOW_H

#include <QDialog>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QVBoxLayout>
#include <QMenuBar>
#include <QAction>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QVector>
#include "ekf_localization.h"

// 跟踪参考值全局变量声明
#include "tracking_params.h"
#include "control_alg_params.h"
class ControlAlgConfigDialog; // 控制算法参数对话框前向声明

class TrackingConfigDialog; // 跟踪参数对话框前向声明

class EkfConfigDialog; // 前向声明

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class PlotWindow : public QDialog
{
    Q_OBJECT
public:
    explicit PlotWindow(QWidget *parent = nullptr, EKFLocalization *ekf = nullptr);
    void addSample(double t_sec, double psi_rad, double r_rad, double x_m, double y_m, double u_ms, double v_ms);
    enum DisplayMode { RadianMode, DegreeMode };
    void setDisplayMode(DisplayMode m);
    DisplayMode displayMode() const { return m_mode; }
    void exportCsv();

private:
    // 原有角度与角速度
    QtCharts::QChartView *m_angleView;
    QtCharts::QChartView *m_rateView;
    QtCharts::QLineSeries *m_angleSeries;
    QtCharts::QLineSeries *m_rateSeries;
    // 参考值曲线（红色）
    QtCharts::QLineSeries *m_angleRefSeries;
    QtCharts::QLineSeries *m_rateRefSeries;
    QtCharts::QValueAxis *m_axisX_angle;
    QtCharts::QValueAxis *m_axisY_angle;
    QtCharts::QValueAxis *m_axisX_rate;
    QtCharts::QValueAxis *m_axisY_rate;
    // 新增 x,y,u,v 曲线
    QtCharts::QChartView *m_xView;
    QtCharts::QChartView *m_yView;
    QtCharts::QChartView *m_uView;
    QtCharts::QChartView *m_vView;
    QtCharts::QLineSeries *m_xSeries;
    QtCharts::QLineSeries *m_ySeries;
    QtCharts::QLineSeries *m_uSeries;
    QtCharts::QLineSeries *m_vSeries;
    // 参考值曲线（红色）
    QtCharts::QLineSeries *m_xRefSeries;
    QtCharts::QLineSeries *m_yRefSeries;
    QtCharts::QLineSeries *m_uRefSeries;
    QtCharts::QLineSeries *m_vRefSeries;
    QtCharts::QValueAxis *m_axisX_x;
    QtCharts::QValueAxis *m_axisY_x;
    QtCharts::QValueAxis *m_axisX_y;
    QtCharts::QValueAxis *m_axisY_y;
    QtCharts::QValueAxis *m_axisX_u;
    QtCharts::QValueAxis *m_axisY_u;
    QtCharts::QValueAxis *m_axisX_v;
    QtCharts::QValueAxis *m_axisY_v;

    double m_timeSpanSec = 60.0; // 显示窗口长度
    int m_maxPoints = 5000;
    struct Sample {
        double t; double psi_rad; double r_rad; double x_m; double y_m; double u_ms; double v_ms; // 实际值
        double ref_x; double ref_y; double ref_psi_rad; double ref_r_rad; double ref_u; double ref_v; // 参考值(记录变化)
    };
    QVector<Sample> m_samples; // 保存原始数据 (角度弧度制 + 位置米 + 速度m/s)
    DisplayMode m_mode = RadianMode;
    QMenuBar *m_menuBar = nullptr;
    QAction *actRadian = nullptr;
    QAction *actDegree = nullptr;
    QAction *actExport = nullptr;
    QAction *actConfig = nullptr;
    QAction *actTrackParams = nullptr; // 跟踪参数设置
    QAction *actControlAlg = nullptr;  // 控制算法参数
    EKFLocalization *m_ekf = nullptr;

    void adjustAxes(double t_sec);
    void rebuildSeries();
    void setupMenu();
    void openConfigDialog();
    void openTrackingDialog();
    void openControlAlgDialog();
};

#endif // PLOTWINDOW_H
