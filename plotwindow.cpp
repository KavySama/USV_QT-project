#include "plotwindow.h"
#include "ekf_config_dialog.h"
#include "tracking_config_dialog.h"
#include "tracking_params.h"
#include "control_alg_config_dialog.h"
#include "control_alg_params.h"
#include <QPainter>
#include <QDateTime>
#include <algorithm>
#include <algorithm>


using namespace QtCharts;

PlotWindow::PlotWindow(QWidget *parent, EKFLocalization *ekf) : QDialog(parent), m_ekf(ekf)
{
    setWindowTitle("plot");
    resize(1100, 950);

    // 实际值系列对象
    m_angleSeries = new QLineSeries(this); m_angleSeries->setName("ψ");
    m_rateSeries  = new QLineSeries(this); m_rateSeries->setName("ψ̇");
    m_xSeries     = new QLineSeries(this); m_xSeries->setName("x");
    m_ySeries     = new QLineSeries(this); m_ySeries->setName("y");
    m_uSeries     = new QLineSeries(this); m_uSeries->setName("u");
    m_vSeries     = new QLineSeries(this); m_vSeries->setName("v");
    // 参考值系列对象（红色）
    m_angleRefSeries = new QLineSeries(this); m_angleRefSeries->setName("ψ_ref");
    m_rateRefSeries  = new QLineSeries(this); m_rateRefSeries->setName("ψ̇_ref");
    m_xRefSeries     = new QLineSeries(this); m_xRefSeries->setName("x_ref");
    m_yRefSeries     = new QLineSeries(this); m_yRefSeries->setName("y_ref");
    m_uRefSeries     = new QLineSeries(this); m_uRefSeries->setName("u_ref");
    m_vRefSeries     = new QLineSeries(this); m_vRefSeries->setName("v_ref");
    QPen refPen(Qt::red); refPen.setWidthF(1.5);
    m_angleRefSeries->setPen(refPen);
    m_rateRefSeries->setPen(refPen);
    m_xRefSeries->setPen(refPen);
    m_yRefSeries->setPen(refPen);
    m_uRefSeries->setPen(refPen);
    m_vRefSeries->setPen(refPen);

    // 图表对象
    QChart *angleChart = new QChart();
    angleChart->addSeries(m_angleSeries);
    angleChart->addSeries(m_angleRefSeries);
    angleChart->legend()->setVisible(true);
    angleChart->legend()->setAlignment(Qt::AlignRight);
    angleChart->setTitle("偏航角 ψ");

    QChart *rateChart = new QChart();
    rateChart->addSeries(m_rateSeries);
    rateChart->addSeries(m_rateRefSeries);
    rateChart->legend()->setVisible(true);
    rateChart->legend()->setAlignment(Qt::AlignRight);
    rateChart->setTitle("偏航角速度 ψ̇");

    QChart *xChart = new QChart();
    xChart->addSeries(m_xSeries);
    xChart->addSeries(m_xRefSeries);
    xChart->legend()->setVisible(true);
    xChart->legend()->setAlignment(Qt::AlignRight);
    xChart->setTitle("位置 x (m)");

    QChart *yChart = new QChart();
    yChart->addSeries(m_ySeries);
    yChart->addSeries(m_yRefSeries);
    yChart->legend()->setVisible(true);
    yChart->legend()->setAlignment(Qt::AlignRight);
    yChart->setTitle("位置 y (m)");

    QChart *uChart = new QChart();
    uChart->addSeries(m_uSeries);
    uChart->addSeries(m_uRefSeries);
    uChart->legend()->setVisible(true);
    uChart->legend()->setAlignment(Qt::AlignRight);
    uChart->setTitle("速度 u (m/s)");

    QChart *vChart = new QChart();
    vChart->addSeries(m_vSeries);
    vChart->addSeries(m_vRefSeries);
    vChart->legend()->setVisible(true);
    vChart->legend()->setAlignment(Qt::AlignRight);
    vChart->setTitle("速度 v (m/s)");

    // 坐标轴
    m_axisX_angle = new QValueAxis();
    m_axisY_angle = new QValueAxis();
    m_axisX_rate  = new QValueAxis();
    m_axisY_rate  = new QValueAxis();
    m_axisX_x     = new QValueAxis();
    m_axisY_x     = new QValueAxis();
    m_axisX_y     = new QValueAxis();
    m_axisY_y     = new QValueAxis();
    m_axisX_u     = new QValueAxis();
    m_axisY_u     = new QValueAxis();
    m_axisX_v     = new QValueAxis();
    m_axisY_v     = new QValueAxis();

    // 标题
    m_axisX_angle->setTitleText("t (s)");
    m_axisY_angle->setTitleText("ψ (rad)");
    m_axisX_rate->setTitleText("t (s)");
    m_axisY_rate->setTitleText("ψ̇ (rad/s)");
    m_axisX_x->setTitleText("t (s)");
    m_axisY_x->setTitleText("x (m)");
    m_axisX_y->setTitleText("t (s)");
    m_axisY_y->setTitleText("y (m)");
    m_axisX_u->setTitleText("t (s)");
    m_axisY_u->setTitleText("u (m/s)");
    m_axisX_v->setTitleText("t (s)");
    m_axisY_v->setTitleText("v (m/s)");

    // 网格
    QVector<QValueAxis*> axes = {m_axisX_angle,m_axisY_angle,m_axisX_rate,m_axisY_rate,m_axisX_x,m_axisY_x,m_axisX_y,m_axisY_y,m_axisX_u,m_axisY_u,m_axisX_v,m_axisY_v};
    for(auto ax: axes) ax->setGridLineVisible(true);

    // 关联轴
    angleChart->addAxis(m_axisX_angle, Qt::AlignBottom);
    angleChart->addAxis(m_axisY_angle, Qt::AlignLeft);
    m_angleSeries->attachAxis(m_axisX_angle);
    m_angleSeries->attachAxis(m_axisY_angle);
    m_angleRefSeries->attachAxis(m_axisX_angle);
    m_angleRefSeries->attachAxis(m_axisY_angle);

    rateChart->addAxis(m_axisX_rate, Qt::AlignBottom);
    rateChart->addAxis(m_axisY_rate, Qt::AlignLeft);
    m_rateSeries->attachAxis(m_axisX_rate);
    m_rateSeries->attachAxis(m_axisY_rate);
    m_rateRefSeries->attachAxis(m_axisX_rate);
    m_rateRefSeries->attachAxis(m_axisY_rate);

    xChart->addAxis(m_axisX_x, Qt::AlignBottom);
    xChart->addAxis(m_axisY_x, Qt::AlignLeft);
    m_xSeries->attachAxis(m_axisX_x);
    m_xSeries->attachAxis(m_axisY_x);
    m_xRefSeries->attachAxis(m_axisX_x);
    m_xRefSeries->attachAxis(m_axisY_x);

    yChart->addAxis(m_axisX_y, Qt::AlignBottom);
    yChart->addAxis(m_axisY_y, Qt::AlignLeft);
    m_ySeries->attachAxis(m_axisX_y);
    m_ySeries->attachAxis(m_axisY_y);
    m_yRefSeries->attachAxis(m_axisX_y);
    m_yRefSeries->attachAxis(m_axisY_y);

    uChart->addAxis(m_axisX_u, Qt::AlignBottom);
    uChart->addAxis(m_axisY_u, Qt::AlignLeft);
    m_uSeries->attachAxis(m_axisX_u);
    m_uSeries->attachAxis(m_axisY_u);
    m_uRefSeries->attachAxis(m_axisX_u);
    m_uRefSeries->attachAxis(m_axisY_u);

    vChart->addAxis(m_axisX_v, Qt::AlignBottom);
    vChart->addAxis(m_axisY_v, Qt::AlignLeft);
    m_vSeries->attachAxis(m_axisX_v);
    m_vSeries->attachAxis(m_axisY_v);
    m_vRefSeries->attachAxis(m_axisX_v);
    m_vRefSeries->attachAxis(m_axisY_v);

    // ChartView
    m_angleView = new QChartView(angleChart);
    m_rateView  = new QChartView(rateChart);
    m_xView     = new QChartView(xChart);
    m_yView     = new QChartView(yChart);
    m_uView     = new QChartView(uChart);
    m_vView     = new QChartView(vChart);
    QVector<QChartView*> views = {m_angleView,m_rateView,m_xView,m_yView,m_uView,m_vView};
    for(auto v: views) v->setRenderHint(QPainter::Antialiasing);

    // 字体优化
    QFont axisFont; axisFont.setPointSize(10); axisFont.setBold(false);
    QVector<QValueAxis*> allAxes = {m_axisX_angle,m_axisY_angle,m_axisX_rate,m_axisY_rate,m_axisX_x,m_axisY_x,m_axisX_y,m_axisY_y,m_axisX_u,m_axisY_u,m_axisX_v,m_axisY_v};
    for(auto ax: allAxes){ ax->setTitleFont(axisFont); ax->setLabelsFont(axisFont);}    

    // 两列布局：左列 x,y,psi；右列 u,v,r
    setupMenu();
    auto *topLayout = new QVBoxLayout(this);
    topLayout->addWidget(m_menuBar);
    auto *rowLayout = new QHBoxLayout();
    auto *leftCol = new QVBoxLayout();
    auto *rightCol = new QVBoxLayout();
    leftCol->addWidget(m_xView);
    leftCol->addWidget(m_yView);
    leftCol->addWidget(m_angleView); // ψ
    rightCol->addWidget(m_uView);
    rightCol->addWidget(m_vView);
    rightCol->addWidget(m_rateView); // r
    rowLayout->addLayout(leftCol,1);
    rowLayout->addLayout(rightCol,1);
    topLayout->addLayout(rowLayout);
    setLayout(topLayout);

    // 初始范围
    m_axisX_angle->setRange(0, m_timeSpanSec);
    m_axisX_rate->setRange(0, m_timeSpanSec);
    m_axisX_x->setRange(0, m_timeSpanSec);
    m_axisX_y->setRange(0, m_timeSpanSec);
    m_axisX_u->setRange(0, m_timeSpanSec);
    m_axisX_v->setRange(0, m_timeSpanSec);
    m_axisY_angle->setRange(-M_PI, M_PI);
    m_axisY_rate->setRange(-2.0, 2.0);
    m_axisY_x->setRange(-10.0, 10.0); // 可根据实际位置更新
    m_axisY_y->setRange(-10.0, 10.0);
    m_axisY_u->setRange(-2.0, 2.0);
    m_axisY_v->setRange(-2.0, 2.0);
}

void PlotWindow::addSample(double t_sec, double psi_rad, double r_rad, double x_m, double y_m, double u_ms, double v_ms)
{
    // 记录样本与当时参考值（参考可能随时间变化）
    m_samples.append({t_sec, psi_rad, r_rad, x_m, y_m, u_ms, v_ms,
                      ref_x, ref_y, ref_psi_rad, ref_r, ref_u, ref_v});
    double psi_display = (m_mode == RadianMode) ? psi_rad : (psi_rad * 180.0 / M_PI);
    double r_display  = (m_mode == RadianMode) ? r_rad  : (r_rad  * 180.0 / M_PI);
    m_angleSeries->append(t_sec, psi_display);
    m_rateSeries->append(t_sec, r_display);
    m_xSeries->append(t_sec, x_m);
    m_ySeries->append(t_sec, y_m);
    m_uSeries->append(t_sec, u_ms);
    m_vSeries->append(t_sec, v_ms);
    // 参考值现为正弦/常值混合：每次调用读取实时 ref_*（由 control_thread 正弦发生器更新）
    double psi_ref_display = (m_mode == RadianMode) ? ref_psi_rad : (ref_psi_rad * 180.0 / M_PI);
    double r_ref_display   = (m_mode == RadianMode) ? ref_r       : (ref_r       * 180.0 / M_PI);
    m_angleRefSeries->append(t_sec, psi_ref_display);
    m_rateRefSeries->append(t_sec, r_ref_display);
    m_xRefSeries->append(t_sec, ref_x);
    m_yRefSeries->append(t_sec, ref_y);
    m_uRefSeries->append(t_sec, ref_u);
    m_vRefSeries->append(t_sec, ref_v);
    if (m_angleSeries->count() > m_maxPoints) {
        int removeCount = m_angleSeries->count() - m_maxPoints;
        m_angleSeries->removePoints(0, removeCount);
        m_rateSeries->removePoints(0, removeCount);
        m_xSeries->removePoints(0, removeCount);
        m_ySeries->removePoints(0, removeCount);
        m_uSeries->removePoints(0, removeCount);
        m_vSeries->removePoints(0, removeCount);
        m_angleRefSeries->removePoints(0, removeCount);
        m_rateRefSeries->removePoints(0, removeCount);
        m_xRefSeries->removePoints(0, removeCount);
        m_yRefSeries->removePoints(0, removeCount);
        m_uRefSeries->removePoints(0, removeCount);
        m_vRefSeries->removePoints(0, removeCount);
        while (m_samples.size() > m_maxPoints) m_samples.removeFirst();
    }
    adjustAxes(t_sec);
}

void PlotWindow::adjustAxes(double t_sec)
{
    double t_min = (t_sec > m_timeSpanSec) ? (t_sec - m_timeSpanSec) : 0.0;
    m_axisX_angle->setRange(t_min, t_min + m_timeSpanSec);
    m_axisX_rate->setRange(t_min, t_min + m_timeSpanSec);
    m_axisX_x->setRange(t_min, t_min + m_timeSpanSec);
    m_axisX_y->setRange(t_min, t_min + m_timeSpanSec);
    m_axisX_u->setRange(t_min, t_min + m_timeSpanSec);
    m_axisX_v->setRange(t_min, t_min + m_timeSpanSec);
    // 角度轴
    if(m_mode == RadianMode) {
        m_axisY_angle->setRange(-M_PI, M_PI);
        m_axisY_angle->setTitleText("ψ (rad)");
    } else {
        m_axisY_angle->setRange(-180.0, 180.0);
        m_axisY_angle->setTitleText("ψ (deg)");
    }
    // 角速度轴
    if(m_mode == RadianMode) {
        m_axisY_rate->setRange(-2.0, 2.0); // rad/s 范围
        m_axisY_rate->setTitleText("ψ̇ (rad/s)");
    } else {
        m_axisY_rate->setRange(-120.0, 120.0); // deg/s 近似范围
        m_axisY_rate->setTitleText("ψ̇ (deg/s)");
    }
    // x,y,u,v 轴简单自适应 (可以根据数据动态扩展)
    auto adjustRange = [](QValueAxis* axis, double minV, double maxV){
        double span = maxV - minV;
        if(span < 1e-6) span = 1.0;
        axis->setRange(minV - 0.1*span, maxV + 0.1*span);
    };
    if(!m_samples.isEmpty()) {
        double xmin=m_samples.last().x_m, xmax=m_samples.last().x_m;
        double ymin=m_samples.last().y_m, ymax=m_samples.last().y_m;
        double umin=m_samples.last().u_ms, umax=m_samples.last().u_ms;
        double vmin=m_samples.last().v_ms, vmax=m_samples.last().v_ms;
        int scan = std::min((int)m_samples.size(), 500); // 最近500点用于范围
        for(int i=m_samples.size()-scan; i<m_samples.size(); ++i){
            const auto &s=m_samples[i];
            xmin = std::min(xmin,s.x_m); xmax=std::max(xmax,s.x_m);
            ymin = std::min(ymin,s.y_m); ymax=std::max(ymax,s.y_m);
            umin = std::min(umin,s.u_ms); umax=std::max(umax,s.u_ms);
            vmin = std::min(vmin,s.v_ms); vmax=std::max(vmax,s.v_ms);
        }
        adjustRange(m_axisY_x,xmin,xmax);
        adjustRange(m_axisY_y,ymin,ymax);
        adjustRange(m_axisY_u,umin,umax);
        adjustRange(m_axisY_v,vmin,vmax);
    }
}

void PlotWindow::rebuildSeries()
{
    m_angleSeries->clear(); m_angleRefSeries->clear();
    m_rateSeries->clear();  m_rateRefSeries->clear();
    m_xSeries->clear();     m_xRefSeries->clear();
    m_ySeries->clear();     m_yRefSeries->clear();
    m_uSeries->clear();     m_uRefSeries->clear();
    m_vSeries->clear();     m_vRefSeries->clear();
    for(const auto &s : m_samples) {
        double psi_display = (m_mode == RadianMode) ? s.psi_rad : (s.psi_rad * 180.0 / M_PI);
        double r_display  = (m_mode == RadianMode) ? s.r_rad  : (s.r_rad  * 180.0 / M_PI);
        m_angleSeries->append(s.t, psi_display);
        m_rateSeries->append(s.t, r_display);
        m_xSeries->append(s.t, s.x_m);
        m_ySeries->append(s.t, s.y_m);
        m_uSeries->append(s.t, s.u_ms);
        m_vSeries->append(s.t, s.v_ms);
        double psi_ref_display = (m_mode == RadianMode) ? s.ref_psi_rad : (s.ref_psi_rad * 180.0 / M_PI);
        double r_ref_display   = (m_mode == RadianMode) ? s.ref_r_rad   : (s.ref_r_rad   * 180.0 / M_PI);
        m_angleRefSeries->append(s.t, psi_ref_display);
        m_rateRefSeries->append(s.t, r_ref_display);
        m_xRefSeries->append(s.t, s.ref_x);
        m_yRefSeries->append(s.t, s.ref_y);
        m_uRefSeries->append(s.t, s.ref_u);
        m_vRefSeries->append(s.t, s.ref_v);
    }
    if(!m_samples.isEmpty()) adjustAxes(m_samples.last().t);
}

void PlotWindow::setDisplayMode(DisplayMode m)
{
    if(m_mode == m) return;
    m_mode = m;
    rebuildSeries();
}

void PlotWindow::exportCsv()
{
    if(m_samples.isEmpty()) return;
    QString fileName = QFileDialog::getSaveFileName(this, "导出CSV", QDir::homePath()+"/psi_plot.csv", "CSV Files (*.csv)");
    if(fileName.isEmpty()) return;
    QFile f(fileName);
    if(!f.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&f);
    out.setCodec("UTF-8");
    out << "t(s),psi(rad),psi(deg),r(rad/s),r(deg/s),x(m),y(m),u(m/s),v(m/s),x_ref(m),y_ref(m),psi_ref(rad),r_ref(rad/s),u_ref(m/s),v_ref(m/s)\n";
    for(const auto &s : m_samples) {
        double psi_deg = s.psi_rad * 180.0 / M_PI;
        double r_deg  = s.r_rad * 180.0 / M_PI;
        // 参考值角度直接输出弧度，不再重复度
        out << QString::number(s.t,'f',3) << ','
            << QString::number(s.psi_rad,'f',6) << ','
            << QString::number(psi_deg,'f',3) << ','
            << QString::number(s.r_rad,'f',6) << ','
            << QString::number(r_deg,'f',3) << ','
            << QString::number(s.x_m,'f',3) << ','
            << QString::number(s.y_m,'f',3) << ','
            << QString::number(s.u_ms,'f',4) << ','
            << QString::number(s.v_ms,'f',4) << ','
            << QString::number(s.ref_x,'f',3) << ','
            << QString::number(s.ref_y,'f',3) << ','
            << QString::number(s.ref_psi_rad,'f',6) << ','
            << QString::number(s.ref_r_rad,'f',6) << ','
            << QString::number(s.ref_u,'f',4) << ','
            << QString::number(s.ref_v,'f',4) << '\n';
    }
    f.close();
}

void PlotWindow::setupMenu()
{
    m_menuBar = new QMenuBar(this);
    QMenu *menuDisplay = new QMenu("显示模式", m_menuBar);
    QMenu *menuExport  = new QMenu("数据", m_menuBar);
    QMenu *menuEkf     = new QMenu("滤波", m_menuBar);
    QMenu *menuTrack   = new QMenu("跟踪", m_menuBar);
    QMenu *menuControl = new QMenu("控制", m_menuBar);

    actRadian = new QAction("弧度(rad)", this);
    actDegree = new QAction("角度(°)", this);
    actRadian->setCheckable(true);
    actDegree->setCheckable(true);
    actRadian->setChecked(true);
    QActionGroup *group = new QActionGroup(this);
    group->addAction(actRadian);
    group->addAction(actDegree);
    group->setExclusive(true);

    connect(actRadian, &QAction::triggered, this, [this](){ setDisplayMode(RadianMode); });
    connect(actDegree, &QAction::triggered, this, [this](){ setDisplayMode(DegreeMode); });

    actExport = new QAction("导出CSV", this);
    connect(actExport, &QAction::triggered, this, [this](){ exportCsv(); });

    actConfig = new QAction("参数调节", this);
    connect(actConfig, &QAction::triggered, this, [this](){ openConfigDialog(); });

    actTrackParams = new QAction("跟踪目标设置", this);
    connect(actTrackParams, &QAction::triggered, this, [this](){ openTrackingDialog(); });

    actControlAlg = new QAction("控制算法参数", this);
    connect(actControlAlg, &QAction::triggered, this, [this](){ openControlAlgDialog(); });

    menuDisplay->addAction(actRadian);
    menuDisplay->addAction(actDegree);
    menuExport->addAction(actExport);
    menuEkf->addAction(actConfig);
    m_menuBar->addMenu(menuDisplay);
    m_menuBar->addMenu(menuExport);
    m_menuBar->addMenu(menuEkf);
    menuTrack->addAction(actTrackParams);
    m_menuBar->addMenu(menuTrack);
    menuControl->addAction(actControlAlg);
    m_menuBar->addMenu(menuControl);
}

void PlotWindow::openConfigDialog()
{
    if(!m_ekf) return;
    EkfConfigDialog dlg(m_ekf, this);
    dlg.exec();
}

void PlotWindow::openTrackingDialog()
{
    TrackingConfigDialog dlg(this);
    if(dlg.exec() == QDialog::Accepted) {
        // 修改后参考曲线不需要整体重绘（阶跃即可），仅在需要切换显示模式时由 rebuildSeries 处理
    }
}

void PlotWindow::openControlAlgDialog()
{
    ControlAlgConfigDialog dlg(this);
    dlg.exec();
}
