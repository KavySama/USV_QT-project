#include "ekf_config_dialog.h"
#include <QHBoxLayout>
#include <QMessageBox>

EkfConfigDialog::EkfConfigDialog(EKFLocalization *ekf, QWidget *parent)
    : QDialog(parent), m_ekf(ekf)
{
    setWindowTitle("EKF 参数调节");
    setMinimumWidth(420);

    spinQx = new QDoubleSpinBox; spinQy = new QDoubleSpinBox; spinQpsi = new QDoubleSpinBox; spinQu = new QDoubleSpinBox; spinQv = new QDoubleSpinBox;
    spinGpsStdX = new QDoubleSpinBox; spinGpsStdY = new QDoubleSpinBox; spinYawStdDeg = new QDoubleSpinBox; spinVelStdU = new QDoubleSpinBox; spinVelStdV = new QDoubleSpinBox;
    spinJump = new QDoubleSpinBox;

    auto cfgSpin = [](QDoubleSpinBox *s, double min, double max, double step){ s->setRange(min,max); s->setDecimals(6); s->setSingleStep(step); };
    cfgSpin(spinQx,1e-8,1e-1,1e-6); cfgSpin(spinQy,1e-8,1e-1,1e-6); cfgSpin(spinQpsi,1e-8,1e-2,1e-6); cfgSpin(spinQu,1e-8,1e-1,1e-6); cfgSpin(spinQv,1e-8,1e-1,1e-6);
    cfgSpin(spinGpsStdX,0.01,20.0,0.01); cfgSpin(spinGpsStdY,0.01,20.0,0.01); cfgSpin(spinYawStdDeg,0.01,10.0,0.01); cfgSpin(spinVelStdU,0.001,5.0,0.001); cfgSpin(spinVelStdV,0.001,5.0,0.001);
    cfgSpin(spinJump,0.1,50.0,0.1);

    // 初始值填充
    if(m_ekf){
        spinQx->setValue(m_ekf->processNoiseX());
        spinQy->setValue(m_ekf->processNoiseY());
        spinQpsi->setValue(m_ekf->processNoisePsi());
        spinQu->setValue(m_ekf->processNoiseU());
        spinQv->setValue(m_ekf->processNoiseV());
        spinGpsStdX->setValue(m_ekf->gpsStdX());
        spinGpsStdY->setValue(m_ekf->gpsStdY());
        spinYawStdDeg->setValue(m_ekf->yawStdDeg());
        spinVelStdU->setValue(m_ekf->velStdU());
        spinVelStdV->setValue(m_ekf->velStdV());
        spinJump->setValue(m_ekf->jumpThresholdVal());
    }

    QFormLayout *form1 = new QFormLayout;
    form1->addRow("Qx", spinQx);
    form1->addRow("Qy", spinQy);
    form1->addRow("Qpsi", spinQpsi);
    form1->addRow("Qu", spinQu);
    form1->addRow("Qv", spinQv);

    QFormLayout *form2 = new QFormLayout;
    form2->addRow("GPS Std X (m)", spinGpsStdX);
    form2->addRow("GPS Std Y (m)", spinGpsStdY);
    form2->addRow("Yaw Std (deg)", spinYawStdDeg);
    form2->addRow("Vel U Std (m/s)", spinVelStdU);
    form2->addRow("Vel V Std (m/s)", spinVelStdV);
    form2->addRow("Jump阈值 (m)", spinJump);

    QGroupBox *grpProc = new QGroupBox("过程噪声 Q"); grpProc->setLayout(form1);
    QGroupBox *grpMeas = new QGroupBox("测量噪声 R / 跳变"); grpMeas->setLayout(form2);

    auto buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons,&QDialogButtonBox::accepted,this,[this](){ applyValues(); accept(); });
    connect(buttons,&QDialogButtonBox::rejected,this,[this](){ reject(); });

    QVBoxLayout *mainLay = new QVBoxLayout;
    mainLay->addWidget(grpProc);
    mainLay->addWidget(grpMeas);
    mainLay->addWidget(buttons);
    setLayout(mainLay);
}

void EkfConfigDialog::applyValues(){
    if(!m_ekf) return;
    m_ekf->setProcessNoise(spinQx->value(), spinQy->value(), spinQpsi->value(), spinQu->value(), spinQv->value());
    // 方差 = (标准差)^2
    m_ekf->setMeasNoiseGPS(spinGpsStdX->value()*spinGpsStdX->value(), spinGpsStdY->value()*spinGpsStdY->value());
    double yawStdRad = spinYawStdDeg->value() * M_PI / 180.0;
    m_ekf->setMeasNoiseYaw(yawStdRad*yawStdRad);
    m_ekf->setMeasNoiseVel(spinVelStdU->value()*spinVelStdU->value(), spinVelStdV->value()*spinVelStdV->value());
    m_ekf->setJumpThreshold(spinJump->value());
}

