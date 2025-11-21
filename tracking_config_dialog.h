#ifndef TRACKING_CONFIG_DIALOG_H
#define TRACKING_CONFIG_DIALOG_H

#include <QDialog>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QLabel>
#include "tracking_params.h"
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class TrackingConfigDialog : public QDialog
{
    Q_OBJECT
public:
    explicit TrackingConfigDialog(QWidget *parent = nullptr)
    {
        setWindowTitle("正弦跟踪参数设置");
        auto *form = new QFormLayout();

        // 基值 + 幅值 + 周期  (ψ 幅值输入为度，再转换为弧度)
        spinXBase = makeSpin(-1000,1000,3,ref_x_base);
        spinXAmp  = makeSpin(0,1000,3,ref_x_amp);
        spinXPer  = makeSpin(0,1000,3,ref_x_period);
        form->addRow("x基值 (m)", spinXBase);
        form->addRow("x幅值 (m)", spinXAmp);
        form->addRow("x周期 (s)", spinXPer);

        spinYBase = makeSpin(-1000,1000,3,ref_y_base);
        spinYAmp  = makeSpin(0,1000,3,ref_y_amp);
        spinYPer  = makeSpin(0,1000,3,ref_y_period);
        form->addRow("y基值 (m)", spinYBase);
        form->addRow("y幅值 (m)", spinYAmp);
        form->addRow("y周期 (s)", spinYPer);

        spinPsiBaseDeg = makeSpin(-180,180,2,ref_psi_base_rad * 180.0/M_PI);
        spinPsiAmpDeg  = makeSpin(0,180,2,ref_psi_amp_rad * 180.0/M_PI);
        spinPsiPer     = makeSpin(0,1000,3,ref_psi_period);
        form->addRow("ψ基值 (°)", spinPsiBaseDeg);
        form->addRow("ψ幅值 (°)", spinPsiAmpDeg);
        form->addRow("ψ周期 (s)", spinPsiPer);

        spinUBase = makeSpin(-20,20,3,ref_u_base);
        spinUAmp  = makeSpin(0,20,3,ref_u_amp);
        spinUPer  = makeSpin(0,1000,3,ref_u_period);
        form->addRow("u基值 (m/s)", spinUBase);
        form->addRow("u幅值 (m/s)", spinUAmp);
        form->addRow("u周期 (s)", spinUPer);

        spinVBase = makeSpin(-20,20,3,ref_v_base);
        spinVAmp  = makeSpin(0,20,3,ref_v_amp);
        spinVPer  = makeSpin(0,1000,3,ref_v_period);
        form->addRow("v基值 (m/s)", spinVBase);
        form->addRow("v幅值 (m/s)", spinVAmp);
        form->addRow("v周期 (s)", spinVPer);

        spinRBase = makeSpin(-10,10,4,ref_r_base);
        spinRAmp  = makeSpin(0,10,4,ref_r_amp);
        spinRPer  = makeSpin(0,1000,3,ref_r_period);
        form->addRow("r基值 (rad/s)", spinRBase);
        form->addRow("r幅值 (rad/s)", spinRAmp);
        form->addRow("r周期 (s)", spinRPer);

        auto *tips = new QLabel("提示: 幅值=0 或 周期<=0 则输出基值; 角速度跟踪判定条件改为 r 幅值>0", this);
        tips->setStyleSheet("color:#555;");

        auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        connect(buttons, &QDialogButtonBox::accepted, this, [this]() {
            // 保存基值
            ref_x_base = spinXBase->value();
            ref_y_base = spinYBase->value();
            ref_psi_base_rad = spinPsiBaseDeg->value() * M_PI / 180.0;
            ref_u_base = spinUBase->value();
            ref_v_base = spinVBase->value();
            ref_r_base = spinRBase->value();
            // 保存幅值
            ref_x_amp = spinXAmp->value();
            ref_y_amp = spinYAmp->value();
            ref_psi_amp_rad = spinPsiAmpDeg->value() * M_PI / 180.0;
            ref_u_amp = spinUAmp->value();
            ref_v_amp = spinVAmp->value();
            ref_r_amp = spinRAmp->value();
            // 保存周期
            ref_x_period = spinXPer->value();
            ref_y_period = spinYPer->value();
            ref_psi_period = spinPsiPer->value();
            ref_u_period = spinUPer->value();
            ref_v_period = spinVPer->value();
            ref_r_period = spinRPer->value();
            accept();
        });
        connect(buttons, &QDialogButtonBox::rejected, this, &TrackingConfigDialog::reject);

        auto *vl = new QVBoxLayout(this);
        vl->addLayout(form);
        vl->addWidget(tips);
        vl->addWidget(buttons);
        setLayout(vl);
        resize(400, 640);
    }
private:
    QDoubleSpinBox *makeSpin(double min, double max, int dec, double val)
    {
        auto *s = new QDoubleSpinBox(this);
        s->setRange(min,max);
        s->setDecimals(dec);
        s->setValue(val);
        return s;
    }
    QDoubleSpinBox *spinXBase; QDoubleSpinBox *spinXAmp; QDoubleSpinBox *spinXPer;
    QDoubleSpinBox *spinYBase; QDoubleSpinBox *spinYAmp; QDoubleSpinBox *spinYPer;
    QDoubleSpinBox *spinPsiBaseDeg; QDoubleSpinBox *spinPsiAmpDeg; QDoubleSpinBox *spinPsiPer;
    QDoubleSpinBox *spinUBase; QDoubleSpinBox *spinUAmp; QDoubleSpinBox *spinUPer;
    QDoubleSpinBox *spinVBase; QDoubleSpinBox *spinVAmp; QDoubleSpinBox *spinVPer;
    QDoubleSpinBox *spinRBase; QDoubleSpinBox *spinRAmp; QDoubleSpinBox *spinRPer;
};

#endif // TRACKING_CONFIG_DIALOG_H
