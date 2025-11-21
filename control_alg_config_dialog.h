#ifndef CONTROL_ALG_CONFIG_DIALOG_H
#define CONTROL_ALG_CONFIG_DIALOG_H

#include <QDialog>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QVBoxLayout>
#include <QLabel>
#include "control_alg_params.h"

class ControlAlgConfigDialog : public QDialog {
    Q_OBJECT
public:
    explicit ControlAlgConfigDialog(QWidget *parent = nullptr) : QDialog(parent) {
        setWindowTitle("控制算法参数设置");
        auto *layout = new QVBoxLayout(this);
        auto *form = new QFormLayout();

        // 模式选择
        modeCombo = new QComboBox(this);
        modeCombo->addItem("PID", 0);
        modeCombo->addItem("SMC", 1);
        modeCombo->setCurrentIndex(g_r_track_mode);
        form->addRow("跟踪模式", modeCombo);

        // PID 参数
        pidKp = new QDoubleSpinBox(this); pidKp->setRange(0, 1000); pidKp->setDecimals(3); pidKp->setValue(g_pid_kp_r);
        pidKi = new QDoubleSpinBox(this); pidKi->setRange(0, 100); pidKi->setDecimals(4); pidKi->setValue(g_pid_ki_r);
        pidKd = new QDoubleSpinBox(this); pidKd->setRange(0, 200); pidKd->setDecimals(3); pidKd->setValue(g_pid_kd_r);
        form->addRow("PID Kp", pidKp);
        form->addRow("PID Ki", pidKi);
        form->addRow("PID Kd", pidKd);

        // SMC 参数
        smcLambda = new QDoubleSpinBox(this); smcLambda->setRange(0, 5); smcLambda->setDecimals(3); smcLambda->setValue(g_smc_lambda_r);
        smcC0 = new QDoubleSpinBox(this); smcC0->setRange(0, 200); smcC0->setDecimals(2); smcC0->setValue(g_smc_C0_r);
        smcSigma = new QDoubleSpinBox(this); smcSigma->setRange(0.001, 1.0); smcSigma->setDecimals(4); smcSigma->setValue(g_smc_sigma_r);
        form->addRow("SMC lambda", smcLambda);
        form->addRow("SMC C0", smcC0);
        form->addRow("SMC sigma", smcSigma);

        // 提示标签
        auto *tips = new QLabel("说明:\n1. PID采用增量式, 误差单位: 度/秒\n2. 死区补偿: |tau_cmd|<10 时自动提升到10\n3. SMC参数请逐步调小或调大测试稳定性", this);
        tips->setWordWrap(true);
        tips->setStyleSheet("color:#444;");

        auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        connect(buttons, &QDialogButtonBox::accepted, this, [this]() {
            g_r_track_mode = modeCombo->currentData().toInt();
            g_pid_kp_r = pidKp->value();
            g_pid_ki_r = pidKi->value();
            g_pid_kd_r = pidKd->value();
            g_smc_lambda_r = smcLambda->value();
            g_smc_C0_r = smcC0->value();
            g_smc_sigma_r = smcSigma->value();
            accept();
        });
        connect(buttons, &QDialogButtonBox::rejected, this, &ControlAlgConfigDialog::reject);

        layout->addLayout(form);
        layout->addWidget(tips);
        layout->addWidget(buttons);
        setLayout(layout);
        resize(400, 420);
    }
private:
    QComboBox *modeCombo;
    QDoubleSpinBox *pidKp;
    QDoubleSpinBox *pidKi;
    QDoubleSpinBox *pidKd;
    QDoubleSpinBox *smcLambda;
    QDoubleSpinBox *smcC0;
    QDoubleSpinBox *smcSigma;
};

#endif // CONTROL_ALG_CONFIG_DIALOG_H
