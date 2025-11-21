#ifndef EKF_CONFIG_DIALOG_H
#define EKF_CONFIG_DIALOG_H

#include <QDialog>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include "ekf_localization.h"

class EkfConfigDialog : public QDialog {
    Q_OBJECT
public:
    explicit EkfConfigDialog(EKFLocalization *ekf, QWidget *parent=nullptr);
private:
    EKFLocalization *m_ekf;
    QDoubleSpinBox *spinQx; QDoubleSpinBox *spinQy; QDoubleSpinBox *spinQpsi; QDoubleSpinBox *spinQu; QDoubleSpinBox *spinQv;
    QDoubleSpinBox *spinGpsStdX; QDoubleSpinBox *spinGpsStdY; QDoubleSpinBox *spinYawStdDeg; QDoubleSpinBox *spinVelStdU; QDoubleSpinBox *spinVelStdV;
    QDoubleSpinBox *spinJump;
    void applyValues();
};

#endif // EKF_CONFIG_DIALOG_H
