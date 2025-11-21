#ifndef EKF_LOCALIZATION_H
#define EKF_LOCALIZATION_H

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



struct EKFState {
    double x;   // 位置 x (m)
    double y;   // 位置 y (m)
    double psi; // 航向 (rad)
    double u;   // 纵向速度 (m/s)
    double v;   // 侧向速度 (m/s)
};

class EKFLocalization {
public:
    EKFLocalization();
    void reset(const EKFState &st);
    void setProcessNoise(double qx, double qy, double qpsi, double qu, double qv);
    void setMeasNoiseGPS(double rx, double ry);
    void setMeasNoiseYaw(double rpsi);
    void setMeasNoiseVel(double ru, double rv);
    void setJumpThreshold(double dist_m);
    // 访问当前参数（方差或阈值）
    double processNoiseX() const { return Qx; }
    double processNoiseY() const { return Qy; }
    double processNoisePsi() const { return Qpsi; }
    double processNoiseU() const { return Qu; }
    double processNoiseV() const { return Qv; }
    double measVarGPSX() const { return Rx; }
    double measVarGPSY() const { return Ry; }
    double measVarYaw() const { return Rpsi; }
    double measVarU() const { return Ru; }
    double measVarV() const { return Rv; }
    double jumpThresholdVal() const { return jumpThreshold; }
    // 标准差形式（便于界面显示）
    double gpsStdX() const { return std::sqrt(Rx); }
    double gpsStdY() const { return std::sqrt(Ry); }
    double yawStdDeg() const { return std::sqrt(Rpsi) * 180.0 / M_PI; }
    double velStdU() const { return std::sqrt(Ru); }
    double velStdV() const { return std::sqrt(Rv); }

    // 预测：给定 dt 和外部估计的角速度 r (rad/s)
    void predict(double dt, double r);
    // 更新：GPS位置
    void updateGPS(double x_meas, double y_meas);
    // 更新：航向 (rad)
    void updateYaw(double psi_meas);
    // 更新：速度 (m/s)
    void updateVel(double u_meas, double v_meas);

    const EKFState &state() const { return m_state; }

private:
    EKFState m_state{};
    // 协方差矩阵 P 5x5, 行主序
    double P[25];

    // 噪声参数
    double Qx=1e-4, Qy=1e-4, Qpsi=1e-5, Qu=1e-3, Qv=1e-3;
    double Rx=0.5*0.5, Ry=0.5*0.5; // GPS 均方误差 (m^2)
    double Rpsi=(0.5*M_PI/180.0)*(0.5*M_PI/180.0); // 航向测量噪声方差 (0.5 deg)
    double Ru=0.2*0.2, Rv=0.2*0.2; // 速度测量噪声 (m/s)^2
    double jumpThreshold=2.0; // m, 超过认为跳变

    double normalizeAngle(double a);
};

#endif // EKF_LOCALIZATION_H
