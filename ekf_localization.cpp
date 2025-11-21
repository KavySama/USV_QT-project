#include "ekf_localization.h"

EKFLocalization::EKFLocalization() {
    // 初始化协方差为适中不确定
    for(int i=0;i<25;i++) P[i]=0.0;
    P[0]=1.0;     // x
    P[6]=1.0;     // y (index (1,1)=1*5+1=6)
    P[12]=0.5;    // psi (2,2)
    P[18]=0.5;    // u (3,3)
    P[24]=0.5;    // v (4,4)
}

void EKFLocalization::reset(const EKFState &st) {
    m_state = st;
    for(int i=0;i<25;i++) P[i]=0.0;
    P[0]=1.0; P[6]=1.0; P[12]=0.5; P[18]=0.5; P[24]=0.5;
}

void EKFLocalization::setProcessNoise(double qx, double qy, double qpsi, double qu, double qv){ Qx=qx; Qy=qy; Qpsi=qpsi; Qu=qu; Qv=qv; }
void EKFLocalization::setMeasNoiseGPS(double rx, double ry){ Rx=rx; Ry=ry; }
void EKFLocalization::setMeasNoiseYaw(double rpsi){ Rpsi=rpsi; }
void EKFLocalization::setMeasNoiseVel(double ru, double rv){ Ru=ru; Rv=rv; }
void EKFLocalization::setJumpThreshold(double dist_m){ jumpThreshold=dist_m; }

// 规范化角度到 [-pi,pi]
double EKFLocalization::normalizeAngle(double a){
    while(a> M_PI) a -= 2*M_PI;
    while(a<-M_PI) a += 2*M_PI;
    return a;
}

void EKFLocalization::predict(double dt, double r){
    // 状态变量
    double x=m_state.x;
    double y=m_state.y;
    double psi=m_state.psi;
    double u=m_state.u;
    double v=m_state.v;

    // 非线性运动模型 x+ v_global * dt
    double c=std::cos(psi);
    double s=std::sin(psi);
    double vx = u*c - v*s;
    double vy = u*s + v*c;

    double x_new = x + vx*dt;
    double y_new = y + vy*dt;
    double psi_new = normalizeAngle(psi + r*dt); // 使用外部角速度 r
    double u_new = u; // 速度随机游走
    double v_new = v;

    // 雅可比 F 对状态
    // x' 对 [x,y,psi,u,v] 的偏导: [1,0, (-u*s - v*c)*dt, c*dt, -s*dt]
    // y' : [0,1, (u*c - v*s)*dt, s*dt, c*dt]
    // psi': [0,0,1,0,0]
    // u': [0,0,0,1,0]
    // v': [0,0,0,0,1]

    double fx_psi = (-u*s - v*c)*dt;
    double fy_psi = (u*c - v*s)*dt;

    // F 矩阵 5x5 行主序
    double F[25] = {
        1,0,fx_psi, c*dt, -s*dt,
        0,1,fy_psi, s*dt,  c*dt,
        0,0,1,      0,     0,
        0,0,0,      1,     0,
        0,0,0,      0,     1
    };

    // P = F P F^T + Q
    // Q 为对角阵
    double Q[25]={0};
    Q[0]=Qx; Q[6]=Qy; Q[12]=Qpsi; Q[18]=Qu; Q[24]=Qv;

    // 先计算 FP
    double FP[25]={0};
    for(int r_i=0;r_i<5;r_i++){
        for(int c_i=0;c_i<5;c_i++){
            double sum=0.0;
            for(int k=0;k<5;k++) sum += F[r_i*5+k]*P[k*5+c_i];
            FP[r_i*5+c_i]=sum;
        }
    }

    // FPF^T
    double FPFt[25]={0};
    for(int r_i=0;r_i<5;r_i++){
        for(int c_i=0;c_i<5;c_i++){
            double sum=0.0;
            for(int k=0;k<5;k++) sum += FP[r_i*5+k]*F[c_i*5+k]; // 注意 F^T 使用 c_i*5+k
            FPFt[r_i*5+c_i]=sum;
        }
    }

    for(int i=0;i<25;i++) P[i]=FPFt[i]+Q[i];

    m_state.x=x_new; m_state.y=y_new; m_state.psi=psi_new; m_state.u=u_new; m_state.v=v_new;
}

void EKFLocalization::updateGPS(double x_meas, double y_meas){
    // 残差
    double dx = x_meas - m_state.x;
    double dy = y_meas - m_state.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if(dist > jumpThreshold){
        // 跳变，忽略该次 GPS 更新
        return;
    }
    // H: [1 0 0 0 0; 0 1 0 0 0]
    // 取出位置协方差子矩阵
    double Pxx = P[0];
    double Pxy = P[1];
    double Pyx = P[5];
    double Pyy = P[6];
    // S = H P H^T + R -> 2x2
    double S11 = Pxx + Rx;
    double S12 = Pxy;
    double S21 = Pyx;
    double S22 = Pyy + Ry;
    // 求逆 S_inv (2x2)
    double det = S11*S22 - S12*S21;
    if(std::fabs(det) < 1e-12) return;
    double S_inv11 =  S22 / det;
    double S_inv12 = -S12 / det;
    double S_inv21 = -S21 / det;
    double S_inv22 =  S11 / det;

    // K = P H^T S_inv -> 5x2 (只需要前两列)
    // 第一列 (对应 x 测量)
    double Kx1 = Pxx*S_inv11 + Pxy*S_inv21; // x
    double Ky1 = Pyx*S_inv11 + Pyy*S_inv21; // y
    double Kpsi1 = P[10]*S_inv11 + P[11]*S_inv21; // psi 行 (2,0) index 2*5+0=10 ; (2,1)=11
    double Ku1   = P[15]*S_inv11 + P[16]*S_inv21; // u 行 (3,0)=15 (3,1)=16
    double Kv1   = P[20]*S_inv11 + P[21]*S_inv21; // v 行
    // 第二列 (对应 y 测量)
    double Kx2 = Pxx*S_inv12 + Pxy*S_inv22;
    double Ky2 = Pyx*S_inv12 + Pyy*S_inv22;
    double Kpsi2 = P[10]*S_inv12 + P[11]*S_inv22;
    double Ku2   = P[15]*S_inv12 + P[16]*S_inv22;
    double Kv2   = P[20]*S_inv12 + P[21]*S_inv22;

    // 更新状态
    m_state.x += Kx1*dx + Kx2*dy;
    m_state.y += Ky1*dx + Ky2*dy;
    m_state.psi += Kpsi1*dx + Kpsi2*dy;
    m_state.u += Ku1*dx + Ku2*dy;
    m_state.v += Kv1*dx + Kv2*dy;
    m_state.psi = normalizeAngle(m_state.psi);

    // 更新协方差: P = (I - K H) P
    // 这里只对前两个测量分量使用标准 Joseph 形式简化
    // 构造 KH (5x5) 只有前两行有值
    double KH[25]={0};
    // 对 x 测量: H_x = [1 0 0 0 0]
    KH[0] = Kx1; KH[1] = Kx2;      // 第一行
    KH[5] = Ky1; KH[6] = Ky2;      // 第二行
    KH[10]= Kpsi1; KH[11]=Kpsi2;   // 第三行
    KH[15]= Ku1; KH[16]= Ku2;      // 第四行
    KH[20]= Kv1; KH[21]= Kv2;      // 第五行

    // (I-KH)
    double IKH[25]={0};
    for(int i=0;i<5;i++) IKH[i*5+i]=1.0; // 单位阵
    for(int r=0;r<5;r++){
        for(int c=0;c<5;c++){
            IKH[r*5+c] -= KH[r*5+c];
        }
    }

    double newP[25]={0};
    // (I-KH)P
    double temp[25]={0};
    for(int r=0;r<5;r++){
        for(int c=0;c<5;c++){
            double sum=0.0; for(int k=0;k<5;k++) sum+=IKH[r*5+k]*P[k*5+c];
            temp[r*5+c]=sum;
        }
    }
    // 完成 newP = temp (I-KH)^T
    for(int r=0;r<5;r++){
        for(int c=0;c<5;c++){
            double sum=0.0; for(int k=0;k<5;k++) sum+=temp[r*5+k]*IKH[c*5+k];
            newP[r*5+c]=sum;
        }
    }
    for(int i=0;i<25;i++) P[i]=newP[i];
}

void EKFLocalization::updateYaw(double psi_meas){
    // 角度展开到当前附近
    double psi_m = normalizeAngle(psi_meas);
    double y = normalizeAngle(psi_m - m_state.psi); // 残差
    // H = [0 0 1 0 0]
    double S = P[12] + Rpsi; // P_psi,psi
    double Kx = P[2] / S;    // (0,2)
    double Ky = P[7] / S;    // (1,2)
    double Kpsi = P[12]/ S;  // (2,2)
    double Ku = P[17] / S;   // (3,2)
    double Kv = P[22] / S;   // (4,2)
    m_state.x += Kx*y;
    m_state.y += Ky*y;
    m_state.psi = normalizeAngle(m_state.psi + Kpsi*y);
    m_state.u += Ku*y;
    m_state.v += Kv*y;
    // 协方差更新 P = (I-KH)P
    // H 只有第三分量为1
    for(int i=0;i<5;i++){
        for(int j=0;j<5;j++){
            // P[i,j] -= K[i]*P[2,j]
            double K_i = (i==0?Kx:(i==1?Ky:(i==2?Kpsi:(i==3?Ku:Kv))));
            P[i*5+j] -= K_i * P[2*5+j];
        }
    }
}

void EKFLocalization::updateVel(double u_meas, double v_meas){
    // 速度测量独立更新 u
    double yu = u_meas - m_state.u;
    double Su = P[18] + Ru; // (3,3)
    double Kxu = P[3]/Su;   // (0,3)
    double Kyu = P[8]/Su;   // (1,3)
    double Kpsiu = P[13]/Su;// (2,3)
    double Kuu = P[18]/Su;  // (3,3)
    double Kvu = P[23]/Su;  // (4,3)
    m_state.x += Kxu*yu; m_state.y += Kyu*yu; m_state.psi = normalizeAngle(m_state.psi + Kpsiu*yu);
    m_state.u += Kuu*yu; m_state.v += Kvu*yu;
    for(int i=0;i<5;i++){
        double K_i = (i==0?Kxu:(i==1?Kyu:(i==2?Kpsiu:(i==3?Kuu:Kvu))));
        for(int j=0;j<5;j++) P[i*5+j] -= K_i * P[3*5+j];
    }
    // v
    double yv = v_meas - m_state.v;
    double Sv = P[24] + Rv; // (4,4)
    double Kxv = P[4]/Sv;   // (0,4)
    double Kyv = P[9]/Sv;   // (1,4)
    double Kpsiv = P[14]/Sv;// (2,4)
    double Kuv = P[19]/Sv;  // (3,4)
    double Kvv = P[24]/Sv;  // (4,4)
    m_state.x += Kxv*yv; m_state.y += Kyv*yv; m_state.psi = normalizeAngle(m_state.psi + Kpsiv*yv);
    m_state.u += Kuv*yv; m_state.v += Kvv*yv;
    for(int i=0;i<5;i++){
        double K_i = (i==0?Kxv:(i==1?Kyv:(i==2?Kpsiv:(i==3?Kuv:Kvv))));
        for(int j=0;j<5;j++) P[i*5+j] -= K_i * P[4*5+j];
    }
}
