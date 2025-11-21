#ifndef TRACKING_PARAMS_H
#define TRACKING_PARAMS_H

// 跟踪参考值（实时值，由正弦生成器更新）
// 位置单位 m，速度单位 m/s，角度弧度制，角速度弧度每秒
extern double ref_x;      // 实时 x 参考
extern double ref_y;      // 实时 y 参考
extern double ref_psi_rad;// 实时 ψ 参考 (rad)
extern double ref_u;      // 实时 u 参考
extern double ref_v;      // 实时 v 参考
extern double ref_r;      // 实时 r 参考 (rad/s)

// 正弦跟踪基值（用户设置的常值部分）
extern double ref_x_base;
extern double ref_y_base;
extern double ref_psi_base_rad;
extern double ref_u_base;
extern double ref_v_base;
extern double ref_r_base; // rad/s 基值

// 正弦跟踪幅值（>=0，幅值为 0 表示该变量保持基值不随时间变化）
extern double ref_x_amp;
extern double ref_y_amp;
extern double ref_psi_amp_rad; // ψ 幅值（弧度）
extern double ref_u_amp;
extern double ref_v_amp;
extern double ref_r_amp;       // r 幅值（rad/s）

// 正弦跟踪周期（>0 时生效，单位 秒，周期<=0 时视为无效仅输出基值）
extern double ref_x_period;
extern double ref_y_period;
extern double ref_psi_period;
extern double ref_u_period;
extern double ref_v_period;
extern double ref_r_period;

// 正弦发生器运行时间累计（秒）
extern double ref_time_accum;

// 更新正弦参考值，dt 为时间步长(秒)
void update_tracking_refs(double dt);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#endif // TRACKING_PARAMS_H
