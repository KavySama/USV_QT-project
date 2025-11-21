#include "control_alg_params.h"

int g_r_track_mode = 0; // 默认 PID

double g_pid_kp_r = 1.8;
double g_pid_ki_r = 1.8;  // 初始无积分，后续可调
// kd 为增量式中的微分参数，仍使用归一化 (e - e_prev)/Ts 形式
// 在增量式公式中我们仍单独放置 g_pid_kd_r 方便调参

double g_pid_kd_r = 0.05;

// SMC 默认参数

double g_smc_lambda_r = 0.8;
double g_smc_C0_r = 25.0;
double g_smc_sigma_r = 0.05;
