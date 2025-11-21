#ifndef CONTROL_ALG_PARAMS_H
#define CONTROL_ALG_PARAMS_H

// 跟踪模式：0 PID, 1 SMC
extern int g_r_track_mode;

// PID 参数（角速度跟踪，基于增量式，误差以度/秒）
extern double g_pid_kp_r;
extern double g_pid_ki_r;
extern double g_pid_kd_r;

// SMC 参数
extern double g_smc_lambda_r;
extern double g_smc_C0_r;
extern double g_smc_sigma_r;

#endif // CONTROL_ALG_PARAMS_H
