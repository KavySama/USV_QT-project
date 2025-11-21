#include "tracking_params.h"
#include <cmath>

// 实时参考初值（与基值初始一致）
double ref_x = 0.0;
double ref_y = 0.0;
double ref_psi_rad = 0.0;
double ref_u = 0.0;
double ref_v = 0.0;
double ref_r = 0.0; // rad/s

// 基值初始（常值部分）
double ref_x_base = 0.0;
double ref_y_base = 0.0;
double ref_psi_base_rad = 0.0;
double ref_u_base = 0.0;
double ref_v_base = 0.0;
double ref_r_base = 0.0;

// 幅值初始（全部 0 代表关闭正弦跟踪）
double ref_x_amp = 0.0;
double ref_y_amp = 0.0;
double ref_psi_amp_rad = 0.0;
double ref_u_amp = 0.0;
double ref_v_amp = 0.0;
double ref_r_amp = 0.0;

// 周期初始（设为默认 10s；周期<=0 时忽略正弦项）
double ref_x_period = 10.0;
double ref_y_period = 10.0;
double ref_psi_period = 10.0;
double ref_u_period = 10.0;
double ref_v_period = 10.0;
double ref_r_period = 10.0;

// 时间累计
double ref_time_accum = 0.0;

// 正弦参考更新函数（集中管理，避免在多个地方复制逻辑）
void update_tracking_refs(double dt)
{
	if(dt <= 0.0) return;
	ref_time_accum += dt;
	const double TWO_PI = 2.0 * M_PI;
	auto sineGen = [](double base, double amp, double period, double t)->double {
		if(amp <= 0.0 || period <= 0.0) return base; // 幅值或周期失效 -> 常值
		return base + amp * std::sin(2.0 * M_PI * t / period);
	};
	ref_x       = sineGen(ref_x_base,       ref_x_amp,       ref_x_period,       ref_time_accum);
	ref_y       = sineGen(ref_y_base,       ref_y_amp,       ref_y_period,       ref_time_accum);
	ref_psi_rad = sineGen(ref_psi_base_rad, ref_psi_amp_rad, ref_psi_period,     ref_time_accum);
	ref_u       = sineGen(ref_u_base,       ref_u_amp,       ref_u_period,       ref_time_accum);
	ref_v       = sineGen(ref_v_base,       ref_v_amp,       ref_v_period,       ref_time_accum);
	ref_r       = sineGen(ref_r_base,       ref_r_amp,       ref_r_period,       ref_time_accum);
}
