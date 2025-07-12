/*

最大矢量圆限制
死区补偿算法
等


*/

#include <lib/foc/foc.h>
#include <lib/focutils/utils/focutils.h>

// 电压限制函数 - 在dq坐标系应用
void svm_apply_voltage_limiting(const struct device *dev, float *vd, float *vq, float Vdc)
{
	struct foc_data *f_data = dev->data;
	modulation_ctrl_t *ctrl = &(f_data->modulation);
	// 计算最大允许相电压幅值 (Vdc/sqrt(3))
	const float Vmax_linear = Vdc * 0.57735f * ctrl->max_modulation; // 0.57735=1/sqrt(3)
	float Vmag;
	sqrt_f32((*vd * *vd + *vq * *vq), &Vmag);

	if (Vmag > Vmax_linear) {
		// 进入过调制区域
		ctrl->overmodulation = true;

		// 线性缩放
		float scale = Vmax_linear / Vmag;
		*vd *= scale;
		*vq *= scale;
	} else {
		ctrl->overmodulation = false;
	}
}

// 初始化调制比控制器
void modulation_manager_init(modulation_ctrl_t *ctrl, float max_modulation, float dead_time,
			     float fsw)
{
	ctrl->max_modulation = max_modulation;
	ctrl->fsw = 10000.0f;    // 默认10kHz
	ctrl->dead_time = 1e-6f; // 默认1μs死区
	ctrl->overmodulation = false;
}

// SVM补偿函数 - 在αβ坐标系应用
void svm_apply_svm_compensation(const struct device *dev, float *valpha, float *vbeta, float Vdc)
{
	struct foc_data *f_data = dev->data;

	deadtime_compensation_apply((const DeadTimeCompConfig *)&f_data->comp_cfg,
				    &f_data->comp_state, valpha, vbeta, Vdc, f_data->i_alpha,
				    f_data->i_beta);
}

// 根据温度动态调整
void update_modulation_limit(modulation_ctrl_t *ctrl, float temp_c)
{
	// 温度每升高1°C，降低0.5%调制比
	float derating = 0.005f * (temp_c - 25.0f);
	ctrl->max_modulation = 0.95f - fmaxf(0, derating);
}
