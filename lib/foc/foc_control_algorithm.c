#include <lib/foc/foc.h>
#include <lib/foc/focutils.h>

void svm_apply_voltage_limiting(const struct device *dev, float *vd, float *vq, float Vdc)
{
	struct foc_data *f_data = dev->data;
	modulation_ctrl_t *ctrl = &(f_data->modulation);
	const float Vmax_linear = Vdc * 0.57735f * ctrl->max_modulation; // 0.57735=1/sqrt(3)
	float Vmag;
	sqrt_f32((*vd * *vd + *vq * *vq), &Vmag);

	if (Vmag > Vmax_linear) {
		ctrl->overmodulation = true;

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
	ctrl->fsw = 10000.0f;	 // 默认10kHz
	ctrl->dead_time = 1e-6f; // 默认1μs死区
	ctrl->overmodulation = false;
}
