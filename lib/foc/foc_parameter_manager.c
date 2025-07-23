#include <lib/foc/foc.h>

void _write(const struct device *dev, int16_t flag, float *input)
{
	struct foc_data *data = dev->data;
	switch (flag) {
	case FOC_PARAM_D_PID: {
		float kp, ki, kc, max, min;
		kp = input[0];
		ki = input[1];
		kc = input[2];
		max = input[3];
		min = input[4];
		pid_init(&data->id_pid, kp, ki, kc, max, min);
	} break;
	case FOC_PARAM_Q_PID: {
		float kp, ki, kc, max, min;
		kp = input[0];
		ki = input[1];
		kc = input[2];
		max = input[3];
		min = input[4];
		pid_init(&data->iq_pid, kp, ki, kc, max, min);
	} break;
	case FOC_PARAM_DQ_REF: {
		float id_ref, iq_ref;
		id_ref = input[0];
		iq_ref = input[1];
		data->id_ref = id_ref;
		data->iq_ref = iq_ref;
	} break;
	case FOC_PARAM_SPEED_REF: {
		float speed_ref;
		speed_ref = input[0];
		// LOG_INF("SPEED PLANNING");
		s_velocity_planning(&data->s_speed_ph, speed_ref);
		// data->speed_ref = speed_ref;
		// data->speed_ref = 150.0f;
	} break;
	case FOC_PARAM_POSI_REF: {
		float posi_ref;
		posi_ref = input[0];
		data->pos_ref = posi_ref;
	} break;
	case FOC_PARAM_POSI_PLANNING: {
		float posi_ref;
		posi_ref = input[0];
		data->pos_splanning_targe = posi_ref;
		data->pos_splanning_start = input[1];
	} break;
	case FOC_PARAM_DQ_REAL: {
		data->i_d = input[0];
		data->i_q = input[1];
	} break;
	case FOC_PARAM_ME_ANGLE_REAL: {
		data->angle = input[0];
		data->eangle = input[1];
	} break;
	case FOC_PARAM_BUSVOL: {
		data->bus_vol = input[0];
		data->bus_curr = input[1];
	} break;
	}
}