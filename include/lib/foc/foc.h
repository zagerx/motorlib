#ifndef ZEPHYR_INCLUDE_CONTROL_FOC_H_
#define ZEPHYR_INCLUDE_CONTROL_FOC_H_

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "zephyr/device.h"
#include <lib/foc/svm.h>
#include <pid.h>
#include <sys/_stdint.h>
#include <sys/types.h>
#include <filter.h>
#include <lib/foc/deadtime_comp.h>
#include <s_posi_planning.h>
#include <s_trajectory_planning.h>

#if defined(CONFIG_MOTOR_DEBUG_SELFLOOPMODE)
struct self_loop_data {
	float self_eangle;
	float offset_sum;
	float offset;
	uint16_t self_count;
	uint16_t alighFlag;
};
#endif

// 调制比控制结构体
typedef struct {
	float max_modulation; // 最大允许调制比 (0.95~1.15)
	float fsw;	      // 开关频率 (Hz)
	float dead_time;      // 死区时间 (秒)
	bool overmodulation;  // 过调制标志
} modulation_ctrl_t;

struct foc_data {
	pid_cb_t id_pid;
	pid_cb_t iq_pid;
	pid_cb_t speed_pid;
	pid_cb_t pos_pid;
	float id_ref, iq_ref;
	float speed_ref;
	float speed_real;
#if defined(CONFIG_SPEEDLOOP_S_SPEEDPLAN)
	s_in_t s_speed_ph;
#endif
	float pos_real, pos_ref, pos_splanning_start, pos_splanning_targe;
	SPosPlanner s_pos_ph;
	modulation_ctrl_t modulation;
	float bus_vol;
	float bus_curr;
	/* Read only variables */
	float i_a, i_b, i_c;
	float i_alpha;
	float i_beta;
	float i_d, i_q;		      /* D/Q axis currents */
	float rads;		      /* Rotor speed (rad/s) */
	float angle;		      /* Mechanical angle */
	float eangle;		      /* Electrical angle */
	float sin_eangle, cos_eangle; /* sin/cos of electrical angle */
	float v_q, v_d;		      /* Q/D axis voltages */

#if defined(CONFIG_MOTOR_DEBUG_SELFLOOPMODE)
	struct self_loop_data self_data;
#endif
};

// struct foc_config {
// 	void (*modulate)(float, float, float *); /* Modulation function */
// };

// struct foc_api {
// 	int (*currloop_init)(const struct device *);
// 	int (*currloop)(const struct device *);
// 	int (*currloop_deinit)(const struct device *);

// 	int (*speedloop_init)(const struct device *);
// 	int (*speedloop)(const struct device *);
// 	int (*speedloop_deinit)(const struct device *);

// 	int (*posloop_init)(const struct device *);
// 	int (*posloop)(const struct device *);
// 	int (*posloop_deinit)(const struct device *);
// };

// static inline void foc_modulate(const struct device *dev, float alpha, float beta, float *dabc)
// {
// 	const struct foc_config *cfg = dev->config;
// 	cfg->modulate(alpha, beta, dabc);
// }

static inline int update_target_speed(const struct device *dev, float target)
{
	struct foc_data *data = dev->data;
#if defined(CONFIG_SPEEDLOOP_S_SPEEDPLAN)
	s_velocity_planning(&data->s_speed_ph, target);
#else
	data->speed_ref = target;
#endif
	return 0;
}
static inline int update_target_position(const struct device *dev, float start_pos,
					 float target_pos, float total_time)
{
	struct foc_data *data = dev->data;
	return s_pos_planning(&data->s_pos_ph, start_pos, target_pos, total_time);
}
static inline int foc_update_vbusvolita_cbuscurr(const struct device *dev, float bus_vol,
						 float bus_curr)
{
	struct foc_data *data = dev->data;
	data->bus_vol = bus_vol;
	data->bus_curr = bus_curr;
	return 0;
}
int posloop_init(const struct device *dev);
int posloop(const struct device *dev);
int posloop_deinit(const struct device *dev);
int speedloop_init(const struct device *dev);
int speedloop_deinit(const struct device *dev);
int speedloop(const struct device *dev);
int foc_currloop(const struct device *dev);
int foc_currloop_init(const struct device *dev);
int foc_currloop_deinit(const struct device *dev);
int foc_currloop_update_idq_Ref(const struct device *dev, float d_ref, float q_ref);

int foc_init(const struct device *dev);
float foc_self_openloop(const struct device *dev, float curr_eangle);

void svm_apply_voltage_limiting(const struct device *dev, float *vd, float *vq, float Vdc);

#endif
