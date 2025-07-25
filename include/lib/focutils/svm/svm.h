/**
 * @file
 *
 * Space Vector Modulation (SVM).
 *
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LIB_SVM_SVM_H_
#define __LIB_SVM_SVM_H_

#include <stdint.h>
#include <stdbool.h>
/**
 * @defgroup spinner_control_svm Space Vector Modulation (SVM) API
 * @{
 */

/** @brief SVM duty cycles. */
typedef struct {
	/** A channel duty cycle. */
	float a;
	/** B channel duty cycle. */
	float b;
	/** C channel duty cycle. */
	float c;
} svm_duties_t;
/** @brief SVM state. */
typedef struct svm {
	/** SVM sector. */
	uint8_t sector;
	/** Duty cycles. */
	svm_duties_t duties;
	/** Minimum allowed duty cycle. */
	float d_min;
	/** Maximum allowed duty cycle. */
	float d_max;

} svm_t;

/**
 * @brief Initialize SVM.
 *
 * @param[in] svm SVM instance.
 */
void svm_init(svm_t *svm);

/**
 * @brief Set v_alpha and v_beta.
 *
 * @param[in] svm SVM instance.
 * @param[in] va v_alpha value.
 * @param[in] vb v_beta value.
 */
void svm_set(float va, float vb, float *dabc);

/** @} */

#endif /* _SPINNER_LIB_SVM_SVM_H_ */
