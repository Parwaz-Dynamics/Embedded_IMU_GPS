/*
 * EKF.c
 *
 *  Created on: 08-Sept-2026
 *      Author: Abdullah-Wasim
 */

#include "EKF.h"
#include "arm_math.h"

// Constants

#define omega_ie 7.292115e-5

typedef struct {
	float meas_omega_ib_b[3];
	float meas_f_ib_b[3];

	double est_r_eb_e[3];
	float est_v_eb_e[3];
	float est_C_eb_e[3][3];
	float est_f_ib_b_bias[3];
	float est_omega_ib_b_bias[3];

	float P_matrix[15][15];
} STATES;

STATES x;

void skew() {

}

void INS_Mechanizaiton(float imu[6], float tor_i) {

	x.meas_f_ib_b[0] = imu[0] - x.meas_f_ib_b[0];
	x.meas_f_ib_b[1] = imu[1] - x.meas_f_ib_b[1];
	x.meas_f_ib_b[2] = imu[2] - x.meas_f_ib_b[2];

	x.meas_omega_ib_b[0] = imu[3] - x.meas_omega_ib_b[0];
	x.meas_omega_ib_b[1] = imu[4] - x.meas_omega_ib_b[1];
	x.meas_omega_ib_b[2] = imu[5] - x.meas_omega_ib_b[2];

	arm_matrix_instance_f32 meas_omega_ib_b;
	arm_mat_init_f32(&meas_omega_ib_b, 3, 1, x.meas_omega_ib_b);

	arm_matrix_instance_f32 meas_f_ib_b;
	arm_mat_init_f32(&meas_f_ib_b, 3, 1, x.meas_f_ib_b);

	double alpha_ie = omega_ie * tor_i;
	float C_EarthData[9] = { arm_cos_f32(alpha_ie), arm_sin_f32(alpha_ie), 0,
			-arm_sin_f32(alpha_ie), arm_cos_f32(alpha_ie), 0, 0, 0, 1 };
	arm_matrix_instance_f32 C_Earth;
	arm_mat_init_f32(&C_Earth, 3, 3, C_EarthData);

	float alpha_ib_bData[3];

	arm_matrix_instance_f32 alpha_ib_b;
	arm_mat_init_f32(&alpha_ib_b, 3, 1, alpha_ib_bData);

	arm_mat_scale_f32(&meas_omega_ib_b, tor_i, &alpha_ib_b);

	float mag_alpha;

	arm_dot_prod_f32(alpha_ib_bData, alpha_ib_bData, 3, &mag_alpha);
	arm_sqrt_f32(mag_alpha, &mag_alpha);

	return;
}

void predict(float imu[6], float tor_i) {

	// INS Mechanizaiton
	INS_Mechanizaiton(imu, tor_i);

	return;
}
