/*
 * EKF.h
 *
 *  Created on: 08-Sept-2026
 *      Author: Abdullah-Wasim
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#include <stdint.h>

// EKF output structure
typedef struct
{
	float timeOfValidity;
	float latitude;
	float longitude;
	float altitude;
	float vN;
	float vE;
	float vD;
	float roll;
	float pitch;
	float yaw;
} outputEKF;

extern outputEKF ekf_out;

// Kalman filter configuration (copied from MATLAB)
typedef struct
{
	float init_att_unc;
	float init_vel_unc;
	float init_pos_unc;
	float init_b_a_unc;
	float init_b_g_unc;
	float gyro_noise_PSD;
	float accel_noise_PSD;
	float accel_bias_PSD;
	float gyro_bias_PSD;
	float pos_meas_SD;
	float vel_meas_SD;
} LC_KF_config;

// Global config (defined in EKF.c)
extern LC_KF_config LC_KF;

// pv_NED2ECEF: made private to EKF.c (no public prototype)

void init_EKF(double lat_rad, double lon_rad, double h_m, float vn, float ve, float vd, float roll, float pitch,
		float yaw, float b_a[3], float b_g[3]);

// Main prediction step: INS mechanization + covariance propagation
void predict(float imu[6], float tor_i);

// Future measurement update (to be implemented)
// Measurement update with GNSS (caller provides lat, lon (rad), height (m)
// and NED velocity components vn, ve, vd). The function converts to ECEF
// internally and performs the update.
void update(double lat_rad, double lon_rad, double h_m, float vn, float ve, float vd);

#endif /* INC_EKF_H_ */
