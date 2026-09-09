/*
 * EKF.h
 *
 *  Created on: 08-Sept-2026
 *      Author: Abdullah-Wasim
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#include <stddef.h>
#include <stdint.h>

// EKF output structure
typedef struct
{
	double timeOfValidity;
	double latitude;
	double longitude;
	float altitude;
	float vN;
	float vE;
	float vD;
	float roll;
	float pitch;
	float yaw;
} outputEKF;

// Full filter state dump for CDC logging.
typedef struct
{
	float px;
	float py;
	float pz;
	float vn;
	float ve;
	float vd;
	float qw;
	float qx;
	float qy;
	float qz;
	float bgx;
	float bgy;
	float bgz;
	float bax;
	float bay;
	float baz;
	float P_px;
	float P_py;
	float P_pz;
	float P_vn;
	float P_ve;
	float P_vd;
	float P_rn;
	float P_re;
	float P_rd;
	float P_bgx;
	float P_bgy;
	float P_bgz;
	float P_bax;
	float P_bay;
	float P_baz;
	float innov_pn;
	float innov_pe;
	float innov_pd;
	float innov_vn;
	float innov_ve;
	float innov_vd;
	float S_pn;
	float S_pe;
	float S_pd;
	float S_vn;
	float S_ve;
	float S_vd;
	uint8_t rejected;
} FilterOutput;

extern outputEKF ekf_out;
extern FilterOutput filter_out;

int format_filter_output(char *buffer, size_t buffer_size, double time_s);

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
