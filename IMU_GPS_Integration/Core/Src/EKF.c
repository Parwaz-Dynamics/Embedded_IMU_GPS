/*
 * EKF.c
 *
 *  Complete INS mechanization + covariance propagation (15‑state EKF)
 *  Created on: 08-Sept-2026
 *      Author: Abdullah-Wasim
 */

#include "EKF.h"
#include "arm_math.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------
#define OMEGA_IE  7.292115e-5f
#define R_0       6378137.0f
#define MU        3.986004418e14f
#define J_2       1.082627e-3f

// ----------------------------------------------------------------------------
// State structure (global)
// ----------------------------------------------------------------------------
typedef struct {
	float meas_omega_ib_b[3];
	float meas_f_ib_b[3];

	double est_r_eb_e[3];      // ECEF position (double for precision)
	float est_v_eb_e[3];      // ECEF velocity
	float est_C_eb_e[3][3];   // body‑to‑ECEF DCM
	double est_L_b;           // cached latitude (rad)
	float est_f_ib_b_bias[3]; // accelerometer bias (body)
	float est_omega_ib_b_bias[3]; // gyro bias (body)

	float P_matrix[15][15];    // covariance matrix
} STATES;

STATES x;

// Global output (accessible from main)
outputEKF ekf_out = { 0 };
FilterOutput filter_out = { 0 };
static float last_innovations[6] = { 0.0f };
static float last_S[36] = { 0.0f };
static float last_nis = 0.0f;
static uint8_t last_rejected = 0;

// Global configuration (can be tuned)
LC_KF_config LC_KF = {
		.init_att_unc = 0.0174533f,      // 1 deg
		.init_vel_unc = 0.1f, .init_pos_unc = 10.0f,
		.init_b_a_unc = 0.2f,                  // m/s^2 (~0.02 g)
		.init_b_g_unc = 0.0087266f,            // rad/s (0.5 deg/s)
		.gyro_noise_PSD = (0.0174533f * 0.02f / 60.0f)
				* (0.0174533f * 0.02f / 60.0f), .accel_noise_PSD = (200.0f
				* 9.80665e-6f) * (200.0f * 9.80665e-6f), .accel_bias_PSD =
				1.0e-5f, .gyro_bias_PSD = 1.0e-9f, .pos_meas_SD = 2.5f,
		.vel_meas_SD = 0.1f, .vel_d_meas_SD = 1.0f };

// ----------------------------------------------------------------------------
// 3×3 matrix helpers (inline, no CMSIS‑DSP overhead)
// ----------------------------------------------------------------------------
static inline void skew3(const float v[3], float S[9]) {
	S[0] = 0.0f;
	S[1] = -v[2];
	S[2] = v[1];
	S[3] = v[2];
	S[4] = 0.0f;
	S[5] = -v[0];
	S[6] = -v[1];
	S[7] = v[0];
	S[8] = 0.0f;
}

static inline void matmul3(const float A[9], const float B[9], float C[9]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 3; k++)
				sum += A[i * 3 + k] * B[k * 3 + j];
			C[i * 3 + j] = sum;
		}
	}
}

static inline void matadd3(const float A[9], const float B[9], float C[9]) {
	for (int i = 0; i < 9; i++)
		C[i] = A[i] + B[i];
}

static inline void matsub3(const float A[9], const float B[9], float C[9]) {
	for (int i = 0; i < 9; i++)
		C[i] = A[i] - B[i];
}

static inline void matscale3(float s, const float A[9], float C[9]) {
	for (int i = 0; i < 9; i++)
		C[i] = s * A[i];
}

static inline void matcopy3(const float A[9], float C[9]) {
	memcpy(C, A, 9 * sizeof(float));
}

// Orthonormalise DCM (Gram‑Schmidt)
static inline void renormalize3(float C[9]) {
	// Column 0
	float n0 = sqrtf(C[0] * C[0] + C[3] * C[3] + C[6] * C[6]);
	float inv_n0 = 1.0f / n0;
	C[0] *= inv_n0;
	C[3] *= inv_n0;
	C[6] *= inv_n0;
	// Column 1
	float dot = C[0] * C[1] + C[3] * C[4] + C[6] * C[7];
	C[1] -= dot * C[0];
	C[4] -= dot * C[3];
	C[7] -= dot * C[6];
	float n1 = sqrtf(C[1] * C[1] + C[4] * C[4] + C[7] * C[7]);
	float inv_n1 = 1.0f / n1;
	C[1] *= inv_n1;
	C[4] *= inv_n1;
	C[7] *= inv_n1;
	// Column 2 = cross(col0, col1)
	C[2] = C[3] * C[7] - C[6] * C[4];
	C[5] = C[6] * C[1] - C[0] * C[7];
	C[8] = C[0] * C[4] - C[3] * C[1];
}

// 3×3 matrix‑vector multiplication: y = A * x
static inline void matvec3(const float A[9], const float x[3], float y[3]) {
	y[0] = A[0] * x[0] + A[1] * x[1] + A[2] * x[2];
	y[1] = A[3] * x[0] + A[4] * x[1] + A[5] * x[2];
	y[2] = A[6] * x[0] + A[7] * x[1] + A[8] * x[2];
}

// ----------------------------------------------------------------------------
// Gravity in ECEF (from MATLAB)
// ----------------------------------------------------------------------------
static void gravity_ECEF(const double r[3], float g[3]) {
	double mag_r = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
	if (mag_r < 1e-12) {
		g[0] = g[1] = g[2] = 0.0f;
		return;
	}
	double z_scale = 5.0 * (r[2] / mag_r) * (r[2] / mag_r);
	double gamma[3];
	double R0_ratio = (R_0 / mag_r);
	double R0_ratio2 = R0_ratio * R0_ratio;
	double factor = 1.5 * J_2 * R0_ratio2;
	gamma[0] = -MU / (mag_r * mag_r * mag_r)
			* (r[0] + factor * (1.0 - z_scale) * r[0]);
	gamma[1] = -MU / (mag_r * mag_r * mag_r)
			* (r[1] + factor * (1.0 - z_scale) * r[1]);
	gamma[2] = -MU / (mag_r * mag_r * mag_r)
			* (r[2] + factor * (3.0 - z_scale) * r[2]);
	// Add centrifugal term
	double omega2 = OMEGA_IE * OMEGA_IE;
	g[0] = (float) (gamma[0] + omega2 * r[0]);
	g[1] = (float) (gamma[1] + omega2 * r[1]);
	g[2] = (float) gamma[2];
}

// ----------------------------------------------------------------------------
// Convert Euler angles (roll, pitch, yaw) to DCM (body to NED)
// ----------------------------------------------------------------------------
static void Euler2DCM(float roll, float pitch, float yaw, float C_b_n[9]) {
	float s_phi = sinf(roll), c_phi = cosf(roll);
	float s_theta = sinf(pitch), c_theta = cosf(pitch);
	float s_psi = sinf(yaw), c_psi = cosf(yaw);

	C_b_n[0] = c_theta * c_psi;
	C_b_n[1] = c_theta * s_psi;
	C_b_n[2] = -s_theta;

	C_b_n[3] = -c_phi * s_psi + s_phi * s_theta * c_psi;
	C_b_n[4] = c_phi * c_psi + s_phi * s_theta * s_psi;
	C_b_n[5] = s_phi * c_theta;

	C_b_n[6] = s_phi * s_psi + c_phi * s_theta * c_psi;
	C_b_n[7] = -s_phi * c_psi + c_phi * s_theta * s_psi;
	C_b_n[8] = c_phi * c_theta;
}

// ----------------------------------------------------------------------------
// Convert NED position (lat, lon, height) and velocity to ECEF
// ----------------------------------------------------------------------------
static void NED2ECEF(double lat_rad, double lon_rad, double h_m, float vn,
		float ve, float vd, double r_eb_e[3], float v_eb_e[3],
		const float C_b_n[9], float C_b_e[9]) {
	const double e2 = 0.00669437999014;

	double sin_lat = sin(lat_rad), cos_lat = cos(lat_rad);
	double sin_lon = sin(lon_rad), cos_lon = cos(lon_rad);
	double R_E = R_0 / sqrt(1.0 - e2 * sin_lat * sin_lat);

	// ECEF position
	r_eb_e[0] = (R_E + h_m) * cos_lat * cos_lon;
	r_eb_e[1] = (R_E + h_m) * cos_lat * sin_lon;
	r_eb_e[2] = ((1.0 - e2) * R_E + h_m) * sin_lat;

	// DCM from NED to ECEF (C_e_n)
	float C_e_n[9] = { (float) (-sin_lat * cos_lon),
			(float) (-sin_lat * sin_lon), (float) cos_lat, (float) (-sin_lon),
			(float) cos_lon, 0.0f, (float) (-cos_lat * cos_lon),
			(float) (-cos_lat * sin_lon), (float) (-sin_lat) };

	// Velocity in ECEF: v_eb_e = C_e_n' * v_ned
	float v_ned[3] = { vn, ve, vd };
	v_eb_e[0] = C_e_n[0] * v_ned[0] + C_e_n[3] * v_ned[1] + C_e_n[6] * v_ned[2];
	v_eb_e[1] = C_e_n[1] * v_ned[0] + C_e_n[4] * v_ned[1] + C_e_n[7] * v_ned[2];
	v_eb_e[2] = C_e_n[2] * v_ned[0] + C_e_n[5] * v_ned[1] + C_e_n[8] * v_ned[2];

	// Body-to-ECEF DCM: C_b_e = C_e_n' * C_b_n
	// C_e_n is orthogonal, so C_e_n' = C_e_n^T.
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 3; k++) {
				sum += C_e_n[k * 3 + i] * C_b_n[k * 3 + j]; // row i of C_e_n' is column i of C_e_n
			}
			C_b_e[i * 3 + j] = sum;
		}
	}
}

// ----------------------------------------------------------------------------
// 15×15 diagonal matrix initialisation
// ----------------------------------------------------------------------------
static void diag_matrix_15(float diag[15], float P[225]) {
	memset(P, 0, 225 * sizeof(float));
	for (int i = 0; i < 15; i++)
		P[i * 15 + i] = diag[i];
}

// Convert NED position/velocity to ECEF (position & velocity only, no attitude)
static void pv_NED2ECEF(double lat_rad, double lon_rad, double h_m, float vn,
		float ve, float vd, double r_eb_e[3], float v_eb_e[3]) {
	const double e = 0.0818191908425;
	const double e2 = e * e;

	double sin_lat = sin(lat_rad), cos_lat = cos(lat_rad);
	double sin_lon = sin(lon_rad), cos_lon = cos(lon_rad);
	double R_E = R_0 / sqrt(1.0 - e2 * sin_lat * sin_lat);

	r_eb_e[0] = (R_E + h_m) * cos_lat * cos_lon;
	r_eb_e[1] = (R_E + h_m) * cos_lat * sin_lon;
	r_eb_e[2] = ((1.0 - e2) * R_E + h_m) * sin_lat;

	// C_e_n (used as transpose for velocity)
	float C_e_n[9] = { (float) (-sin_lat * cos_lon),
			(float) (-sin_lat * sin_lon), (float) cos_lat, (float) (-sin_lon),
			(float) cos_lon, 0.0f, (float) (-cos_lat * cos_lon),
			(float) (-cos_lat * sin_lon), (float) (-sin_lat) };

	// v_eb_e = C_e_n' * v_ned
	float v_ned[3] = { vn, ve, vd };
	v_eb_e[0] = C_e_n[0] * v_ned[0] + C_e_n[3] * v_ned[1] + C_e_n[6] * v_ned[2];
	v_eb_e[1] = C_e_n[1] * v_ned[0] + C_e_n[4] * v_ned[1] + C_e_n[7] * v_ned[2];
	v_eb_e[2] = C_e_n[2] * v_ned[0] + C_e_n[5] * v_ned[1] + C_e_n[8] * v_ned[2];
}

static void ECEF2NED(const double r_eb_e[3], const float v_eb_e[3],
		const float C_b_e[9], double *lat_rad, double *lon_rad, double *h,
		float v_ned[3], float C_b_n[9]) {
	const double R0 = 6378137.0;
	const double e = 0.0818191908425;
	const double e2 = e * e;

	double xr = r_eb_e[0], yr = r_eb_e[1], zr = r_eb_e[2];

	// Longitude
	double lambda = atan2(yr, xr);

	// Closed-form latitude & height (exact MATLAB algorithm)
	double beta = sqrt(xr * xr + yr * yr);
	double k1 = sqrt(1.0 - e2) * fabs(zr);
	double k2 = e2 * R0;
	double E = (k1 - k2) / beta;
	double F = (k1 + k2) / beta;
	double P = (4.0 / 3.0) * (E * F + 1.0);
	double Q = 2.0 * (E * E - F * F);
	double D = P * P * P + Q * Q;
	double sqrtD = sqrt(D);
	double V = cbrt(sqrtD - Q) - cbrt(sqrtD + Q);
	double G = 0.5 * (sqrt(E * E + V) + E);
	double T = sqrt(G * G + (F - V * G) / (2.0 * G - E)) - G;
	double lat = (zr >= 0.0 ? 1.0 : -1.0)
			* atan((1.0 - T * T) / (2.0 * T * sqrt(1.0 - e2)));
	double h_b = (beta - R0 * T) * cos(lat)
			+ (zr - (zr >= 0.0 ? 1.0 : -1.0) * R0 * sqrt(1.0 - e2)) * sin(lat);

	*lat_rad = lat;
	*lon_rad = lambda;
	*h = h_b;

	double sin_lat = sin(lat), cos_lat = cos(lat);
	double sin_lon = sin(lambda), cos_lon = cos(lambda);

	// C_e_n (row-major, same as MATLAB)
	float C_e_n[9] = { (float) (-sin_lat * cos_lon),
			(float) (-sin_lat * sin_lon), (float) cos_lat, (float) (-sin_lon),
			(float) cos_lon, 0.0f, (float) (-cos_lat * cos_lon),
			(float) (-cos_lat * sin_lon), (float) (-sin_lat) };

	// v_ned = C_e_n * v_eb_e
	v_ned[0] = C_e_n[0] * v_eb_e[0] + C_e_n[1] * v_eb_e[1]
			+ C_e_n[2] * v_eb_e[2];
	v_ned[1] = C_e_n[3] * v_eb_e[0] + C_e_n[4] * v_eb_e[1]
			+ C_e_n[5] * v_eb_e[2];
	v_ned[2] = C_e_n[6] * v_eb_e[0] + C_e_n[7] * v_eb_e[1]
			+ C_e_n[8] * v_eb_e[2];

	// C_b_n = C_e_n * C_b_e
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			float s = 0.0f;
			for (int k = 0; k < 3; k++)
				s += C_e_n[i * 3 + k] * C_b_e[k * 3 + j];
			C_b_n[i * 3 + j] = s;
		}
}

static int inv6x6(const float A[36], float Ainv[36]) {
	float aug[6][12];
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++)
			aug[i][j] = A[i * 6 + j];
		for (int j = 6; j < 12; j++)
			aug[i][j] = (i == (j - 6)) ? 1.0f : 0.0f;
	}

	for (int i = 0; i < 6; i++) {
		// Find pivot
		int pivot = i;
		for (int j = i + 1; j < 6; j++) {
			if (fabsf(aug[j][i]) > fabsf(aug[pivot][i]))
				pivot = j;
		}
		if (fabsf(aug[pivot][i]) < 1e-12f)
			return 0; // singular

		// Swap rows
		if (pivot != i) {
			for (int j = 0; j < 12; j++) {
				float t = aug[i][j];
				aug[i][j] = aug[pivot][j];
				aug[pivot][j] = t;
			}
		}

		// Normalize pivot row
		float div = aug[i][i];
		for (int j = 0; j < 12; j++)
			aug[i][j] /= div;

		// Eliminate other rows
		for (int j = 0; j < 6; j++) {
			if (j != i) {
				float factor = aug[j][i];
				for (int k = 0; k < 12; k++)
					aug[j][k] -= factor * aug[i][k];
			}
		}
	}

	// Extract inverse
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++)
			Ainv[i * 6 + j] = aug[i][6 + j];
	}
	return 1;
}

static void DCM2Euler(const float C_b_n[9], float *roll, float *pitch,
		float *yaw) {
	// ZYX convention (roll, pitch, yaw)
	*pitch = -asinf(C_b_n[0 * 3 + 2]);                 // -asin(C13)
	*roll = atan2f(C_b_n[1 * 3 + 2], C_b_n[2 * 3 + 2]); // atan2(C23, C33)
	*yaw = atan2f(C_b_n[1 * 3 + 0], C_b_n[0 * 3 + 0]); // atan2(C21, C11)
}

static void DCM2Quaternion(const float C_b_e[9], float quat[4]) {
	float tr = C_b_e[0] + C_b_e[4] + C_b_e[8];

	if (tr > 0.0f) {
		float s = sqrtf(tr + 1.0f) * 2.0f;
		quat[0] = 0.25f * s;
		quat[1] = (C_b_e[7] - C_b_e[5]) / s;
		quat[2] = (C_b_e[2] - C_b_e[6]) / s;
		quat[3] = (C_b_e[3] - C_b_e[1]) / s;
	} else if ((C_b_e[0] > C_b_e[4]) && (C_b_e[0] > C_b_e[8])) {
		float s = sqrtf(1.0f + C_b_e[0] - C_b_e[4] - C_b_e[8]) * 2.0f;
		quat[0] = (C_b_e[7] - C_b_e[5]) / s;
		quat[1] = 0.25f * s;
		quat[2] = (C_b_e[1] + C_b_e[3]) / s;
		quat[3] = (C_b_e[2] + C_b_e[6]) / s;
	} else if (C_b_e[4] > C_b_e[8]) {
		float s = sqrtf(1.0f + C_b_e[4] - C_b_e[0] - C_b_e[8]) * 2.0f;
		quat[0] = (C_b_e[2] - C_b_e[6]) / s;
		quat[1] = (C_b_e[1] + C_b_e[3]) / s;
		quat[2] = 0.25f * s;
		quat[3] = (C_b_e[5] + C_b_e[7]) / s;
	} else {
		float s = sqrtf(1.0f + C_b_e[8] - C_b_e[0] - C_b_e[4]) * 2.0f;
		quat[0] = (C_b_e[3] - C_b_e[1]) / s;
		quat[1] = (C_b_e[2] + C_b_e[6]) / s;
		quat[2] = (C_b_e[5] + C_b_e[7]) / s;
		quat[3] = 0.25f * s;
	}
}

static void symmetrize_P(void) {
	for (int i = 0; i < 15; i++) {
		for (int j = i + 1; j < 15; j++) {
			float avg = 0.5f * (x.P_matrix[i][j] + x.P_matrix[j][i]);
			x.P_matrix[i][j] = avg;
			x.P_matrix[j][i] = avg;
		}
		if (x.P_matrix[i][i] < 1e-12f) {
			x.P_matrix[i][i] = 1e-12f;
		}
	}
}

static void update_output(void) {
	double lat_rad, lon_rad, h_m;
	float v_ned[3], C_b_n[9];

	// Convert current ECEF state to NED
	ECEF2NED(x.est_r_eb_e, x.est_v_eb_e, (const float*) x.est_C_eb_e, &lat_rad,
			&lon_rad, &h_m, v_ned, C_b_n);

	// Fill output struct
	ekf_out.latitude = (float) (lat_rad * 180.0 / M_PI);
	ekf_out.longitude = (float) (lon_rad * 180.0 / M_PI);
	ekf_out.altitude = (float) h_m;
	ekf_out.vN = v_ned[0];
	ekf_out.vE = v_ned[1];
	ekf_out.vD = v_ned[2];

	float roll, pitch, yaw;
	DCM2Euler(C_b_n, &roll, &pitch, &yaw);
	ekf_out.roll = roll * 180.0f / M_PI;
	ekf_out.pitch = pitch * 180.0f / M_PI;
	ekf_out.yaw = yaw * 180.0f / M_PI;

	float quat[4];
	DCM2Quaternion((const float*) x.est_C_eb_e, quat);

	filter_out.px = (float) x.est_r_eb_e[0];
	filter_out.py = (float) x.est_r_eb_e[1];
	filter_out.pz = (float) x.est_r_eb_e[2];
	filter_out.vn = v_ned[0];
	filter_out.ve = v_ned[1];
	filter_out.vd = v_ned[2];
	filter_out.qw = quat[0];
	filter_out.qx = quat[1];
	filter_out.qy = quat[2];
	filter_out.qz = quat[3];
	filter_out.bgx = x.est_omega_ib_b_bias[0];
	filter_out.bgy = x.est_omega_ib_b_bias[1];
	filter_out.bgz = x.est_omega_ib_b_bias[2];
	filter_out.bax = x.est_f_ib_b_bias[0];
	filter_out.bay = x.est_f_ib_b_bias[1];
	filter_out.baz = x.est_f_ib_b_bias[2];

	filter_out.P_px = x.P_matrix[6][6];
	filter_out.P_py = x.P_matrix[7][7];
	filter_out.P_pz = x.P_matrix[8][8];
	filter_out.P_vn = x.P_matrix[3][3];
	filter_out.P_ve = x.P_matrix[4][4];
	filter_out.P_vd = x.P_matrix[5][5];
	filter_out.P_rn = x.P_matrix[0][0];
	filter_out.P_re = x.P_matrix[1][1];
	filter_out.P_rd = x.P_matrix[2][2];
	filter_out.P_bgx = x.P_matrix[12][12];
	filter_out.P_bgy = x.P_matrix[13][13];
	filter_out.P_bgz = x.P_matrix[14][14];
	filter_out.P_bax = x.P_matrix[9][9];
	filter_out.P_bay = x.P_matrix[10][10];
	filter_out.P_baz = x.P_matrix[11][11];

	filter_out.innov_pn = last_innovations[0];
	filter_out.innov_pe = last_innovations[1];
	filter_out.innov_pd = last_innovations[2];
	filter_out.innov_vn = last_innovations[3];
	filter_out.innov_ve = last_innovations[4];
	filter_out.innov_vd = last_innovations[5];
	filter_out.S_pn = last_S[0];
	filter_out.S_pe = last_S[7];
	filter_out.S_pd = last_S[14];
	filter_out.S_vn = last_S[21];
	filter_out.S_ve = last_S[28];
	filter_out.S_vd = last_S[35];
	filter_out.nis = last_nis;
	filter_out.rejected = last_rejected;

	// Cache latitude for later use (geocentric radius calc)
	x.est_L_b = lat_rad;
}

int format_filter_output(char *buffer, size_t buffer_size, double time_s) {
	int len = snprintf(buffer, buffer_size, "Res,%0.3f", time_s);
	if (len < 0) {
		return len;
	}

	size_t used = (size_t) len;
	if (used >= buffer_size) {
		return len;
	}

	const float *values[] = {
		&filter_out.px,
		&filter_out.py,
		&filter_out.pz,
		&filter_out.vn,
		&filter_out.ve,
		&filter_out.vd,
		&filter_out.qw,
		&filter_out.qx,
		&filter_out.qy,
		&filter_out.qz,
		&filter_out.bgx,
		&filter_out.bgy,
		&filter_out.bgz,
		&filter_out.bax,
		&filter_out.bay,
		&filter_out.baz,
		&filter_out.P_px,
		&filter_out.P_py,
		&filter_out.P_pz,
		&filter_out.P_vn,
		&filter_out.P_ve,
		&filter_out.P_vd,
		&filter_out.P_rn,
		&filter_out.P_re,
		&filter_out.P_rd,
		&filter_out.P_bgx,
		&filter_out.P_bgy,
		&filter_out.P_bgz,
		&filter_out.P_bax,
		&filter_out.P_bay,
		&filter_out.P_baz,
		&filter_out.innov_pn,
		&filter_out.innov_pe,
		&filter_out.innov_pd,
		&filter_out.innov_vn,
		&filter_out.innov_ve,
		&filter_out.innov_vd,
		&filter_out.S_pn,
		&filter_out.S_pe,
		&filter_out.S_pd,
		&filter_out.S_vn,
		&filter_out.S_ve,
		&filter_out.S_vd,
		&filter_out.nis
	};

	for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
		int written = snprintf(buffer + used, buffer_size - used, ",%0.6f", *values[i]);
		if (written < 0) {
			return written;
		}
		used += (size_t) written;
		if (used >= buffer_size) {
			return (int) used;
		}
	}

	int written = snprintf(buffer + used, buffer_size - used, ",%d\r\n",
			filter_out.rejected);
	if (written < 0) {
		return written;
	}

	return (int) (used + (size_t) written);
}

// ----------------------------------------------------------------------------
// Initialise the global EKF state (call once before first predict)
// ----------------------------------------------------------------------------
void init_EKF(double lat_rad, double lon_rad, double h_m, float vn, float ve,
		float vd, float roll, float pitch, float yaw, float b_a[3],
		float b_g[3]) {
	// 1. Convert attitude (body to NED) from Euler angles
	float C_b_n[9];
	Euler2DCM(roll, pitch, yaw, C_b_n);

	// 2. Convert NED position/velocity to ECEF and combine with attitude
	float C_b_e[9];
	NED2ECEF(lat_rad, lon_rad, h_m, vn, ve, vd, x.est_r_eb_e, x.est_v_eb_e,
			C_b_n, C_b_e);
	;

	// Store the body-to-ECEF DCM
	memcpy(x.est_C_eb_e, C_b_e, 9 * sizeof(float));

	// 3. Set biases (often start at zero)
	memcpy(x.est_f_ib_b_bias, b_a, 3 * sizeof(float));
	memcpy(x.est_omega_ib_b_bias, b_g, 3 * sizeof(float));

	// 4. Initialise covariance matrix (diagonal)
	float P_diag[15] = { LC_KF.init_att_unc * LC_KF.init_att_unc,
			LC_KF.init_att_unc * LC_KF.init_att_unc, LC_KF.init_att_unc
					* LC_KF.init_att_unc, LC_KF.init_vel_unc
					* LC_KF.init_vel_unc, LC_KF.init_vel_unc
					* LC_KF.init_vel_unc, LC_KF.init_vel_unc
					* LC_KF.init_vel_unc, LC_KF.init_pos_unc
					* LC_KF.init_pos_unc, LC_KF.init_pos_unc
					* LC_KF.init_pos_unc, LC_KF.init_pos_unc
					* LC_KF.init_pos_unc, LC_KF.init_b_a_unc
					* LC_KF.init_b_a_unc, LC_KF.init_b_a_unc
					* LC_KF.init_b_a_unc, LC_KF.init_b_a_unc
					* LC_KF.init_b_a_unc, LC_KF.init_b_g_unc
					* LC_KF.init_b_g_unc, LC_KF.init_b_g_unc
					* LC_KF.init_b_g_unc, LC_KF.init_b_g_unc
					* LC_KF.init_b_g_unc };
	diag_matrix_15(P_diag, (float*) x.P_matrix);
}

// ----------------------------------------------------------------------------
// Public predict() – called from main loop
// ----------------------------------------------------------------------------
void predict(float imu[6], float tor_i) {
	// ----- 1. Correct IMU measurements using current biases -----
	float f_corrected[3] = { imu[0] - x.est_f_ib_b_bias[0], imu[1]
			- x.est_f_ib_b_bias[1], imu[2] - x.est_f_ib_b_bias[2] };
	float w_corrected[3] = { imu[3] - x.est_omega_ib_b_bias[0], imu[4]
			- x.est_omega_ib_b_bias[1], imu[5] - x.est_omega_ib_b_bias[2] };

	// ----- 2. Save old states -----
	double old_r[3];
	memcpy(old_r, x.est_r_eb_e, 3 * sizeof(double));
	float old_v[3];
	memcpy(old_v, x.est_v_eb_e, 3 * sizeof(float));
	float old_C[9];
	memcpy(old_C, x.est_C_eb_e, 9 * sizeof(float));
	float old_P[225];
	memcpy(old_P, x.P_matrix, 225 * sizeof(float));

	// Extract old latitude for geocentric radius in Phi
	double est_L_b_old, dummy_lon, dummy_h;
	float dummy_vned[3], dummy_Cbn[9];
	ECEF2NED(old_r, old_v, old_C, &est_L_b_old, &dummy_lon, &dummy_h,
			dummy_vned, dummy_Cbn);

	// ----- 3. Compute delta angles -----
	float alpha[3] = { w_corrected[0] * tor_i, w_corrected[1] * tor_i,
			w_corrected[2] * tor_i };
	float mag_alpha = sqrtf(
			alpha[0] * alpha[0] + alpha[1] * alpha[1] + alpha[2] * alpha[2]);

	// ----- 4. Earth rotation matrix (small angle) -----
	double alpha_ie = OMEGA_IE * tor_i;
	float C_Earth[9] = { cosf(alpha_ie), sinf(alpha_ie), 0, -sinf(alpha_ie),
			cosf(alpha_ie), 0, 0, 0, 1 };

	// ----- 5. Attitude update -----
	float Alpha[9];
	skew3(alpha, Alpha);
	float Alpha2[9];
	matmul3(Alpha, Alpha, Alpha2);

	float sin_mag, cos_mag;
	float term_a, term_a2, term_a2b;
	if (mag_alpha > 1e-8f) {
		sin_mag = sinf(mag_alpha);
		cos_mag = cosf(mag_alpha);
		float inv_mag2 = 1.0f / (mag_alpha * mag_alpha);
		term_a = sin_mag / mag_alpha;
		term_a2 = (1.0f - cos_mag) * inv_mag2;
		term_a2b = (1.0f - sin_mag / mag_alpha) * inv_mag2;
	} else {
		// Series expansions to avoid division by zero
		float a2 = mag_alpha * mag_alpha;
		term_a = 1.0f - a2 / 6.0f;
		term_a2 = 0.5f - a2 / 24.0f;
		term_a2b = 1.0f / 6.0f - a2 / 120.0f;
	}

	// Identity matrix
	float I[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

	// C_new_old = I + term_a*Alpha + term_a2*Alpha^2
	float scaled_Alpha[9], scaled_Alpha2[9];
	matscale3(term_a, Alpha, scaled_Alpha);
	matscale3(term_a2, Alpha2, scaled_Alpha2);
	float C_new_old[9];
	matadd3(I, scaled_Alpha, C_new_old);
	matadd3(C_new_old, scaled_Alpha2, C_new_old);

	// B = I + term_a2*Alpha + term_a2b*Alpha^2
	float scaled_Alpha_B[9], scaled_Alpha2_B[9];
	matscale3(term_a2, Alpha, scaled_Alpha_B);
	matscale3(term_a2b, Alpha2, scaled_Alpha2_B);
	float B[9];
	matadd3(I, scaled_Alpha_B, B);
	matadd3(B, scaled_Alpha2_B, B);

	// old_C * B
	float oldC_B[9];
	matmul3(old_C, B, oldC_B);

	// Earth correction: 0.5 * Ω_ie * old_C * tor_i
	float Omega_ie[9] = { 0, -OMEGA_IE, 0, OMEGA_IE, 0, 0, 0, 0, 0 };
	float Omega_oldC[9], earth_corr[9];
	matmul3(Omega_ie, old_C, Omega_oldC);
	matscale3(0.5f * tor_i, Omega_oldC, earth_corr);

	// Average attitude: ave_C = oldC_B - earth_corr
	float ave_C[9];
	matsub3(oldC_B, earth_corr, ave_C);

	// Final attitude: C_b_e = C_Earth * old_C * C_new_old
	float temp[9];
	matmul3(old_C, C_new_old, temp);
	float C_new[9];
	matmul3(C_Earth, temp, C_new);
	renormalize3(C_new);                     // prevent drift
	memcpy(x.est_C_eb_e, C_new, 9 * sizeof(float));

	// ----- 6. Velocity update -----
	// Specific force in ECEF
	float f_ib_e[3];
	matvec3(ave_C, f_corrected, f_ib_e);

	// Gravity at old position
	float g_e[3];
	gravity_ECEF(old_r, g_e);

	// Coriolis: 2*Ω_ie*old_v
	float coriolis_old[3];
	matvec3(Omega_ie, old_v, coriolis_old);
	for (int i = 0; i < 3; i++)
		coriolis_old[i] *= 2.0f;

	// Predictor velocity
	float v_pred[3];
	for (int i = 0; i < 3; i++)
		v_pred[i] = old_v[i] + tor_i * (f_ib_e[i] + g_e[i] - coriolis_old[i]);

	// Coriolis at v_pred
	float coriolis_new[3];
	matvec3(Omega_ie, v_pred, coriolis_new);
	for (int i = 0; i < 3; i++)
		coriolis_new[i] *= 2.0f;

	// New velocity (trapezoidal)
	float v_new[3];
	for (int i = 0; i < 3; i++)
		v_new[i] = old_v[i]
				+ tor_i
						* (f_ib_e[i] + g_e[i]
								- 0.5f * (coriolis_old[i] + coriolis_new[i]));

	memcpy(x.est_v_eb_e, v_new, 3 * sizeof(float));

	// ----- 7. Position update -----
	double r_new[3];
	for (int i = 0; i < 3; i++)
		r_new[i] = old_r[i] + 0.5 * tor_i * (old_v[i] + v_new[i]);
	memcpy(x.est_r_eb_e, r_new, 3 * sizeof(double));

	// ----- 8. Covariance propagation (15×15) -----
	// Build Phi matrix
	float Phi[225] = { 0 };
	for (int i = 0; i < 15; i++)
		Phi[i * 15 + i] = 1.0f;   // I15 baseline — REQUIRED
	float Omega_ie_skew[9] = { 0, -OMEGA_IE, 0, OMEGA_IE, 0, 0, 0, 0, 0 };

	// Phi(1:3,1:3) = I - Ω_ie*dt
	for (int i = 0; i < 3; i++)
		Phi[i * 15 + i] = 1.0f;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			Phi[i * 15 + j] -= Omega_ie_skew[i * 3 + j] * tor_i;

	// Phi(1:3,13:15) = C_b_e * dt
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			Phi[i * 15 + 12 + j] = C_new[i * 3 + j] * tor_i;

	// Phi(4:6,1:3) = -dt * skew(C_b_e * f_ib_b)
	float Cf_vec[3];
	matvec3(C_new, f_corrected, Cf_vec);   // use C_new (new attitude)
	float skew_Cf[9];
	skew3(Cf_vec, skew_Cf);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			Phi[(3 + i) * 15 + j] = -tor_i * skew_Cf[i * 3 + j];

	// Phi(4:6,4:6) = I - 2*Ω_ie*dt
	for (int i = 0; i < 3; i++)
		Phi[(3 + i) * 15 + 3 + i] = 1.0f;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			Phi[(3 + i) * 15 + 3 + j] -= 2.0f * Omega_ie_skew[i * 3 + j]
					* tor_i;

	// Phi(4:6,7:9) = -tor_i * 2 * Gravity_ECEF(r_new) / geocentric_radius * r_new_hat'
	// geocentric_radius from MATLAB: R_0/sqrt(1-(e*sin(L))^2) * sqrt(cos(L)^2 + (1-e^2)^2*sin(L)^2)
	{
		const double e_val = 0.0818191908425;
		double sin_L = sin(est_L_b_old);
		double cos_L = cos(est_L_b_old);
		double e2_val = e_val * e_val;
		double geocentric_radius = R_0 / sqrt(1.0 - e2_val * sin_L * sin_L)
				* sqrt(
						cos_L * cos_L
								+ (1.0 - e2_val) * (1.0 - e2_val) * sin_L
										* sin_L);

		// gravity at NEW position (matches MATLAB: Gravity_ECEF(r_eb_e) using new r)
		float g_new[3];
		gravity_ECEF(r_new, g_new);

		float r_norm = (float) sqrt(
				r_new[0] * r_new[0] + r_new[1] * r_new[1]
						+ r_new[2] * r_new[2]);
		if (r_norm > 1e-12f) {
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					Phi[(3 + i) * 15 + 6 + j] = -tor_i * 2.0f * g_new[i]
							/ (float) geocentric_radius
							* ((float) r_new[j] / r_norm);
		}
	}

	// Phi(4:6,10:12) = C_b_e * dt
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			Phi[(3 + i) * 15 + 9 + j] = C_new[i * 3 + j] * tor_i;

	// Phi(7:9,4:6) = I * dt
	for (int i = 0; i < 3; i++)
		Phi[(6 + i) * 15 + 3 + i] = tor_i;

	// Q matrix (discrete noise)
	float Q[225] = { 0 };
	float gyro_noise = LC_KF.gyro_noise_PSD * tor_i;
	float accel_noise = LC_KF.accel_noise_PSD * tor_i;
	float accel_bias = LC_KF.accel_bias_PSD * tor_i;
	float gyro_bias = LC_KF.gyro_bias_PSD * tor_i;
	for (int i = 0; i < 3; i++) {
		Q[i * 15 + i] = gyro_noise;          // attitude (gyro noise)
		Q[(3 + i) * 15 + 3 + i] = accel_noise;   // velocity (accel noise)
		Q[(9 + i) * 15 + 9 + i] = accel_bias;    // accel bias
		Q[(12 + i) * 15 + 12 + i] = gyro_bias;   // gyro bias
	}

	// Propagate P = Phi * (P + 0.5*Q) * Phi' + 0.5*Q
	// Use arm_matrix for 15x15
	arm_matrix_instance_f32 matPhi, matQ, matP, matTemp1, matTemp2, matTemp3;
	arm_mat_init_f32(&matPhi, 15, 15, Phi);
	arm_mat_init_f32(&matQ, 15, 15, Q);
	arm_mat_init_f32(&matP, 15, 15, old_P);
	static float temp1_data[225], temp2_data[225], temp3_data[225];
	arm_mat_init_f32(&matTemp1, 15, 15, temp1_data);
	arm_mat_init_f32(&matTemp2, 15, 15, temp2_data);
	arm_mat_init_f32(&matTemp3, 15, 15, temp3_data);

	// Temp1 = P + 0.5*Q
	arm_mat_scale_f32(&matQ, 0.5f, &matTemp1); // matTemp1 = 0.5*Q
	arm_mat_add_f32(&matP, &matTemp1, &matTemp2); // matTemp2 = P + 0.5*Q

	// Temp1 = Phi * (P + 0.5*Q)
	arm_mat_mult_f32(&matPhi, &matTemp2, &matTemp1);

	// Temp3 = Phi' (transpose)
	arm_mat_trans_f32(&matPhi, &matTemp3);

	// Temp2 = Phi * (P + 0.5*Q) * Phi'
	arm_mat_mult_f32(&matTemp1, &matTemp3, &matTemp2);

	// matP = Temp2 + 0.5*Q
	arm_mat_scale_f32(&matQ, 0.5f, &matTemp1); // matTemp1 = 0.5*Q
	arm_mat_add_f32(&matTemp2, &matTemp1, &matP);

	// Store back
	memcpy(x.P_matrix, matP.pData, 225 * sizeof(float));
	symmetrize_P();

	update_output();
}

void update(double lat_rad, double lon_rad, double h_m, float vn, float ve,
		float vd, int velocityValid) {
	// Convert GNSS NED (lat,lon,h,vn,ve,vd) to ECEF internally
	double GNSS_r_eb_e[3];
	float GNSS_v_eb_e[3];
	pv_NED2ECEF(lat_rad, lon_rad, h_m, vn, ve, vd, GNSS_r_eb_e, GNSS_v_eb_e);
	// ----- 1. Innovation -----
	float delta_z[6];
	for (int i = 0; i < 3; i++) {
		delta_z[i] = (float) (GNSS_r_eb_e[i] - x.est_r_eb_e[i]); // both double now
		delta_z[3 + i] = GNSS_v_eb_e[i] - x.est_v_eb_e[i];
	}
	last_innovations[0] = delta_z[0];
	last_innovations[1] = delta_z[1];
	last_innovations[2] = delta_z[2];
	last_innovations[3] = delta_z[3];
	last_innovations[4] = delta_z[4];
	last_innovations[5] = delta_z[5];

	// ----- 2. Measurement matrix H (6×15) -----
	float H[90] = { 0 };  // 6*15
	// H(1:3, 7:9) = -I
	for (int i = 0; i < 3; i++)
		H[i * 15 + 6 + i] = -1.0f;
	// H(4:6, 4:6) = -I
	for (int i = 0; i < 3; i++)
		H[(3 + i) * 15 + 3 + i] = -1.0f;

	// ----- 3. Measurement noise R (6×6) -----
	float R[36] = { 0 };
	float pos_sd2 = LC_KF.pos_meas_SD * LC_KF.pos_meas_SD;
	float vel_sd2 = LC_KF.vel_meas_SD * LC_KF.vel_meas_SD;
	float vel_d_sd2 = LC_KF.vel_d_meas_SD * LC_KF.vel_d_meas_SD;
	float vel_sd2_inflated = (10.0f * LC_KF.vel_meas_SD) * (10.0f * LC_KF.vel_meas_SD);
	for (int i = 0; i < 3; i++) {
		R[i * 6 + i] = pos_sd2;
		R[(3 + i) * 6 + 3 + i] = vel_sd2;
	}
	if (velocityValid == 0) {
		R[3 * 6 + 3] = vel_sd2_inflated;
		R[4 * 6 + 4] = vel_sd2_inflated;
	}
	R[5 * 6 + 5] = vel_d_sd2;

	// ----- 4. Flatten P (15×15) -----
	float P_flat[225];
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++)
			P_flat[i * 15 + j] = x.P_matrix[i][j];
	}

	// ----- 5. Compute HP = H * P (6×15) -----
	float HP[90]; // 6*15
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 15; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 15; k++) {
				sum += H[i * 15 + k] * P_flat[k * 15 + j];
			}
			HP[i * 15 + j] = sum;
		}
	}

	// ----- 6. S = HP * H' + R (6×6) -----
	float S[36] = { 0 };
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 15; k++) {
				sum += HP[i * 15 + k] * H[j * 15 + k]; // H' column j
			}
			S[i * 6 + j] = sum + R[i * 6 + j];
		}
	}

	// ----- 7. Compute Kalman gain K = P * H' * inv(S) -----
	for (int i = 0; i < 36; i++) {
		last_S[i] = S[i];
	}
	last_rejected = 0;

	// PHt = P * H' (15×6)
	float PHt[90]; // 15*6
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 6; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 15; k++) {
				sum += P_flat[i * 15 + k] * H[j * 15 + k]; // H' column j
			}
			PHt[i * 6 + j] = sum;
		}
	}

	// Inverse of S
	float Sinv[36];
	if (!inv6x6(S, Sinv)) {
		// Singular – skip update
		last_rejected = 1;
		last_nis = 0.0f;
		update_output();
		return;
	}

	float nis = 0.0f;
	float Sinv_delta_z[6] = { 0.0f };
	for (int i = 0; i < 6; i++) {
		float sum = 0.0f;
		for (int j = 0; j < 6; j++) {
			sum += Sinv[i * 6 + j] * delta_z[j];
		}
		Sinv_delta_z[i] = sum;
	}
	for (int i = 0; i < 6; i++) {
		nis += delta_z[i] * Sinv_delta_z[i];
	}
	last_nis = nis;
	if (nis > 22.46f) {
		last_rejected = 1;
		update_output();
		return;
	}

	// K = PHt * Sinv (15×6)
	float K[90]; // 15*6
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 6; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 6; k++) {
				sum += PHt[i * 6 + k] * Sinv[k * 6 + j];
			}
			K[i * 6 + j] = sum;
		}
	}

	// ----- 8. State correction dx = K * delta_z (15×1) -----
	float dx[15];
	for (int i = 0; i < 15; i++) {
		float sum = 0.0f;
		for (int j = 0; j < 6; j++) {
			sum += K[i * 6 + j] * delta_z[j];
		}
		dx[i] = sum;
	}

	// ----- 9. Apply corrections (closed‑loop) -----
	// Attitude: C_b_e_new = (I - Skew(dx(1:3))) * C_b_e_old
	float skew_dx[9];
	skew3(dx, skew_dx);                     // dx[0..2] are attitude errors
	float I_minus_skew[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
	for (int i = 0; i < 9; i++)
		I_minus_skew[i] -= skew_dx[i];

	// Flatten old C
	float old_C[9];
	memcpy(old_C, x.est_C_eb_e, 9 * sizeof(float));

	float C_new[9];
	matmul3(I_minus_skew, old_C, C_new);
	memcpy(x.est_C_eb_e, C_new, 9 * sizeof(float));

	// Velocity
	for (int i = 0; i < 3; i++)
		x.est_v_eb_e[i] -= dx[3 + i];

	// Position
	for (int i = 0; i < 3; i++)
		x.est_r_eb_e[i] -= dx[6 + i];

	// Biases
	for (int i = 0; i < 3; i++) {
		x.est_f_ib_b_bias[i] += dx[9 + i];
		x.est_omega_ib_b_bias[i] += dx[12 + i];
	}

	// ----- 10. Covariance update (Joseph form) -----
	// KH = K * H (15×15)
	float KH[225] = { 0 };
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 6; k++) {
				sum += K[i * 6 + k] * H[k * 15 + j];
			}
			KH[i * 15 + j] = sum;
		}
	}

	// I - KH
	float I_minus_KH[225];
	for (int i = 0; i < 225; i++)
		I_minus_KH[i] = -KH[i];
	for (int i = 0; i < 15; i++)
		I_minus_KH[i * 15 + i] += 1.0f;

	// temp1 = (I - KH) * P
	float temp1[225];
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 15; k++) {
				sum += I_minus_KH[i * 15 + k] * P_flat[k * 15 + j];
			}
			temp1[i * 15 + j] = sum;
		}
	}

	// temp2 = temp1 * (I - KH)'
	float temp2[225];
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 15; k++) {
				sum += temp1[i * 15 + k] * I_minus_KH[j * 15 + k]; // transpose
			}
			temp2[i * 15 + j] = sum;
		}
	}

	// KR = K * R (15×6)
	float KR[90]; // 15*6
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 6; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 6; k++) {
				sum += K[i * 6 + k] * R[k * 6 + j];
			}
			KR[i * 6 + j] = sum;
		}
	}

	// KRKt = KR * K' (15×15)
	float KRKt[225];
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 6; k++) {
				sum += KR[i * 6 + k] * K[j * 6 + k]; // K' column j
			}
			KRKt[i * 15 + j] = sum;
		}
	}

	// P_new = temp2 + KRKt
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 15; j++) {
			x.P_matrix[i][j] = temp2[i * 15 + j] + KRKt[i * 15 + j];
		}
	}

	symmetrize_P();
	update_output();
}
