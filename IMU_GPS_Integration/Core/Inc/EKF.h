/*
 * EKF.h
 *
 *  Created on: 08-Sept-2026
 *      Author: Abdullah-Wasim
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

typedef struct{
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
}outputEKF;

void predict(float imu[6], float tor_i);
void update();


#endif /* INC_EKF_H_ */
