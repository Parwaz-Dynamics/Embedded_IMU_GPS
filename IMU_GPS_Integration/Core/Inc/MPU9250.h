/*
 * MPU9250.h
 *
 *  Created on: 24-Jul-2026
 *      Author: HP
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

typedef struct {
	I2C_HandleTypeDef i2c;
	uint8_t i2cAddress;
}Info;

typedef struct {
	float omega_ib_b[3];
	float f_ib_b[3];

	Info info;
} IMU;


HAL_StatusTypeDef initMPU9250(I2C_HandleTypeDef *i2c, uint8_t daddr);
HAL_StatusTypeDef readMPU9250(I2C_HandleTypeDef *i2c, uint8_t daddr, IMU *imu);

#endif /* INC_MPU9250_H_ */
