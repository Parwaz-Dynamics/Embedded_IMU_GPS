/*
 * MPU9250.c
 *
 *  Created on: 24-Jul-2026
 *      Author: HP
 */

#include "main.h"
#include "MPU9250.h"
#include "stdio.h"
#include "usbd_cdc_if.h"

/* MPU9250 I2C Configuration */
#define MPU9250_WHO_AM_I     0x75
#define MPU9250_PWR_MGMT_1   0x6B
#define MPU9250_SMPLRT_DIV   0x19
#define MPU9250_CONFIG       0x1A
#define MPU9250_GYRO_CONFIG  0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG2 0x1D
#define MPU9250_INT_PIN_CFG  0x37
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H  0x43

/* MPU9250 Configuration Values */
#define SMPLRT_DIV_200HZ     4
#define DLPF_1KHZ            0x01
#define GYRO_RANGE_500DPS    0x08
#define ACCEL_RANGE_4G       0x08
#define CLOCK_SEL_PLL_XGYRO  0x01

uint8_t error_msg[50];

HAL_StatusTypeDef initMPU9250(I2C_HandleTypeDef *i2c, uint8_t daddr)
{

	uint8_t MPU9250_ADDR = (daddr << 1);
	uint8_t data;

	if (HAL_I2C_IsDeviceReady(i2c, MPU9250_ADDR, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 1\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}

	uint8_t whoami;
	if (HAL_I2C_Mem_Read(i2c, MPU9250_ADDR, MPU9250_WHO_AM_I,
	I2C_MEMADD_SIZE_8BIT, &whoami, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 2\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}

	if (whoami != 0x70)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 3\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}

	data = CLOCK_SEL_PLL_XGYRO;
	if (HAL_I2C_Mem_Write(i2c, MPU9250_ADDR, MPU9250_PWR_MGMT_1,
	I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 4\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}
	HAL_Delay(10);

	data = SMPLRT_DIV_200HZ;
	if (HAL_I2C_Mem_Write(i2c, MPU9250_ADDR, MPU9250_SMPLRT_DIV,
	I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 5\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}

	data = DLPF_1KHZ;
	if (HAL_I2C_Mem_Write(i2c, MPU9250_ADDR, MPU9250_CONFIG,
	I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 6\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}

	data = GYRO_RANGE_500DPS;
	if (HAL_I2C_Mem_Write(i2c, MPU9250_ADDR, MPU9250_GYRO_CONFIG,
	I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 7\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}

	data = ACCEL_RANGE_4G;
	if (HAL_I2C_Mem_Write(i2c, MPU9250_ADDR, MPU9250_ACCEL_CONFIG,
	I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 8\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}

	data = 0x00;
	if (HAL_I2C_Mem_Write(i2c, MPU9250_ADDR, MPU9250_ACCEL_CONFIG2,
	I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 9\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}

	/* I2C bypass for magnetometer */
	uint8_t intPinCfg;
	if (HAL_I2C_Mem_Read(i2c, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
	I2C_MEMADD_SIZE_8BIT, &intPinCfg, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 10\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}

	intPinCfg |= 0x02;

	if (HAL_I2C_Mem_Write(i2c, MPU9250_ADDR, MPU9250_INT_PIN_CFG,
	I2C_MEMADD_SIZE_8BIT, &intPinCfg, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		while (1)
		{
			int len = snprintf((char*) error_msg, sizeof(error_msg), "Error 11\r\n");

			CDC_Transmit_FS(error_msg, len);
			HAL_Delay(100);
		}
		return HAL_ERROR;
	}

	/* Data-ready interrupt DISABLED (polling mode) */

	return HAL_OK;
}

HAL_StatusTypeDef readMPU9250(I2C_HandleTypeDef *i2c, uint8_t daddr, IMU *imu)
{
	uint8_t MPU9250_ADDR = (daddr << 1);
	uint8_t raw[14];
	if (HAL_I2C_Mem_Read(i2c, MPU9250_ADDR, MPU9250_ACCEL_XOUT_H,
	I2C_MEMADD_SIZE_8BIT, raw, 14, 10) != HAL_OK)
		return HAL_ERROR;

	// Accelerometer (±4G, LSB = 0.00012207 g)
	imu->f_ib_b[0] = (int16_t) ((raw[0] << 8) | raw[1]) * 0.00012207f;
	imu->f_ib_b[1] = (int16_t) ((raw[2] << 8) | raw[3]) * 0.00012207f;
	imu->f_ib_b[2] = (int16_t) ((raw[4] << 8) | raw[5]) * 0.00012207f;

	// Gyroscope (±500 DPS, LSB = 0.01526 deg/s)
	imu->omega_ib_b[0] = (int16_t) ((raw[8] << 8) | raw[9]) * 0.01526f;
	imu->omega_ib_b[1] = (int16_t) ((raw[10] << 8) | raw[11]) * 0.01526f;
	imu->omega_ib_b[2] = (int16_t) ((raw[12] << 8) | raw[13]) * 0.01526f;

	return HAL_OK;
}
