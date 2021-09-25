/*
 * GY86VanTho.h
 *
 *  Created on: Jun 3, 2021
 *      Author: Admin
 */

#ifndef INC_GY86VANTHO_H_
#define INC_GY86VANTHO_H_

#include "stdio.h"
#include "main.h"
I2C_HandleTypeDef hi2c1;

typedef enum {
	GY86_Result_OK=0x00,
	GY86_Result_Error,
	MPU6050_Result_Notconnected,
	HMC5883_Result_NotConnected,
	MS5611_Result_NotConnected,
}GY86_Result;
typedef struct Acceleromoter_t
{
	float x;
	float y;
	float z;
}Accelerometer;

typedef struct Gyrometer_t
{
	float x;
	float y;
	float z;
}Gyrometer;

typedef struct Magnetometer_t
{
	float x;
	float y;
	float z;
} Magnetometer;

typedef struct Pressure_t
{
	float pressure;
}Pressure;

GY86_Result IsReadyToInterfaceMPU6050();
GY86_Result IsReadyToInterfaceHMC5883L();
GY86_Result ReadAllParameter();

#endif /* INC_GY86VANTHO_H_ */
