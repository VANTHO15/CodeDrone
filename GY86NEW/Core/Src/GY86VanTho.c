/*
 * GY86VanTho.c
 *
 *  Created on: Jun 3, 2021
 *      Author: Admin
 */
#include "GY86VanTho.h"
#include <stdio.h>

// i2c device addresses
#define HMC5883L_ADDRESS 0b0011110
#define HMC5883L_ADDRESS_DATASHEET 	0b0011110
#define MS5611_ADDRESS   0b1110111
#define MPU6050_I2C_ADDR			0xD0	// sau khi đã dịch bit
#define MPU6050_I_AM				0x68
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_I2C_SLV0_ADDR		0x25
#define	MPU6050_I2C_SLV0_REG		0x26
#define MPU6050_I2C_SLV0_CTRL		0x27
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_EXT_SENS_DATA_0		0x49
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_I2C_MST_DELAY_CTRL	0x67
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

#define SMPLRT_DIV_REG    0x19
#define GYRO_CONFIG_REG    0x1B

#define HMC5883L_CONFIG_A			0x00
#define HMC5883L_CONFIG_B			0x01
#define HMC5883L_MODE_REG			0x02
#define HMC5883L_MAGN_X_H			0x03
#define HMC5883L_MAGN_X_L			0x04
#define HMC5883L_MAGN_Z_H			0x05
#define HMC5883L_MAGN_Z_L			0x06
#define HMC5883L_MAGN_Y_H			0x07
#define HMC5883L_MAGN_Y_L			0x08
#define HMC5883L_STATUS_REG			0x09
#define HMC5883L_IDEN_A				0x10
#define HMC5883L_IDEN_B				0x11
#define HMC5883L_IDEN_C				0x12

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

Accelerometer accel;
Gyrometer gyro;
Magnetometer magn;
Pressure press;

GY86_Result IsReadyToInterfaceMPU6050()
{
//	uint8_t data;	// value to wakeup MPU6050
//
//	/**** Kiểm tra giao tiếp I2C của MPU6050 *****/
//	if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, 2, 5) != HAL_OK)
//	{
//		return MPU6050_Result_Notconnected;
//	}
//
//	/* Check who am I */
//	if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &data, 1, 500) != HAL_OK)
//	{
//		return MPU6050_Result_Notconnected;
//	}
//	/* Checking */
//	while(data != MPU6050_I_AM)
//	{
//		/* Return error */
//		return MPU6050_Result_Notconnected;
//	}
//
//	data = 0x00;
//	/* Wakeup MPU6050 */
//	if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 500) != HAL_OK)
//	{
//		return MPU6050_Result_Notconnected;
//	}
//
//	// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
//	data = 0x07;
//	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
//	// set accelerometer configuration in ACCEL_CONGIG Resister
//	// XA_ST=0, YA_ST=0,ZA_ST=0,FS_SEL=0 ->+-2g
//	data=0x00;
//	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	    uint8_t check;
	    uint8_t Data;

	    // check device ID WHO_AM_I

	    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

	    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	    {
	        // power management register 0X6B we should write all 0's to wake the sensor up
	        Data = 0;
	        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

	        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	        Data = 0x07;
	        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

	        // Set accelerometer configuration in ACCEL_CONFIG Register
	        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
	        Data = 0x00;
	        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

	        // Set Gyroscopic configuration in GYRO_CONFIG Register
	        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
	        Data = 0x00;
	        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
	        return MPU6050_Result_Notconnected;
	    }

	return GY86_Result_OK;
}
GY86_Result IsReadyToInterfaceHMC5883L()
{
	// Tắt chế độ I2C master mode
	uint8_t data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	// Bật chế độ I2C master bypass mode
	data = 0x02;
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);
////////////////////////////////////////////////////////////////////////////////////////////////////


	/**** Kiểm tra giao tiếp I2C của HMC5883L *****/
	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)HMC5883L_ADDRESS, 2, 5) != HAL_OK)
	{
		return HMC5883_Result_NotConnected;
	}
	// Thiết lập sample rate = 75Hz và số lần lấy mẫu trung bình là 8
	data = 0x78;
	if ( HAL_I2C_Mem_Write(&hi2c1, (uint16_t)HMC5883L_ADDRESS, (uint16_t)HMC5883L_CONFIG_A, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Thiết lập full scale = +/- 1.3 Gauss
	data= 0x20;
	if ( HAL_I2C_Mem_Write(&hi2c1, (uint16_t)HMC5883L_ADDRESS, (uint16_t)HMC5883L_CONFIG_B, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Bật chế độ continuous measurement
	data = 0x00;
	if ( HAL_I2C_Mem_Write(&hi2c1, (uint16_t)HMC5883L_ADDRESS, (uint16_t)HMC5883L_MODE_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}



	// Tắt chế độ I2C master bypass mode
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	// Bật chế độ I2C master mode
	data = 0x20;
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Thiết lập địa chỉ của thanh ghi cần đọc là HMC5883L
    data = HMC5883L_ADDRESS_DATASHEET | 0x80;
	if ( HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_I2C_SLV0_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Bắt đầu đọc giá trị địa chỉ thanh ghi là 0x03 (x axis)
	data = 0x03;
	if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_I2C_SLV0_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Đọc 6 thanh ghi liên tiếp trong HMC5883L
	data = 6 | 0x80;
	if ( HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_I2C_SLV0_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}
	// cho phép slave 0
	data = 1;
	if ( HAL_I2C_Mem_Write(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)0x67, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}


	return GY86_Result_OK;
}
GY86_Result ReadAllParameter()
{
	 uint8_t Rec_Data[6];

	    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

	    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

	    accel.x = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	    accel.y = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	    accel.z = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	    /*** convert the RAW values into acceleration in 'g'
	         we have to divide according to the Full scale value set in FS_SEL
	         I have configured FS_SEL = 0. So I am dividing by 16384.0
	         for more details check ACCEL_CONFIG Register              ****/

	    accel.x = accel.x / 16384.0;
	    accel.y = accel.y / 16384.0;
	    accel.z = accel.z / Accel_Z_corrector;
//	uint8_t data[20];
//
//
//	if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 20, 500) != HAL_OK)
//	{
//		return GY86_Result_Error;
//	}
//	/* Format accelerometer data */
//	// 139, 640 la gia tri offset theo cac phuong x, y do viec dat cam bien MPU6050 khong nam thang bang
//	accel.x = (int16_t)(data[0] << 8 | data[1]) ;
//	accel.y = (int16_t)(data[2] << 8 | data[3]);
//	accel.z = (int16_t)(data[4] << 8 | data[5]);
//
//	accel.x /=16384.0;
//	accel.y /=16384.0;
//	accel.z /=16384.0;
//	/* Format temperature */
//	temp = (data[6] << 8 | data[7]);

	/* Format gyroscope data */
//	gyro.x = (int16_t)(data[8] << 8 | data[9]);
//	gyro.y = (int16_t)(data[10] << 8 | data[11]);
//	gyro.z = (int16_t)(data[12] << 8 | data[13]);
//
//	gyro.x /=131.0;
//	gyro.y /=131.0;
//	gyro.z /=131.0;
//
//	magn.x   = (int16_t)(data[14] << 8 | data[15]);
//	magn.y   = (int16_t)(data[16] << 8 | data[17]);
//	magn.z   = (int16_t)(data[18] << 8 | data[19]);

	/* Return OK */
	return GY86_Result_OK;
}



