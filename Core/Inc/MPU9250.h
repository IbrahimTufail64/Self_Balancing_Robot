/*
 * MPU9250.h
 *
 *  Created on: Jul 14, 2025
 *      Author: ibrah
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include <stdio.h>
#include "stm32h7xx_hal.h"

#define DEVICE_ADDRESS 0b1101000
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACC_CONFIG_REG 0x1C
#define PWR_CONFIG_REG 107

#define AK8963_WHO_AM_I  0x00

#define alpha 0.98f
#define dt  0.01f
#define PI  3.14159265f

#define AK8963_ADDRESS   0x0C
#define MPU9250_ADDRESS  (0x68 << 1)

typedef struct {
    	  float x_acc_g;
    	  float y_acc_g;
    	  float z_acc_g;
    	  float x_gyro_degree;
    	  float y_gyro_degree;
    	  float z_gyro_degree;
    	  float x_mag;
    	  float y_mag;
    	  float z_mag;
      } raw_readings;



void init_MPU( I2C_HandleTypeDef * hi2c);

void get_values_MPU(I2C_HandleTypeDef * hi2c,raw_readings * readings);

void mpu_write_ak8963_register(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val);

void mpu_read_ak8963_registers(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint8_t len);
//
//void get_values_MPU(I2C_HandleTypeDef * hi2c,float *x_acc_g, float *y_acc_g, float *z_acc_g,
//                    float *x_gyro_deg, float *y_gyro_deg, float *z_gyro_deg);


#endif /* INC_MPU9250_H_ */
