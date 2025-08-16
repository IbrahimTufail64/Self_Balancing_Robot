
/*
 * MPU9250.c
 *
 *  Created on: Jul 14, 2025
 *      Author: ibrah
 */

#include "MPU9250.h"

	  uint8_t MPU9250_ADDR =  (0x68 << 1);
	  uint8_t AK8963_ADDR = (0x0C << 1);  // always 0x18
	  float mag_sens_adj [3];

void init_MPU( I2C_HandleTypeDef * hi2c){

	 uint8_t CONFIG_DATA       = 0x03;
	 uint8_t GYRO_CONFIG_DATA  = 0x00;
	 uint8_t ACC_CONFIG_DATA   = 0x00;
	 uint8_t PWR_CONFIG_DATA   = 0x00;

	 // Device initialization
	  HAL_StatusTypeDef ret =  HAL_I2C_IsDeviceReady(hi2c, (DEVICE_ADDRESS << 1), 2, 50);
//  	  if (ret == HAL_OK) {
//  		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // debug LED
//  	  }

	  //Device Configuration

	  ret =  HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), CONFIG_REG, 1 , &CONFIG_DATA, 1, 50);
	  if(ret == HAL_OK){
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); // debug led
	  }

	  // Register 27 – Gyroscope Configuration

	  ret =  HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), GYRO_CONFIG_REG, 1 , &GYRO_CONFIG_DATA, 1, 50);
	  if(ret == HAL_OK){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); // debug led
	  }

	  // Register 28 – Accelerometer Configuration

	  ret =  HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), ACC_CONFIG_REG, 1 , &ACC_CONFIG_DATA, 1, 50);
	  if(ret == HAL_OK){
	//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); // debug led
	  }


	  // Register 107 – power management Configuration

	  ret =  HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), PWR_CONFIG_REG, 1 , &PWR_CONFIG_DATA, 1, 50);
	  if(ret == HAL_OK){
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); // debug led
	  }








//	  uint8_t i2c_tx_data = 0x80;
//	  ret = HAL_I2C_Mem_Write_DMA(hi2c, AK8963_ADDRESS, 0x37, 1, &i2c_tx_data, 1);
//  	  if (ret == HAL_OK) {
//  		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // debug LED
//  	  }
//	  uint8_t i2c_rx_data = 0;
//	  ret = HAL_I2C_Mem_Read(hi2c, AK8963_ADDRESS, AK8963_WHO_AM_I, 1, &i2c_rx_data, 1,25);
//
//	  //reset the AK8963
//	  i2c_tx_data = 0x01;
//	  ret = HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS, 0x0B, 1, &i2c_tx_data, 1,25);
//
//	  //select 16 bit data output mode and select fused ROM access mode
//	  	i2c_tx_data = 0x1F;
//	  ret = HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS, 0x0A, 1, &i2c_tx_data, 1,25);
//
//	  HAL_Delay(10);
//
//	  //HAL_I2C_Mem_Read_DMA(&hi2c1, AK8963_ADDRESS, AK8963_ASAX, MEM_ADD_SIZE, MPU9250.raw_ASA, 3);
//
//		//select power down mode before entering the continuous measurement mode
//		i2c_tx_data = 0x16;
//		ret = HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS, 0x0A, 1, &i2c_tx_data, 1,25);
//		HAL_Delay(10);



//	  // Enable I2C Master mode on MPU9250
//	  uint8_t bypass_enable = 0x02; // BYPASS_EN bit = 1
//	  HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), 0x37, 1, &bypass_enable, 1, 100);
//
//	  uint8_t who_am_i;
//	  ret =  HAL_I2C_Mem_Read(hi2c, (DEVICE_ADDRESS << 1), 0x00, 1, &who_am_i, 1, 100);
//	  if(ret == HAL_OK){
//		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // debug LED
//	  }
//
//	  uint8_t reset = 0x01;
//	  HAL_I2C_Mem_Write(hi2c,  (DEVICE_ADDRESS << 1), 0x0B, 1, &reset, 1, 100);
//	  HAL_Delay(10);
//
//	  uint8_t mode = 0x16;  // 0x10 (16-bit output) + 0x06 (continuous mode 2)
//	  HAL_I2C_Mem_Write(hi2c,  (DEVICE_ADDRESS << 1), 0x0A, 1, &mode, 1, 100);
//	  HAL_Delay(10);




	  uint8_t data;

	    // Enable I2C bypass mode (so we can talk to AK8963 directly)
	    data = 0x02;
	    HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, 0x37, 1, &data, 1, HAL_MAX_DELAY);

	    // Power down magnetometer
	    data = 0x00;
	    HAL_I2C_Mem_Write(hi2c, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
	    HAL_Delay(10);

	    // Enter Fuse ROM mode
	    data = 0x0F;
	    HAL_I2C_Mem_Write(hi2c, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
	    HAL_Delay(10);

	    // Read ASA calibration values
	    uint8_t asa[3];
	    HAL_I2C_Mem_Read(hi2c, AK8963_ADDR, 0x10, 1, asa, 3, HAL_MAX_DELAY);

	    mag_sens_adj[0] = ((float)(asa[0] - 128) / 256.0f) + 1.0f;
	    mag_sens_adj[1] = ((float)(asa[1] - 128) / 256.0f) + 1.0f;
	    mag_sens_adj[2] = ((float)(asa[2] - 128) / 256.0f) + 1.0f;

	    // Power down again
	    data = 0x00;
	    HAL_I2C_Mem_Write(hi2c, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
	    HAL_Delay(10);

	    // Set to Continuous Measurement Mode 2 with 16-bit output
	    data = 0x16;
	    HAL_I2C_Mem_Write(hi2c, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
	    HAL_Delay(10);





}


void get_values_MPU(I2C_HandleTypeDef * hi2c, raw_readings * readings)
{
    const float gyro_sensitivity = 0.0076293f; // ±250dps range
    const float accel_sensitivity = 0.000061f; // ±2g range

    // Burst read from 0x3B to 0x48 (14 bytes: acc[6], temp[2], gyro[6])
    uint8_t imu_data[14] = {0};
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, (DEVICE_ADDRESS << 1), 0x3B, 1, imu_data, 14, 50);

    if (ret == HAL_OK) {
        // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // debug LED
    }

    // Accelerometer data
    int16_t x_acc = (int16_t)((imu_data[0] << 8) | imu_data[1]);
    int16_t y_acc = (int16_t)((imu_data[2] << 8) | imu_data[3]);
    int16_t z_acc = (int16_t)((imu_data[4] << 8) | imu_data[5]);

    readings->x_acc_g = x_acc * accel_sensitivity;
    readings->y_acc_g = y_acc * accel_sensitivity;
    readings->z_acc_g = z_acc * accel_sensitivity;

    // Gyroscope data
    int16_t x_gyro = (int16_t)((imu_data[8] << 8) | imu_data[9]);
    int16_t y_gyro = (int16_t)((imu_data[10] << 8) | imu_data[11]);
    int16_t z_gyro = (int16_t)((imu_data[12] << 8) | imu_data[13]);

    readings->x_gyro_degree = x_gyro * gyro_sensitivity;
    readings->y_gyro_degree = y_gyro * gyro_sensitivity;
    readings->z_gyro_degree = z_gyro * gyro_sensitivity;

    // MAGNETOMETER READING - FIXED VERSION
    uint8_t st1 = 0;
    ret = HAL_I2C_Mem_Read(hi2c, AK8963_ADDR, 0x02, 1, &st1, 1, 100);

    if (ret == HAL_OK && (st1 & 0x01)) // Check communication and data ready
    {
        uint8_t mag_data[7] = {0}; // Read HXL, HXH, HYL, HYH, HZL, HZH, ST2
        ret = HAL_I2C_Mem_Read(hi2c, AK8963_ADDR, 0x03, 1, mag_data, 7, 100);

        if (ret == HAL_OK)
        {
            uint8_t st2 = mag_data[6]; // ST2 is the 7th byte

            if (!(st2 & 0x08)) // Check for overflow (HOFL bit)
            {
                // AK8963 uses little-endian format: Low byte first, then High byte
                int16_t x_mag_raw = (int16_t)(mag_data[1] << 8 | mag_data[0]); // HXH | HXL
                int16_t y_mag_raw = (int16_t)(mag_data[3] << 8 | mag_data[2]); // HYH | HYL
                int16_t z_mag_raw = (int16_t)(mag_data[5] << 8 | mag_data[4]); // HZH | HZL

                // Apply sensitivity adjustment (from ASA values read during init)
                readings->x_mag = x_mag_raw * mag_sens_adj[0];
                readings->y_mag = y_mag_raw * mag_sens_adj[1];
                readings->z_mag = z_mag_raw * mag_sens_adj[2];

                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // debug LED for successful read
            }
            else
            {
                // Overflow occurred - data is invalid
                readings->x_mag = 0;
                readings->y_mag = 0;
                readings->z_mag = 0;
            }
        }
    }
    else
    {
        // No new data available or communication error
        readings->x_mag = 0;
        readings->y_mag = 0;
        readings->z_mag = 0;
    }
}

// Ignore these for now
void mpu_write_ak8963_register(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val) {
    HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), 0x63, 1, &val, 1, 100);     // Set data
    HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), 0x26, 1, &reg, 1, 100);     // Set AK reg
    uint8_t addr = AK8963_ADDRESS & 0x7F;                                // Write mode
    HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), 0x25, 1, &addr, 1, 100);    // Set AK addr
    uint8_t ctrl = 0x81; // Enable, 1 byte
    HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), 0x27, 1, &ctrl, 1, 100);    // Start transfer
    HAL_Delay(10);
}

void mpu_read_ak8963_registers(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint8_t len) {
    HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), 0x26, 1, &reg, 1, 100);     // Set AK reg
    uint8_t addr = (DEVICE_ADDRESS << 1) | 0x80;                                // Read mode
    HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), 0x25, 1, &addr, 1, 100);    // Set AK addr
    uint8_t ctrl = 0x80 | len; // Enable, len bytes
    HAL_I2C_Mem_Write(hi2c, (DEVICE_ADDRESS << 1), 0x27, 1, &ctrl, 1, 100);    // Start transfer
    HAL_Delay(10);
    HAL_I2C_Mem_Read(hi2c, (DEVICE_ADDRESS << 1), 0x49, 1, buf, len, 100);     // Read from EXT_SENS_DATA
}




