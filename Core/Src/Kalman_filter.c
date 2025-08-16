/*
 * Kalman_filter.c
 *
 *  Created on: Jul 16, 2025
 *      Author: ibrah
 */


#include "Kalman_filter.h"
#include "MPU9250.h"



void Complementary_filter(raw_readings * readings,float * pitch, float * roll){




	  float roll_acc = atan2(readings->y_acc_g, readings->z_acc_g) * 180.0f / PI;
	  float pitch_acc = atan2(-(readings->x_acc_g), sqrt(readings->y_acc_g * readings->y_acc_g + readings->z_acc_g * readings->z_acc_g)) * 180.0f / PI;

	  *roll = alpha * (*roll + readings->x_gyro_degree * dt) + (1 - alpha) * roll_acc;
	  *pitch = alpha * (*pitch + readings->y_gyro_degree * dt) + (1 - alpha) * pitch_acc;
}
