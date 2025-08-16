/*
 * Kalman_filter.h
 *
 *  Created on: Jul 16, 2025
 *      Author: ibrah
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_
#include "MPU9250.h"

void Complementary_filter(raw_readings * readings,float * pitch, float * roll);


#endif /* INC_KALMAN_FILTER_H_ */
