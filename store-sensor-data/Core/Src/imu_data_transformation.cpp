/*
 * imu_data_transformation.cpp
 *
 *  Created on: 20 Mar 2021
 *      Author: dominikpiorkowski
 */

//Converts captured FLOAT accelerometer data into 2 * UINT8 variables which can be stored in external Flash memory
//uint8_t* accel_serialise(float accel)
//{
//	uint8_t accel_int_8[2];
//	uint16_t accel_int_16;
//
//	accel = accel + 16;
//	accel = accel * 1000;
//
//	accel_int_16 = static_cast<uint16_t>(accel);
//	accel_int_8[0] = accel_int_16 & 0xff;
//	accel_int_8[1] = (accel_int_16 >> 8);
//
//	return accel_int_8;
//}
//
////Converts stored in Flash memory data into accelerometer data format in FLOAT
//float accel_parse(uint8_t *accel)
//{
//	float accel_float;
//	uint16_t accel_int_16;
//
//	accel_int_16 = (accel[1] << 8);
//	accel_int_16 = accel_int_16 | accel[0];
//
//	accel_float = static_cast<float>(accel_int_16);
//	accel_float = accel_float / 1000;
//	accel_float = accel_float - 16;
//
//	return accel_float;
//}
