/*
 * Imu.h
 *
 *  Created on: Feb 9, 2022
 *      Author: sujiwo
 */

#ifndef NAVIMU_IMU_H_
#define NAVIMU_IMU_H_

#include <Eigen/Core>


namespace navimu {


using Eigen::Vector3f;
using Eigen::Matrix3f;

class Imu {
public:
	Imu();
	virtual ~Imu();

	// Utility functions

	static float magnetic_compass(const Vector3f &mag_reading,
		float dec, float inc,
		float roll, float pitch,
		const Matrix3f &nav_to_body=Matrix3f::Identity());

protected:
	Vector3f
		arw,
		vrw,
		gyro_std,
		accel_std,
		gyro_stbias,
		accel_stbias,
		gyro_dybias,
		accel_dybias,
		gyro_correl_time,
		accel_correl_time,
		gb_psd,
		ab_psd,
		ini_align,
		ini_align_err;
	float
		freq;
};

} /* namespace navimu */

#endif /* NAVIMU_IMU_H_ */
