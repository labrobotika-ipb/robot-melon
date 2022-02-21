/*
 * Imu.cpp
 *
 *  Created on: Feb 9, 2022
 *      Author: sujiwo
 */

#include <cmath>
#include "Imu.h"

namespace navimu {

Imu::Imu() {
	// TODO Auto-generated constructor stub

}

Imu::~Imu() {
	// TODO Auto-generated destructor stub
}


float Imu::magnetic_compass(const Vector3f &mag_reading,
		float dec, float inc,
		float roll, float pitch,
		const Matrix3f &nav_to_body)
{
	auto B = mag_reading.norm();
	Vector3f D = {cos(dec)*cos(inc), sin(dec)*cos(inc), sin(inc)};
	auto m_b = nav_to_body * D * B;

	auto x = m_b[1] * cos(roll) + m_b[2] * sin(roll);
	auto y = m_b[0]*cos(pitch) + m_b[1]*sin(roll)*sin(pitch) + m_b[2]*cos(roll)*sin(pitch);

	auto yawm = atan2(x, y) + dec;
	// XXX: may need to be corrected
	return yawm;
}


} /* namespace navimu */


