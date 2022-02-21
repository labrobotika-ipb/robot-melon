/*
 * ImuGnssIntegrator.cpp
 *
 *  Created on: Feb 9, 2022
 *      Author: sujiwo
 */

#include <cmath>
#include <cfloat>
#include "ImuGnssIntegrator.h"


namespace navimu {


inline Matrix3f
skewmatrix(const Vector3f &S)
{
	Matrix3f Sk;
	Sk <<  0, -S[2], S[1],
		S[2],  0,   -S[0],
	   -S[1],  S[0], 0;
	return Sk;
}


ImuGnssIntegrator::ImuGnssIntegrator(ros::NodeHandle &n, const GnssCollector &collector, const std::string &imutopic) :
	rosnode(n),
	mGnssColl(collector)
{
	subsc = rosnode.subscribe(imutopic, 10, &ImuGnssIntegrator::imuUpdate, this);
	lastSolv = {Vector3f(), Vector3f(), Vector3f(), Vector3f(), ros::Time::now()};

	solutionTopic = imutopic + "/navimu";
}


ImuGnssIntegrator::~ImuGnssIntegrator()
{
	// TODO Auto-generated destructor stub
}


void
ImuGnssIntegrator::imuUpdate(const sensor_msgs::Imu &imudata)
{
	if (!mGnssColl.hasInitialized()) {
		return;
	}

	auto curGnssState = mGnssColl.getLastMessage();

}


Matrix3f
ImuGnssIntegrator::earth_rate(const float latitude)
{
	Matrix3f R;
	R << 0, sin(latitude), 0,
		-sin(latitude), 0, -cos(latitude),
		0, cos(latitude), 0;

	return R*7.2921155e-5;
}


void
ImuGnssIntegrator::radius(const float latitude, float &meridian, float &normal)
{
	double
		a = 6378137.0,                  // WGS84 Equatorial radius in meters
		e = 0.0818191908425;            // WGS84 eccentricity
	auto e2 = pow(e,2);
//	auto den = 1- (e2 *((sin(latitude))^2));
	auto den = 1- (e2 *(pow(sin(latitude),2)));
	meridian = a * ((1-e2) / (pow(den, 1.5)));
	normal = a / sqrt(den);
}


Matrix3f ImuGnssIntegrator::transport_rate(const float latitude,
		const float V_north, float V_east,
		float altitude)
{
	float meridian, normal;
	radius(latitude, meridian, normal);

	Vector3f om_en_n;
	om_en_n[0] = V_east / (normal + altitude);						// North
	om_en_n[1] = -(V_north / (meridian + altitude));				// East
	om_en_n[2] = -(V_east * tan(latitude) / (normal + altitude));	// Down

	return skewmatrix(om_en_n);
}


Vector3f
ImuGnssIntegrator::gravity(const float latitude, const float altitude)
{
	//   RM, meridian radius of curvature (North-South)(m).
	//   RN, normal radius of curvature (East-West) (m).

	// Parameters
	float
		RN = 6378137,               // WGS84 Equatorial radius in meters
		RM = 6356752.31425,         // WGS84 Polar radius in meters
		e = 0.0818191908425,        // WGS84 eccentricity
		f = 1 / 298.257223563,      // WGS84 flattening
		mu = 3.986004418E14,        // WGS84 Earth gravitational constant (m^3 s^-2)
		omega_ie_n = 7.292115E-5;   // Earth rotation rate (rad/s)

	// Calculate surface gravity using the Somigliana model, (2.134)
	float sinl2 = pow(sin(latitude), 2);
	float g_0 = 9.7803253359 * (1 + 0.001931853 * sinl2) / sqrt(1 - pow(e,2) * sinl2);

	Vector3f gn;

	// Calculate north gravity using (2.140)
	gn[0] = -8.08E-9 * altitude * sin(2 * latitude);
	// East gravity is zero
	gn[1] = 0;
	// Calculate down gravity using (2.139)
	gn[2] = g_0 * (1 - (2 / RN) * (1 + f * (1 - 2 * sinl2) +
	    (pow(omega_ie_n,2) * pow(RN,2) * RM / mu)) * altitude + (3 * pow(altitude,2) / pow(RN,2)));

	return gn;
}



}	// namespace navimu
