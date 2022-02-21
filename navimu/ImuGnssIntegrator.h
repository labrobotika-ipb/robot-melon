/*
 * ImuGnssIntegrator.h
 *
 *  Created on: Feb 9, 2022
 *      Author: sujiwo
 */

#ifndef NAVIMU_IMUGNSSINTEGRATOR_H_
#define NAVIMU_IMUGNSSINTEGRATOR_H_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <mutex>
#include <thread>
#include <Eigen/Dense>
#include "Collectors.h"

using Eigen::Vector3f;
using Eigen::Matrix3f;


namespace navimu {


struct Solution
{
	Vector3f
		attitude,
		velocity,
		gravity,
		position;
	ros::Time time;
};


class ImuGnssIntegrator {
public:

	struct Config {

	};

	ImuGnssIntegrator(ros::NodeHandle &n,
			const GnssCollector &,
			const std::string &imuTopic="imu");
	virtual ~ImuGnssIntegrator();

//	void gnssUpdate(const sensor_msgs::NavSatFix &fix);
	void imuUpdate(const sensor_msgs::Imu &imudata);

	static Matrix3f earth_rate(const float latitude);
	static Matrix3f transport_rate(
		const float latitude,
		const float V_north, float V_east,
		float altitude);
	static void radius(const float latitude, float &meridian, float &normal);
	static Vector3f gravity(const float latitude, const float altitude);

protected:
	ros::NodeHandle &rosnode;
	ros::Subscriber subsc;
	const GnssCollector &mGnssColl;

	Solution lastSolv;

	std::string solutionTopic;
};

}

#endif /* NAVIMU_IMUGNSSINTEGRATOR_H_ */
