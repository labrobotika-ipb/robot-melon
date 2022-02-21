/*
 * node.cpp
 *
 *  Created on: Feb 9, 2022
 *      Author: sujiwo
 */


#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "Collectors.h"
#include "ImuGnssIntegrator.h"
#include "Imu.h"


using namespace std;


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "");
	ros::NodeHandle n;

//	navimu::ImuCollector imu(n);
	navimu::GnssCollector gnss(n);

	navimu::ImuGnssIntegrator navsys(n, gnss);

	ros::spin();

	return 0;
}
