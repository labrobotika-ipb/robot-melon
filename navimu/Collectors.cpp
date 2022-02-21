/*
 * Collectors.cpp
 *
 *  Created on: Feb 18, 2022
 *      Author: sujiwo
 */


#include <cfloat>
#include <cmath>
#include "Collectors.h"


namespace navimu {


ImuCollector::ImuCollector(ros::NodeHandle &n):
	rosnode(n)
{
	std::vector<sensor_msgs::Imu>();
}


GnssCollector::GnssCollector(ros::NodeHandle &n, const std::string &topic):
	rosnode(n)
{
	subsc = rosnode.subscribe(topic, 10, &GnssCollector::callback, this);
}


void GnssCollector::callback(const sensor_msgs::NavSatFix &msg)
{
	if (isnan(msg.latitude) or isnan(msg.longitude) or isnan(msg.altitude))
		return;

	_hasInitialized = true;
	lastMsg = msg;
}


sensor_msgs::NavSatFix
GnssCollector::getLastMessage() const
{
	return lastMsg;
}




}	// namespace navimu
