/*
 * Collectors.h
 *
 *  Created on: Feb 18, 2022
 *      Author: sujiwo
 */

#ifndef NAVIMU_COLLECTORS_H_
#define NAVIMU_COLLECTORS_H_


#include <vector>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>


namespace navimu {

class ImuCollector : public std::vector<sensor_msgs::Imu>
{
public:
ImuCollector(ros::NodeHandle &);

private:
ros::NodeHandle &rosnode;
};


class GnssCollector
{
public:
	GnssCollector(ros::NodeHandle &, const std::string &topic="fix");

	void callback(const sensor_msgs::NavSatFix&);

	bool hasInitialized() const
	{ return _hasInitialized; }

	sensor_msgs::NavSatFix getLastMessage() const;

private:
	ros::NodeHandle &rosnode;
	ros::Subscriber subsc;

	bool _hasInitialized = false;
	sensor_msgs::NavSatFix lastMsg;
};



}

#endif /* NAVIMU_COLLECTORS_H_ */
