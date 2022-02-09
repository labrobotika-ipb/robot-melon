/*
 * mpu_9250_node.cpp
 *
 *  Created on: Nov 26, 2021
 *      Author: sujiwo
 */

#include <cstdio>
#include <string>
#include <array>
#include <vector>
#include <ros/ros.h>
#include "mpu_driver.h"



using namespace std;


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mpu_9250_driver");
	ros::NodeHandle dNode("~");

	// Get parameters
	int bus_number = dNode.param<int>("bus_number", -1);
	int mpu6050_address = dNode.param<int>("mpu6050_address", 0x68);
	int sample_rate = dNode.param<int>("sample_rate", 30);
	string frame_id = dNode.param<string>("frame_id", "imu");
	string imuTopic = dNode.param<string>("imu_topic", "imu");
	string magnTopic = dNode.param<string>("magnetometer_topic", "magnetometer");
	string calibrationFile = dNode.param<string>("calibration_file", "/var/tmp/test.yml");
	float local_gravity = dNode.param<float>("local_gravity", 0);
	bool is_calibrating = bool(dNode.param<int>("is_calibrating", false));

	MpuDriver::MpuConfig driverConfig;
	driverConfig.mpu_address = mpu6050_address;
	driverConfig.local_gravity_const = local_gravity;
	driverConfig.is_calibrating = is_calibrating;

	if (calibrationFile.empty()==false and !is_calibrating) {
		driverConfig.dCalibration = MpuDriver::Calibration::loadCalibration(calibrationFile);
	}

	MpuDriver driver(bus_number, driverConfig);
	float datarate = driver.set_dlpf_frequencies(
		MpuDriver::MpuConfig::gyro_dlpf_frequency_type::F_250HZ,
		MpuDriver::MpuConfig::accel_dlpf_frequency_type::F_218HZ,
		8000.0f);
	printf("Data rate: %f\n", datarate);

	ros::Publisher imuPub, tempPub, magnetPub;
	imuPub = dNode.advertise<sensor_msgs::Imu>(imuTopic, 500);
	if (driver.hasMagnetometer())
		magnetPub = dNode.advertise<sensor_msgs::MagneticField>(magnTopic, 500);

	// XXX: find sensor data rate
	ros::Rate timer(100);

	while(ros::ok()) {
		MpuDriver::MpuData data;
		sensor_msgs::Imu imudata;
		sensor_msgs::MagneticField magn;

		imudata.header.frame_id = frame_id;

		driver.read_values(data);
		data.toRos(imudata);
		if (driver.hasMagnetometer()) {
			data.toRos(magn);
		}

		imudata.header.frame_id = frame_id;
		magn.header.frame_id = frame_id;
		imudata.header.stamp = ros::Time::now();
		magn.header.stamp = ros::Time::now();

		imuPub.publish(imudata);
		if (driver.hasMagnetometer())
			magnetPub.publish(magn);

		ros::spinOnce();
		timer.sleep();
	}

	return 0;
}
