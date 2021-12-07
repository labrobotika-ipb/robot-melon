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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <Eigen/Core>

extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "smbus.h"
}

#define M_G 9.78f

using namespace std;

class mpu9250_driver
{
public:

	typedef Eigen::Vector3f Vector3;
	typedef Eigen::Vector4f Vector4;
	typedef Eigen::Matrix4f Matrix4;

	struct MpuData {
		Vector3
			accel = Vector3::Zero(),
			gyro = Vector3::Zero(),
			magneticField = Vector3::Zero();
		float temperature = 0;

		void toRos (sensor_msgs::Imu &imudata)
		{
			imudata.linear_acceleration.x = accel.x();
			imudata.linear_acceleration.y = accel.y();
			imudata.linear_acceleration.z = accel.z();

			imudata.angular_velocity.x = gyro.x();
			imudata.angular_velocity.y = gyro.y();
			imudata.angular_velocity.z = gyro.z();
		}

		void toRos (sensor_msgs::MagneticField &magn)
		{
			magn.magnetic_field.x = magneticField.x();
			magn.magnetic_field.y = magneticField.y();
			magn.magnetic_field.z = magneticField.z();
		}

		void toRos (sensor_msgs::Temperature &temp)
		{
			temp.temperature = temperature;
		}

		void toRos (sensor_msgs::Imu &imu, sensor_msgs::MagneticField &magn, sensor_msgs::Temperature &temp)
		{
			toRos(imu); toRos(magn); toRos(temp);
		}
	};

	struct MpuConfig {
		enum AccelSensitivity {
			FSR_2G = 2,
			FSR_4G = 4,
			FSR_8G = 8,
			FSR_16G = 16
		};

		MpuConfig() {}
		float
			accel_sensitivity = FSR_2G,	// in g (9.81ms-2)
			gyro_sensitivity = 250.0;	// in degree-per-sec
		float
			local_gravity_const = M_G;
		Matrix4 calibration = Matrix4::Identity();
	};

	mpu9250_driver(int bus_num, const MpuConfig &cfg=MpuConfig()):
		devcfg(cfg)
	{
		initialize(bus_num);
	}

	~mpu9250_driver()
	{
		if (mHasMagnetometer) {
			write_register(ak8963_fd, AK8963_CNTL, 0);
			close(ak8963_fd);
		}
		close(mpu9250_fd);
	}

	bool read_values(MpuData &data)
	{
		// Accelerator readings are in G
		data.accel.x() = devcfg.accel_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, ACCEL_XOUT_H))) /32768.0f;
		data.accel.y() = devcfg.accel_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, ACCEL_YOUT_H))) /32768.0f;
		data.accel.z() = devcfg.accel_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, ACCEL_ZOUT_H))) /32768.0f;
		data.temperature = 0.0;
		data.gyro.x() = devcfg.gyro_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, GYRO_XOUT_H))) /32768.0f;
		data.gyro.y() = devcfg.gyro_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, GYRO_YOUT_H))) /32768.0f;
		data.gyro.z() = devcfg.gyro_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, GYRO_ZOUT_H))) /32768.0f;

		if (mHasMagnetometer) {
			auto hx = read_raw_words(ak8963_fd, HXH-1, false);
			auto hy = read_raw_words(ak8963_fd, HYH-1, false);
			auto hz = read_raw_words(ak8963_fd, HZH-1, false);
			auto bstat = static_cast<uint8_t>(read_register(ak8963_fd, AK8963_ST2));

			if (bstat & 0b1000) {
				// overflow
				data.magneticField.x() = std::numeric_limits<float>::quiet_NaN();
				data.magneticField.y() = std::numeric_limits<float>::quiet_NaN();
				data.magneticField.z() = std::numeric_limits<float>::quiet_NaN();
			}

			else {
				// 16 bit or 14 bit resolution
				float resolution = (bstat & 0b10000 ? 32768.0f : 17778.0f);
				data.magneticField.x() = static_cast<float>(static_cast<short>(hx)) / resolution;
				data.magneticField.y() = static_cast<float>(static_cast<short>(hy)) / resolution;
				data.magneticField.z() = static_cast<float>(static_cast<short>(hz)) / resolution;
			}
		}

		return true;
	}

	bool hasMagnetometer() const
	{ return mHasMagnetometer; }

	const uint
		//MPU6050 Registers
		MPU6050_ADDR 		= 0x68,
		MPU6050_WHO_AM_I 	= 0x75,
		PWR_MGMT_1   		= 0x6B,
		SMPLRT_DIV			= 0x19,
		CONFIG				= 0x1A,
		GYRO_CONFIG			= 0x1B,
		ACCEL_CONFIG		= 0x1C,
		INT_PIN_CFG  = 0x37,
		INT_ENABLE   = 0x38,
		ACCEL_XOUT_H = 0x3B,
		ACCEL_YOUT_H = 0x3D,
		ACCEL_ZOUT_H = 0x3F,
		TEMP_OUT_H   = 0x41,
		GYRO_XOUT_H  = 0x43,
		GYRO_YOUT_H  = 0x45,
		GYRO_ZOUT_H  = 0x47,
		//AK8963 registers
		AK8963_ADDR   = 0x0C,
		AK8963_ST1    = 0x02,
		HXH          = 0x04,
		HYH          = 0x06,
		HZH          = 0x08,
		AK8963_ST2   = 0x09,
		AK8963_CNTL  = 0x0A,
		AK8963_ASAX = 0x10;

protected:
	static bool read_registers(int dev_fd, uint register_address, uint num_bytes, vector<uint8_t> &buffer)
	{
		buffer.resize(num_bytes, 0);
		auto s = i2c_smbus_read_block_data(dev_fd, (uint8_t)register_address, buffer.data());
		if (s<0)
			return false;
		else {
			buffer.resize(s);
			return true;
		}
	}

	static int8_t read_register(int dev_fd, uint register_address)
	{
		auto s = i2c_smbus_read_byte_data(dev_fd, (__u8)register_address);
		if (errno<0)
			throw runtime_error("Unable to read register");
		else return s;
	}

	static bool write_register(int dev_fd, uint register_address, int8_t w)
	{
		auto s = i2c_smbus_write_byte_data(dev_fd, (uint8_t)register_address, (uint8_t)w);
		return true;
	}

	static uint16_t read_raw_words(int dev_fd, uint register_address, bool isBe=true)
	{
		auto vl = i2c_smbus_read_word_data(dev_fd, register_address);
		if (isBe)
			return be16toh(vl);
		else
			return le16toh(vl);
	}

	void initialize(int bus_num)
	{
		string devpath("/dev/i2c-");
		devpath += to_string(bus_num);
		mpu9250_fd = open(devpath.c_str(), O_RDWR);
		if (mpu9250_fd<0)
			throw std::runtime_error("Unable to open I2C");

		// Enable device
		ioctl(mpu9250_fd, I2C_SLAVE, MPU6050_ADDR);

		// Check who am i
		auto i=read_register(mpu9250_fd, MPU6050_WHO_AM_I);
		if (i==0x71)
			mHasMagnetometer = true;
		else {
			if (i!=0x68)
				throw runtime_error("MPU6050-compatible not found");
			mHasMagnetometer = false;
		}

		// Initialize Accelerometer & gyro first
		write_register(mpu9250_fd, PWR_MGMT_1, 0x80);
		usleep(5000);
		write_register(mpu9250_fd, PWR_MGMT_1, 0x01);
		write_register(mpu9250_fd, INT_PIN_CFG, 0x22);
		write_register(mpu9250_fd, INT_ENABLE, 1);
		write_register(mpu9250_fd, PWR_MGMT_1, 0x00);

		if (mHasMagnetometer) {
			ak8963_fd = open(devpath.c_str(), O_RDWR);
			ioctl(ak8963_fd, I2C_SLAVE, AK8963_ADDR);
			write_register(ak8963_fd, AK8963_CNTL, 0x16);
		}
	}

private:
	int
		mpu9250_fd = -1,
		ak8963_fd = -1;
	bool mHasMagnetometer = false;

	// Default full-scale range
	float
		accel_scale = 2.0f,			// 2G
		gyro_scale = 250.f;			// 250 deg/s

	const MpuConfig devcfg;

};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mpu_9250_driver");
	ros::NodeHandle dNode;

	// Get parameters
	int bus_number = dNode.param<int>("bus_number", 1);
	int sample_rate = dNode.param<int>("hz", 30);
	string frame_id = dNode.param<string>("frame_id", "imu");
	string imuTopic = dNode.param<string>("imu_topic", "imu");
	string magnTopic = dNode.param<string>("magnetometer_topic", "magnetometer");

	mpu9250_driver driver(bus_number);

	ros::Publisher imuPub, tempPub, magnetPub;
	imuPub = dNode.advertise<sensor_msgs::Imu>(imuTopic, 500);
	if (driver.hasMagnetometer())
		magnetPub = dNode.advertise<sensor_msgs::MagneticField>(magnTopic, 500);

	// XXX: find sensor data rate
	ros::Rate timer(2);

	while(ros::ok()) {
		mpu9250_driver::MpuData data;
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
