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
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "smbus.h"


using namespace std;

class mpu9250_driver
{
public:
	mpu9250_driver(int bus_num)
	{
		string devpath("/dev/i2c-");
		devpath += to_string(bus_num);
		mpu9250_fd = open(devpath.c_str(), O_RDWR);
		if (mpu9250_fd<0)
			throw std::runtime_error("Unable to open I2C");

		// Enable device
		ioctl(mpu9250_fd, I2C_SLAVE, MPU6050_ADDR);

		// Check who am i
		if (read_register(mpu9250_fd, MPU6050_WHO_AM_I) != 0x71)
			throw runtime_error("MPU6050-compatible not found");

		// Initialize.
		// XXX: Should delay per-line execution by 1 ms
		write_register(mpu9250_fd, PWR_MGMT_1, 0x80);
		write_register(mpu9250_fd, PWR_MGMT_1, 0x00);
		write_register(mpu9250_fd, PWR_MGMT_1, 0x01);
		write_register(mpu9250_fd, CONFIG, 0x00);
		write_register(mpu9250_fd, GYRO_CONFIG, 0b00000);
		write_register(mpu9250_fd, ACCEL_CONFIG, 0b00000);
		write_register(mpu9250_fd, INT_PIN_CFG, 0x22);
		write_register(mpu9250_fd, INT_ENABLE, 1);
	}

	~mpu9250_driver()
	{
		close(mpu9250_fd);
	}

	bool read_accel_values(array<float,3> &cur_accel_values)
	{
		vector<uint8_t> accel_buf;
		read_registers(mpu9250_fd, ACCEL_XOUT_H, 14, accel_buf);

	}

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
		auto s = i2c_smbus_read_byte_data(dev_fd, (uint8_t)register_address);
		if (errno<0)
			throw runtime_error("Unable to read register");
		else return s;
	}

	static bool write_register(int dev_fd, uint register_address, int8_t w)
	{
		auto s = i2c_smbus_write_byte_data(dev_fd, (uint8_t)register_address, (uint8_t)w);
		return true;
	}

private:
	int mpu9250_fd = -1;
};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mpu_9250_driver");
	ros::NodeHandle dNode;

	mpu9250_driver driver(1);

	return 0;
}
