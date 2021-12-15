/*
 * mpu_driver.cpp
 *
 *  Created on: Dec 7, 2021
 *      Author: sujiwo
 */

#include "yaml-cpp/yaml.h"
#include "mpu_driver.h"

extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "smbus.h"
}

using namespace std;

MpuDriver::MpuDriver(int bus_num, const MpuConfig &cfg) :
	devcfg(cfg)
{
	initialize(bus_num);
}


MpuDriver::~MpuDriver()
{
	if (mHasMagnetometer) {
		write_register(ak8963_fd, AK8963_CNTL, 0);
		close(ak8963_fd);
	}
	close(mpu9250_fd);
}


MpuDriver::Calibration MpuDriver::Calibration::loadCalibration(const std::string &calibFile)
{
	Calibration cl;

	YAML::Node calibnode = YAML::LoadFile(calibFile);
	auto gyro = calibnode["gyro"];
	auto accel = calibnode["accel"];

	cl.gyro.x() = gyro[0].as<float>();
	cl.gyro.y() = gyro[1].as<float>();
	cl.gyro.z() = gyro[2].as<float>();

	for (int i=0; i<3; ++i) {
		cl.accel(i,0) = accel[i][0].as<float>();
		cl.accel(i,1) = accel[i][1].as<float>();
	}

	return cl;
}


bool MpuDriver::read_values(MpuData &data)
{
	// Accelerator default readings are in G
	data.accel.x() = devcfg.accel_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, ACCEL_XOUT_H))) /32768.0f;
	data.accel.y() = devcfg.accel_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, ACCEL_YOUT_H))) /32768.0f;
	data.accel.z() = devcfg.accel_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, ACCEL_ZOUT_H))) /32768.0f;
	if (devcfg.is_calibrating==false) {
		// To m/s2 when not calibrating (default), else return value in G
		data.accel = data.accel.cwiseProduct(devcfg.dCalibration.accel.col(0)) + devcfg.dCalibration.accel.col(1);
		data.accel *= devcfg.local_gravity_const;
	}
	data.temperature = 0.0;

	data.gyro.x() = devcfg.gyro_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, GYRO_XOUT_H))) /32768.0f;
	data.gyro.y() = devcfg.gyro_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, GYRO_YOUT_H))) /32768.0f;
	data.gyro.z() = devcfg.gyro_sensitivity * static_cast<float>(static_cast<short>(read_raw_words(mpu9250_fd, GYRO_ZOUT_H))) /32768.0f;
	// to rad/s, when not calibrating
	if (devcfg.is_calibrating==false) {
		data.gyro = (data.gyro - devcfg.dCalibration.gyro) * M_PI/180.0;
	}

	if (mHasMagnetometer) {
		auto hx = read_raw_words(ak8963_fd, HXH-1, false);
		auto hy = read_raw_words(ak8963_fd, HYH-1, false);
		auto hz = read_raw_words(ak8963_fd, HZH-1, false);
		// XXX: Move this outside critical function
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


bool MpuDriver::read_registers(int dev_fd, uint register_address, uint num_bytes, std::vector<uint8_t> &buffer)
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


int8_t MpuDriver::read_register(int dev_fd, uint register_address)
{
	auto s = i2c_smbus_read_byte_data(dev_fd, (__u8)register_address);
	if (errno<0)
		throw runtime_error("Unable to read register");
	else return s;
}


bool MpuDriver::write_register(int dev_fd, uint register_address, int8_t w)
{
	auto s = i2c_smbus_write_byte_data(dev_fd, (uint8_t)register_address, (uint8_t)w);
	return true;
}


uint16_t MpuDriver::read_raw_words(int dev_fd, uint register_address, bool isBigEndian)
{
	auto vl = i2c_smbus_read_word_data(dev_fd, register_address);
	if (isBigEndian)
		return be16toh(vl);
	else
		return le16toh(vl);
}


void MpuDriver::initialize(int bus_num)
{
	string devpath("/dev/i2c-");
	devpath += to_string(bus_num);
	mpu9250_fd = open(devpath.c_str(), O_RDWR);
	if (mpu9250_fd<0)
		throw std::runtime_error("Unable to open I2C");

	// Enable device
	ioctl(mpu9250_fd, I2C_SLAVE, devcfg.mpu_address);

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


float MpuDriver::set_dlpf_frequencies(
		MpuConfig::gyro_dlpf_frequency_type gyro_frequency,
		MpuConfig::accel_dlpf_frequency_type accel_frequency,
		float max_sample_rate)
{
    // Read the current configuration for gyro/temp.
    unsigned char gyro_configuration = read_register(mpu9250_fd, CONFIG);
    // Clear the DLPF_CFG field (0:2)
    gyro_configuration &= 0xF8;
    // Set the DLPF_CFG field.
    gyro_configuration |= static_cast<unsigned char>(gyro_frequency);
    // Write new configuration.
    write_register(mpu9250_fd, CONFIG, gyro_configuration);

    // Set up the new configuration for the accel.
    // FCHOICE = 0b1, but needs to be specified as inverse (0b0).
    write_register(mpu9250_fd, ACCEL_CONFIG_2, static_cast<unsigned char>(accel_frequency));

    // Calculate new sample rate.

    // Get dlpf/internal frqeuencies for both gyro and accel.
    unsigned int gyro_dlpf_frequency = 0;
    unsigned int gyro_internal_frequency = 0;
    // These values populated from data sheet.
    switch(gyro_frequency)
    {
		case MpuConfig::gyro_dlpf_frequency_type::F_5HZ:
		{
			gyro_dlpf_frequency = 5;
			gyro_internal_frequency = 1000;
			break;
		}
		case MpuConfig::gyro_dlpf_frequency_type::F_10Hz:
		{
			gyro_dlpf_frequency = 10;
			gyro_internal_frequency = 1000;
			break;
		}
		case MpuConfig::gyro_dlpf_frequency_type::F_20Hz:
		{
			gyro_dlpf_frequency = 20;
			gyro_internal_frequency = 1000;
			break;
		}
		case MpuConfig::gyro_dlpf_frequency_type::F_41HZ:
		{
			gyro_dlpf_frequency = 41;
			gyro_internal_frequency = 1000;
			break;
		}
		case MpuConfig::gyro_dlpf_frequency_type::F_92HZ:
		{
			gyro_dlpf_frequency = 92;
			gyro_internal_frequency = 1000;
			break;
		}
		case MpuConfig::gyro_dlpf_frequency_type::F_184HZ:
		{
			gyro_dlpf_frequency = 184;
			gyro_internal_frequency = 1000;
			break;
		}
		case MpuConfig::gyro_dlpf_frequency_type::F_250HZ:
		{
			gyro_dlpf_frequency = 250;
			gyro_internal_frequency = 8000;
			break;
		}
    }

    unsigned int accel_dlpf_frequency = 0;
    unsigned int accel_internal_frequency = 1000; // Same for all dlpf frequencies.
    switch(accel_frequency)
    {
		case MpuConfig::accel_dlpf_frequency_type::F_5HZ:
		{
			accel_dlpf_frequency = 5;
			break;
		}
		case MpuConfig::accel_dlpf_frequency_type::F_10HZ:
		{
			accel_dlpf_frequency = 10;
			break;
		}
		case MpuConfig::accel_dlpf_frequency_type::F_21HZ:
		{
			accel_dlpf_frequency = 21;
			break;
		}
		case MpuConfig::accel_dlpf_frequency_type::F_44HZ:
		{
			accel_dlpf_frequency = 44;
			break;
		}
		case MpuConfig::accel_dlpf_frequency_type::F_99HZ:
		{
			accel_dlpf_frequency = 99;
			break;
		}
		case MpuConfig::accel_dlpf_frequency_type::F_218HZ:
		{
			accel_dlpf_frequency = 218;
			break;
		}
    }

    // Determine the maximum dlpf frequency.
    unsigned int internal_frequency = 0;
    unsigned int dlpf_frequency = 0;
    if(accel_dlpf_frequency > gyro_dlpf_frequency)
    {
        internal_frequency = accel_internal_frequency;
        dlpf_frequency = accel_dlpf_frequency;
    }
    else
    {
        internal_frequency = gyro_internal_frequency;
        dlpf_frequency = gyro_dlpf_frequency;
    }

    // Calculate frequency divider.
    // First, determine the desired approximate measurement frequency.
    // NOTE: Temp DLPF bandwidth is always a few hertz higher than gyro, but use of 0.5 on top of 2x multiplier (nyquist) gives enough headroom.
    float desired_frequency = std::min(static_cast<float>(dlpf_frequency) * 2.5F, max_sample_rate);
    // Calculate a frequency divider to obtain an actual frequency nearest to the desired frequency without going over.
    unsigned char frequency_divider = static_cast<unsigned char>(std::max(1.0F, std::ceil(static_cast<float>(internal_frequency) / desired_frequency)));

    // Set the sample rate divider (formula is INTERNAL_SAMPLE_RATE / (1 + DIVIDER)
    write_register(mpu9250_fd, SMPLRT_DIV, frequency_divider - 1);

    // Return the actual sample frequency.
    return internal_frequency / static_cast<float>(frequency_divider);
}
