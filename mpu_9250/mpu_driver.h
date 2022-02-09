/*
 * mpu_driver.h
 *
 *  Created on: Dec 7, 2021
 *      Author: sujiwo
 */

#ifndef MPU_9250_MPU_DRIVER_H_
#define MPU_9250_MPU_DRIVER_H_

#include <string>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define M_G 9.81f


class MpuDriver {
public:

	typedef Eigen::Vector3f Vector3;
	typedef Eigen::Vector4f Vector4;
	typedef Eigen::Matrix4f Matrix4;
	typedef Eigen::Quaternionf Quaternion;

	struct MpuData {
		Vector3
			accel = Vector3::Zero(),
			gyro = Vector3::Zero(),
			magneticField = Vector3::Zero();
		float temperature = 0;

		// In roll/pitch/yaw, degrees
		Vector3 mHeading = Vector3::Zero();

		void toRos (sensor_msgs::Imu &imudata) const;

		void toRos (sensor_msgs::MagneticField &magn) const;

		void toRos (sensor_msgs::Temperature &temp) const
		{ temp.temperature = temperature; }

		void toRos (sensor_msgs::Imu &imu, sensor_msgs::MagneticField &magn, sensor_msgs::Temperature &temp) const
		{ toRos(imu); toRos(magn); toRos(temp); }
	};

	struct Calibration {
		// Gyroscope calibration values are means of each axes values when stationary
		Vector3 gyro;
		// Accelerometer calibration values are expressed in matrix 3x2, one row for each axis;
		// To fit the new axis value: x = x*x_1 + x_2
		Eigen::Matrix<float,3,2> accel;

		Calibration();

		static Calibration loadCalibration(const std::string &path);
	};

	struct MpuConfig {

	    /// \brief Enumerates the digital low-pass filter (DLPF) cutoff frequencies for the accelerometers.
	    enum class accel_dlpf_frequency_type
	    {
	        F_218HZ = 0x00,
	        F_99HZ = 0x02,
	        F_44HZ = 0x03,
	        F_21HZ = 0x04,
	        F_10HZ = 0x05,
	        F_5HZ = 0x06
	    };
	    /// \brief Enumerates the digital low-pass filter (DLPF) cutoff frequencies for the gyros.
	    enum class gyro_dlpf_frequency_type
	    {
	        F_250HZ = 0x00,
	        F_184HZ = 0x01,
	        F_92HZ = 0x02,
	        F_41HZ = 0x03,
	        F_20Hz = 0x04,
	        F_10Hz = 0x05,
	        F_5HZ = 0x06
	    };

		enum AccelSensitivity {
			FSR_2G = 2,
			FSR_4G = 4,
			FSR_8G = 8,
			FSR_16G = 16
		};

		MpuConfig() {}
		int mpu_address = 0x68;
		float
			accel_sensitivity = FSR_2G,	// in g (9.81ms-2)
			gyro_sensitivity = 250.0;	// in degree-per-sec
		float
			local_gravity_const = M_G;
		bool
			is_calibrating = false;

		Calibration dCalibration;
	};

	const uint
		//MPU6050 Registers
		MPU6050_ADDR 		= 0x68,
		MPU6050_WHO_AM_I 	= 0x75,
		PWR_MGMT_1   		= 0x6B,
		SMPLRT_DIV			= 0x19,
		CONFIG				= 0x1A,
		GYRO_CONFIG			= 0x1B,
		ACCEL_CONFIG		= 0x1C,
		ACCEL_CONFIG_2		= 0x1D,
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

	MpuDriver(int bus_num, const MpuConfig &cfg=MpuConfig());
	virtual ~MpuDriver();

	void loadCalibration(const std::string &calibFile);

	bool read_values(MpuData &data);

	inline bool hasMagnetometer() const
	{ return mHasMagnetometer; }

	/*
	 * Sets the digital low-pass filter (DLPF) cutoff frequencies for the accelerometers and gyroscopes.
	 * @param gyro_frequency The cut-off frequency for the gyroscopes and temperature sensor.
	 * @param accel_frequency The cut-off frequency for the accelerometers.
	 * @param max_sample_rate The maximum sample rate to use. Defaults to unlimited.
	 * @return The configured data sample rate (Hz)
	 * @note The data rate is set to the nearest minimum value of lpf/2.5 or max_sample_rate.
	 */
	float set_dlpf_frequencies(
		MpuConfig::gyro_dlpf_frequency_type gyro_frequency,
		MpuConfig::accel_dlpf_frequency_type accel_frequency,
		float max_sample_rate = 8000.0F);

private:
	int
		mpu9250_fd = -1,
		ak8963_fd = -1;
	bool mHasMagnetometer = false;

	Vector3 mHeading = Vector3::Zero();

	// Default full-scale range
	float
		accel_scale = 2.0f,			// 2G
		gyro_scale = 250.f;			// 250 deg/s

	const MpuConfig devcfg;

	Calibration dCalib;

	ros::Time lastUpdate;

protected:
	static bool read_registers(int dev_fd, uint register_address, uint num_bytes, std::vector<uint8_t> &buffer);
	static int8_t read_register(int dev_fd, uint register_address);
	static bool write_register(int dev_fd, uint register_address, int8_t w);
	static uint16_t read_raw_words(int dev_fd, uint register_address, bool isBigEndian=true);
	void initialize(int bus_num);
	void update_orientation(MpuData &data, const ros::Duration &timediff);
};

#endif /* MPU_9250_MPU_DRIVER_H_ */
