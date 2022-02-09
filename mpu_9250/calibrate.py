#!/usr/bin/python3

"""
Calibration How-To
==================

1. Launch driver node:
   $ roslaunch mpu_9250 mpu_9250.launch bus_number:=<bus_number> mpu6050_address:=<device_address_in_i2c> is_calibrating:=1
   Important: is_calibrating parameter is mandatory for calibration
2. Run calibrate node:
   $ rosrun mpu_9250 calibrate.py /imu:=/mpu_9250/imu
   Follow instructions on the screen
"""


import rospy
import numpy as np
import yaml
from copy import copy
from scipy.optimize import curve_fit
from sensor_msgs.msg import Imu

calibration_size = 500
calibrate_output = '/tmp/calibration.yml'

def collect_message(topicname, messageType, buffer_len=100):
    buffer = []
    buffer_counter=0
    def _my_callback(msg):
        nonlocal buffer, buffer_counter
        if buffer_counter < buffer_len:
            buffer.append(msg)
            buffer_counter += 1
    _subscriber = rospy.Subscriber(topicname, messageType, _my_callback)
    for i in range(buffer_len):
        rospy.wait_for_message(topicname, messageType)
    return buffer


# Calibration computes average of all axes when stationary
def calibrate_gyro():
    global calibration_size
    print("Gyroscope Calibration")
    message_buffer = collect_message("imu", Imu, calibration_size)
    gyro_msgs = [[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in message_buffer]
    gyro_values = np.array(gyro_msgs)
    calb_values = np.average(gyro_values, axis=0)
    print("Gyroscope Calibration complete")
    return [float(calb_values[0]), float(calb_values[1]), float(calb_values[2])] 

def calibrate_accel():
    global calibration_size
    
    def accel_fit(x_input,m_x,b):
        return (m_x*x_input)+b # fit equation for accel calibration

    print("Accelerometer Calibration")
    mpu_offsets = [[],[],[]] # offset array to be printed
    axis_vec = ['Z','Y','X'] # axis labels
    cal_directions = ["upward","downward","perpendicular to gravity"] # direction for IMU cal
    cal_indices = [2,1,0] # axis indices
    for qq,ax_qq in enumerate(axis_vec):
        ax_offsets = [[],[],[]]
        print("-"*50)
        for direc_ii,direc in enumerate(cal_directions):
            input("-"*8+" Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the "+\
              ax_qq+"-axis pointed "+direc)
            message_buffer = collect_message("imu", Imu, calibration_size) # clear buffer between readings
            mpu_array = []
            for msg in message_buffer:
                ax,ay,az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                mpu_array.append([ax,ay,az]) 
            ax_offsets[direc_ii] = np.array(mpu_array)[:,cal_indices[qq]] # offsets for direction

        # Use three calibrations (+1g, -1g, 0g) for linear fit
        popts,_ = curve_fit(accel_fit,np.append(np.append(ax_offsets[0],
                                 ax_offsets[1]),ax_offsets[2]),
                   np.append(np.append(1.0*np.ones(np.shape(ax_offsets[0])),
                    -1.0*np.ones(np.shape(ax_offsets[1]))),
                        0.0*np.ones(np.shape(ax_offsets[2]))),
                            maxfev=10000)
        mpu_offsets[cal_indices[qq]] = popts.tolist() # place slope and intercept in offset array
    print('Accelerometer Calibrations Complete')
    return mpu_offsets


if __name__ == '__main__' :
    rospy.init_node('calibrator')

    gyro_offsets = calibrate_gyro()
    accel_offsets = calibrate_accel()
    
    calib_file = open(calibrate_output, 'w')
    yaml.dump({'gyroscope': gyro_offsets, 'accelerometer': accel_offsets}, calib_file)
    calib_file.close()
    
    print("Output written to {}; Press CTRL+C to exit".format(calibrate_output))
    
    pass