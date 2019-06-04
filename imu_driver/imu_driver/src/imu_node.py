#!/usr/bin/env python
import smbus
import time
import sys
import os

import rospy

from imu_driver import mpu9250
from imu_driver.srv import SetIMUState, SetIMUStateResponse

class IMUHandler(object):
    def __init__(self, sensor_name):

        self.sensor_name = sensor_name
        #self.mux_port = mux_port

        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        self.current_state = True

        self.state_service = rospy.Service('~set_%s_state'%self.sensor_name,
                           SetIMUState,
                           self.state_handler)

        self.sensor = mpu9250()


    def state_handler(self, req):
        self.current_state = req.state
        return SetToFStateResponse("Success")

    def publish_data(self):
        sensor = mpu9250()
    	try:
            a = sensor.accel
            print 'Accel: {:.3f} {:.3f} {:.3f} mg'.format(*a)
            g = sensor.gyro
            print 'Gyro: {:.3f} {:.3f} {:.3f} dps'.format(*g)
            # m = imu.mag
            # print 'Magnet: {:.3f} {:.3f} {:.3f} mT'.format(*m)
            # m = imu.temp
            # print 'Temperature: {:.3f} C'.format(m)
            time.sleep(0.001)

        except IOError as (errno, strerror):
            print "I/O error({0}): {1}".format(errno, strerror)

if __name__ == '__main__':
    rospy.init_node('~imu_node', anonymous=False)

    imu_sensor = IMUHandler('imu_sensor')
    #
    # tof_sensors['left'] = ToFHandler('left_tof',0x01)
    # tof_sensors['middle'] = ToFHandler('middle_tof',0x02)
    # tof_sensors['right'] = ToFHandler('right_tof',0x04)

    while not rospy.is_shutdown():
        #for sensor_name,sensor in imu_sensor.iteritems():
        if imu_sensor.current_state:
            imu_sensor.publish_data()
        time.sleep(0.001)
