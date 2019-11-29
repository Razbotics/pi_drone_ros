#!/usr/bin/env python
import rospy
import time
import traceback
from pymultiwii import MultiWii
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from pi_drone_ros.msg import sendRC
from std_msgs.msg import Bool

class pi_drone_driver():
    def __init__(self):
        self.board = MultiWii("/dev/ttyACM0")
        rospy.init_node("pi_drone", anonymous=True)
        self.imu_pub = rospy.Publisher("/pi_drone/imu", Imu, queue_size=1)
        self.attitude_pub = rospy.Publisher("/pi_drone/rpy", Vector3, queue_size=1)
        rospy.Subscriber("/pi_drone/RC_in", sendRC, self.send_rc_callback)
        rospy.Subscriber("/pi_drone/arm", Bool, self.arm_callback)
        self.imu_msg = Imu()
        self.rpy_msg = Vector3()
        self.rc_channels = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000]
        self.arm = False
        self.prev_imu_time = 0
        self.prev_attitude_time = 0

        while not rospy.is_shutdown():
            self.attitude_publisher(30)
            self.imu_publisher(50)

    def deg2rad(self, deg):
        rad = (3.14159/180.0) * deg
        return rad

    def attitude_publisher(self, rate):
        if ((time.time() - self.prev_attitude_time) >= (1.0 / rate)):
            self.prev_attitude_time = time.time()
            self.board.getData(MultiWii.ATTITUDE)
            self.rpy_msg.x = self.board.attitude['angx']
            self.rpy_msg.y = self.board.attitude['angy']
            self.rpy_msg.z = self.board.attitude['heading']
            self.attitude_pub.publish(self.rpy_msg)


    def imu_publisher(self, rate):
        if ((time.time() - self.prev_imu_time) >= (1.0 / rate)):
            self.prev_imu_time = time.time()
            self.board.getData(MultiWii.RAW_IMU)

            ax = self.board.rawIMU['ax']
            ay = self.board.rawIMU['ay']
            az = self.board.rawIMU['az']
            gx = self.board.rawIMU['gx']
            gy = self.board.rawIMU['gy']
            gz = self.board.rawIMU['gz']


            self.imu_msg.header.stamp = rospy.Time.now()
            self.imu_msg.header.frame_id = "imu_link"

            self.imu_msg.orientation_covariance = [-1.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0]
            self.imu_msg.angular_velocity_covariance = [-1.0, 0.0, 0.0,
                                                         0.0, 0.0, 0.0,
                                                         0.0, 0.0, 0.0]
            self.imu_msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0,
                                                            0.0, 0.0, 0.0,
                                                            0.0, 0.0, 0.0]

            self.imu_msg.linear_acceleration.x = 9.81 * ax/512
            self.imu_msg.linear_acceleration.y = 9.81 * ay/512
            self.imu_msg.linear_acceleration.z = 9.81 * az/512

            self.imu_msg.angular_velocity.x = self.deg2rad(gx/4.096)
            self.imu_msg.angular_velocity.y = self.deg2rad(gy/4.096)
            self.imu_msg.angular_velocity.z = self.deg2rad(gz/4.096)
            #rospy.loginfo(imu_msg)
            self.imu_pub.publish(self.imu_msg)


    def send_rc_callback(self, rc_data):
        self.rc_channels[0] = rc_data.channels[0]
        self.rc_channels[1] = rc_data.channels[1]
        self.rc_channels[2] = rc_data.channels[2]
        self.rc_channels[3] = rc_data.channels[3]
        if self.arm:
            self.rc_channels[4] = 2000
        else:
            self.rc_channels[4] = 1000
        #print(self.rc_channels)

        self.board.sendCMD(16,MultiWii.SET_RAW_RC, self.rc_channels)

    def arm_callback(self, msg):
        if msg.data:
            self.arm = True
        else:
            self.arm = False



if __name__ == '__main__':
    try:
       pi_drone_driver()
    except:
        rospy.logerr("Unhandled Exception in the Joy2RCin"+
                    " Node:+\n"+traceback.format_exc())
