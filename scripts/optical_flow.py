#!/usr/bin/env python
import numpy as np
import math
import rospy
import time
import traceback
import tf
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from raspicam_node.msg import MotionVectors
from std_msgs.msg import Bool

def broadcast_tf(pose_msg):
	br = tf.TransformBroadcaster()
	br.sendTransform((pose_msg.position.x, pose_msg.position.y, pose_msg.position.z),
						(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w),
						 rospy.Time.now(), "base_link", "odom")

def deg2rad(deg):
    rad = (3.14159/180.0) * deg
    return rad

class lowPassFilter():
	def __init__(self, tap_coefs):
		self.tap_coefs = tap_coefs
		self.len_coefs = len(tap_coefs)
		self.sample_buffer = [0]*self.len_coefs

	def filter(self, data):
		self.result = 0
		self.sample_buffer.append(data)
		if len(self.sample_buffer) > self.len_coefs:
			self.sample_buffer.pop(0)

		for i in range(self.len_coefs):
			self.result += self.tap_coefs[i]*self.sample_buffer[i]
		return self.result


class piOpticalFlow():
	def __init__(self):
		rospy.init_node("pi_optical_flow", anonymous=True)
		self.scalar_x = 1.0  #motion vector to pixel scalar
		self.scalar_y = 1.0
		rospy.Subscriber("/pi_drone/rpy", Vector3, self.rpy_callback)
		rospy.Subscriber("/raspicam_node/motion_vectors", MotionVectors, self.motion_vectors_callback)
		rospy.Subscriber("/pi_drone/reset_pose", Bool, self.reset_pose)
		self.of_pub = rospy.Publisher("/pi_drone/optical_flow", Vector3, queue_size = 1)
		self.pose_pub = rospy.Publisher("/pi_drone/pose", PoseStamped, queue_size = 1)
		self.optical_flow_msg = Vector3()
		self.pose_msg = PoseStamped()
		self.rate = 20.0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.prev_roll = 0
		self.prev_pitch = 0
		self.velocity_x = 0
		self.velocity_y = 0
		self.prev_velocity_x = 0
		self.prev_velocity_y = 0
		self.position_x = 0
		self.position_y = 0
		self.motion_vectors_x = [0]
		self.motion_vectors_y = [0]
		self.prev_flow_process_time = 0
		self.altitude = 1.0 #need real time altitude from sensor just a place holder for now
		#self.tap_coefs = [0.1,0.25,0.05,0.2,0.05,0.25,0.1]
		self.tap_coefs = [0.05, 0.2, 0.5, 0.2, 0.05] #these coefs are for exponential average filter
		self.low_pass_filter_x = lowPassFilter(self.tap_coefs)
		self.low_pass_filter_y = lowPassFilter(self.tap_coefs)
		self.low_pass_filter_roll = lowPassFilter(self.tap_coefs)
		self.low_pass_filter_pitch = lowPassFilter(self.tap_coefs)
		self.low_pass_filter_corrected_x = lowPassFilter(self.tap_coefs)
		self.low_pass_filter_corrected_y = lowPassFilter(self.tap_coefs)
		self.initialise = True
		self.initial_yaw = 0

		while not rospy.is_shutdown():
			self.process_flow(self.rate)

	def reset_pose(self, msg):
		self.yaw = 0
		self.initial_yaw = 0
		self.initialise = True
		self.position_x = 0
		self.position_y = 0

	def rpy_callback(self, rpy_msg):
		self.roll = deg2rad(rpy_msg.x)
		self.pitch = deg2rad(rpy_msg.y)
		self.yaw = deg2rad(360.0 - rpy_msg.z)
		if self.initialise:
			self.initial_yaw = self.yaw
			self.initialise = False

	def motion_vectors_callback(self, vector_msg):
		self.motion_vectors_x = vector_msg.x
		self.motion_vectors_y = vector_msg.y

	def process_flow(self, rate):
	    if ((time.time() - self.prev_flow_process_time) >= (1.0 / self.rate)):
			self.prev_flow_process_time = time.time()

			x_vectors = np.array(self.motion_vectors_x)
			y_vectors = np.array(self.motion_vectors_y)
			x_vectors = x_vectors[np.nonzero(x_vectors)]
			y_vectors = y_vectors[np.nonzero(y_vectors)]
			if len(x_vectors) == 0:
				x_vectors = np.append(x_vectors,[0])
			if len(y_vectors) == 0:
				y_vectors = np.append(y_vectors,[0])
			x_shift = -np.average(y_vectors) #optical flow is relative to quadrotor
			y_shift = -np.average(x_vectors) # x forward    y left

			delta_rotation_roll = (self.roll - self.prev_roll) * self.rate # 1/dt = rate
			delta_rotation_pitch = (self.pitch - self.prev_pitch) * self.rate
			self.prev_roll = self.roll
			self.prev_pitch = self.pitch

			x_shift = self.rate * self.altitude * math.atan2(self.scalar_x * x_shift, 500)  # z * v/f, f = 500 pixels
			y_shift = self.rate * self.altitude * math.atan2(self.scalar_y * y_shift, 500)  # scalar still needs to be determined
			#print(x_shift, y_shift)

			motion_x = self.low_pass_filter_x.filter(x_shift)
			motion_y = self.low_pass_filter_y.filter(y_shift)
			delta_rotation_roll = self.low_pass_filter_roll.filter(delta_rotation_roll)
			delta_rotation_pitch = self.low_pass_filter_pitch.filter(delta_rotation_pitch)
			#print(delta_rotation_roll)
			# remove rotation movement
			motion_corrected_x = motion_x + delta_rotation_pitch #recover translational components
			motion_corrected_y = motion_y - delta_rotation_roll

			#applying filter again
			self.velocity_x = self.low_pass_filter_corrected_x.filter(motion_corrected_x)
			self.velocity_y = self.low_pass_filter_corrected_y.filter(motion_corrected_y)

			self.optical_flow_msg.x = self.velocity_x #delta_rotation_roll
			self.optical_flow_msg.y = self.velocity_y

			self.of_pub.publish(self.optical_flow_msg)
			self.publish_pose()
			#print(data_motion[0], data_motion[1])

	def publish_pose(self):
		velocity_global_x = self.velocity_x * math.cos(self.yaw) - self.velocity_y * math.sin(self.yaw)
		velocity_global_y = self.velocity_x * math.sin(self.yaw) + self.velocity_y * math.cos(self.yaw)
		self.position_x += velocity_global_x  * (1.0/self.rate)
		self.position_y += velocity_global_y  * (1.0/self.rate)
		self.pose_msg.header.stamp = rospy.Time.now()
		self.pose_msg.header.frame_id = "odom"
		self.pose_msg.pose.position.x = self.position_x
		self.pose_msg.pose.position.y = self.position_y
		self.pose_msg.pose.position.z = 0.0 #self.altitude
		euler_to_quat = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw - self.initial_yaw)
		self.pose_msg.pose.orientation.x = euler_to_quat[0]
		self.pose_msg.pose.orientation.y = euler_to_quat[1]
		self.pose_msg.pose.orientation.z = euler_to_quat[2]
		self.pose_msg.pose.orientation.w = euler_to_quat[3]
		self.pose_pub.publish(self.pose_msg)
		broadcast_tf(self.pose_msg.pose)


if __name__ == "__main__":
    try:
       piOpticalFlow()
    except:
        rospy.logerr("Unhandled Exception in the pi optical flow"+
                    " Node:+\n"+traceback.format_exc())
