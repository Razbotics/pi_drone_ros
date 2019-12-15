#!/usr/bin/env python
import numpy as np
import math
import rospy
import time
import traceback
from geometry_msgs.msg import Vector3
from raspicam_node.msg import MotionVectors

class lowPassFilter():
	def __init__(self, tap_coefs):
		self.tap_coefs = tap_coefs
		self.len_coefs = len(tap_coefs)
		self.sample_buffer = [0]*self.len_coefs
		self.buffer_index = 0

	def filter(self, data):
		self.result = 0
		self.sample_buffer[self.buffer_index] = data
		self.buffer_index += 1
		if self.buffer_index >= self.len_coefs:
			self.buffer_index=0

		self.index = self.buffer_index
		for i in range(self.len_coefs):
			self.index = self.index - 1
			if self.index < 0:
				self.index = self.len_coefs - 1
			self.result += self.tap_coefs[i]*self.sample_buffer[self.index]
		return self.result


class piOpticalFlow():
	def __init__(self):
		rospy.init_node("pi_optical_flow", anonymous=True)
		self.scalar = 10.0
		rospy.Subscriber("/pi_drone/rpy", Vector3, self.rpy_callback)
		rospy.Subscriber("/raspicam_node/motion_vectors", MotionVectors, self.motion_vectors_callback)
		self.of_pub = rospy.Publisher("/pi_drone/optical_flow", Vector3, queue_size = 1)
		self.optical_flow_msg = Vector3()
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.prev_roll = 0
		self.prev_pitch = 0
		self.motion_vectors_x = [0]
		self.motion_vectors_y = [0]
		self.prev_flow_process_time = 0
		self.altitude = 1 #need real time altitude from sensor just a place holder for now
		#self.tap_coefs = [0.1,0.25,0.05,0.2,0.05,0.25,0.1]
		self.tap_coefs = [0.2] * 5 #these coefs are for average filter
		self.low_pass_filter_x = lowPassFilter(self.tap_coefs)
		self.low_pass_filter_y = lowPassFilter(self.tap_coefs)
		self.low_pass_filter_roll = lowPassFilter(self.tap_coefs)
		self.low_pass_filter_pitch = lowPassFilter(self.tap_coefs)
		self.low_pass_filter_corrected_x = lowPassFilter(self.tap_coefs)
		self.low_pass_filter_corrected_y = lowPassFilter(self.tap_coefs)

		while not rospy.is_shutdown():
			self.process_flow(20.0)

	def deg2rad(self, deg):
	    rad = (3.14159/180.0) * deg
	    return rad

	def rpy_callback(self, rpy_msg):
		self.roll = self.deg2rad(rpy_msg.x)
		self.pitch = self.deg2rad(rpy_msg.y)
		self.yaw = self.deg2rad(rpy_msg.z)


	def motion_vectors_callback(self, vector_msg):
		self.motion_vectors_x = vector_msg.x
		self.motion_vectors_y = vector_msg.y


	def process_flow(self, rate):
	    if ((time.time() - self.prev_flow_process_time) >= (1.0 / rate)):
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

			delta_rotation_roll = (self.roll - self.prev_roll) * rate
			delta_rotation_pitch = (self.pitch - self.prev_pitch) * rate
			self.prev_roll = self.roll
			self.prev_pitch = self.pitch

			x_shift = rate * self.scalar * (x_shift/2714)  # v/f, f = 2714 pixels according to specs
			y_shift = rate * self.scalar * (y_shift/2714)  # scalar still needs to be determined
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
			motion_corrected_x = self.low_pass_filter_corrected_x.filter(motion_corrected_x)
			motion_corrected_y = self.low_pass_filter_corrected_y.filter(motion_corrected_y)
			velocity_x = motion_corrected_x * self.altitude
			velocity_y = motion_corrected_y * self.altitude

			self.optical_flow_msg.x = velocity_x #delta_rotation_roll
			self.optical_flow_msg.y = velocity_y
			self.optical_flow_msg.z = motion_y

			self.of_pub.publish(self.optical_flow_msg)
			#print(data_motion[0], data_motion[1])


if __name__ == "__main__":
    try:
       piOpticalFlow()
    except:
        rospy.logerr("Unhandled Exception in the pi optical flow"+
                    " Node:+\n"+traceback.format_exc())
