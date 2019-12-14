#!/usr/bin/env python
import numpy as np
import math
import rospy
import time
import traceback
from geometry_msgs.msg import Vector3
from raspicam_node.msg import MotionVectors

class piOpticalFlow():
	def __init__(self):
		rospy.init_node("pi_optical_flow", anonymous=True)
		self.tap_coefs = [0.11554707639860304,0.29982701496719005,0.07038867308845864,\
		             0.2136045403910259,0.07038867308845864,0.29982701496719005,0.11554707639860304]
		self.TAP_NUMBER = len(self.tap_coefs)
		rospy.Subscriber("/pi_drone/rpy", Vector3, self.rpy_callback)
		rospy.Subscriber("/raspicam_node/motion_vectors", MotionVectors, self.motion_vectors_callback)
		self.of_pub = rospy.Publisher("/pi_drone/optical_flow", Vector3, queue_size = 1)
		self.optical_flow_msg = Vector3()
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.prev_roll = 0
		self.prev_pitch = 0
		self.buffer_index = 0
		self.motion_vectors_x_len = 0
		self.motion_vectors_y_len = 0
		self.motion_vectors_x = [0]
		self.motion_vectors_y = [0]
		self.sample_buffer_x = [0]*self.TAP_NUMBER
		self.sample_buffer_y = [0]*self.TAP_NUMBER
		self.prev_flow_process_time = 0
		self.altitude = 1 #need real time altitude from sensor just a place holder for now

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
		self.motion_vectors_x_len = vector_msg.mbx
		self.motion_vectors_y_len = vector_msg.mby
		self.motion_vectors_x = vector_msg.x
		self.motion_vectors_y = vector_msg.y


	def get_filtered_value(self):
		result_x = 0
		result_y = 0
		index = self.buffer_index

		for i in range(self.TAP_NUMBER):
			index = index - 1
			if index < 0:
				index = self.TAP_NUMBER - 1

			result_x += self.tap_coefs[i]*self.sample_buffer_x[index]
			result_y += self.tap_coefs[i]*self.sample_buffer_y[index]

		return [result_x, result_y]


	def process_flow(self, rate):
	    if ((time.time() - self.prev_flow_process_time) >= (1.0 / rate)):
			self.prev_flow_process_time = time.time()

			x_shift = -np.average(self.motion_vectors_y)
			y_shift = -np.average(self.motion_vectors_x)

			delta_rotation_roll = self.roll - self.prev_roll
			delta_rotation_pitch = self.pitch - self.prev_pitch

			self.prev_roll = self.roll
			self.prev_pitch = self.pitch
			#print(int(100*delta_rotation_pitch), int(100*delta_rotation_roll))
			x_shift = (x_shift/127)*16*rate * 1.12e-6  #pixels/sec * 1.12e-6 = m/sec
			y_shift = (y_shift/127)*16*rate * 1.12e-6 #pixel size 1.12 micrometer = 1.12e-6 m

			x_shift = math.atan2(x_shift, 3.04e-3) #focal length 3.04e-3 atan2(x_shift/focal_length) = rad/sec
            y_shift = math.atan2(y_shift, 3.04e-3)
			print(x_shift, y_shift)
			# remove rotation movement
			x_shift_corrected = x_shift + delta_rotation_pitch*rate #x_shift - rad/sec
			y_shift_corrected = y_shift - delta_rotation_roll*rate  #y_shift - rad/sec
            x_shift_corrected *= self.altitude #rad/sec * altitude = m/s
			x_shift_corrected *= self.altitude #rad/sec * altitude = m/s

			self.sample_buffer_x[self.buffer_index] = x_shift_corrected
			self.sample_buffer_y[self.buffer_index] = y_shift_corrected
			self.buffer_index += 1
			if self.buffer_index >= self.TAP_NUMBER:
				self.buffer_index=0

			data_motion = self.get_filtered_value()

			self.optical_flow_msg.x = data_motion[0]
			self.optical_flow_msg.y = data_motion[1]
			self.of_pub.publish(self.optical_flow_msg)
			#print(data_motion[0], data_motion[1])


if __name__ == "__main__":
    try:
       piOpticalFlow()
    except:
        rospy.logerr("Unhandled Exception in the pi optical flow"+
                    " Node:+\n"+traceback.format_exc())
