
# Copyright 2024 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

import time
import numpy as np
import math

from std_msgs.msg import String
from synapse_msgs.msg import EdgeVectors
from synapse_msgs.msg import TrafficStatus
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

QOS_PROFILE_DEFAULT = 10

PI = math.pi

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = 0.0
TURN_MAX = 1.0
SPEED_STOP = 0.0
SPEED_MIN = 0.0
SPEED_MAX = 1.0
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 1.0
THRESHOLD_OBSTACLE_HORIZONTAL = 0.10
FREE_RANGE = 1.0

MIN_FWD_VEL = 0.1
MAX_FWD_VEL = 0.5
class LineFollower(Node):
	""" Initializes line follower node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('line_follower')

		# Subscription for edge vectors.
		self.subscription_vectors = self.create_subscription(
			EdgeVectors,
			'/edge_vectors',
			self.edge_vectors_callback,
			QOS_PROFILE_DEFAULT)

		# Publisher for joy (for moving the rover in manual mode).
		self.publisher_joy = self.create_publisher(
			Joy,
			'/cerebri/in/joy',
			QOS_PROFILE_DEFAULT)

		# Subscription for traffic status.
		self.subscription_traffic = self.create_subscription(
			TrafficStatus,
			'/traffic_status',
			self.traffic_status_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for LIDAR data.
		# self.subscription_lidar = self.create_subscription(
		# 	LaserScan,
		# 	'/scan',
		# 	self.lidar_callback,
		# 	QOS_PROFILE_DEFAULT)
		self.subscription_ramp_det = self.create_subscription(
			Bool,
			'/ramp_detected',
			self.ramp_det_callback,
			QOS_PROFILE_DEFAULT)
		# self.vel_sub = self.create_subscription(
		# 	Twist,
		# 	'/my_vel',
		# 	self.vel_callback,
		# 	QOS_PROFILE_DEFAULT
		# )
		self.publish_front_lidar = self.create_publisher(
			LaserScan,
			'/front_scan',
			QOS_PROFILE_DEFAULT
			)
		self.subscriber_signs = self.create_subscriber(
			String,
			'/signs',
			self.signs_callback,
			QOS_PROFILE_DEFAULT)

		self.traffic_status = TrafficStatus()

		self.obstacle_detected = False

		self.ramp_detected = False
		self.speed = 0.0
		self.turn = 0.0
		self.beta = 0.95
		self.time_now = time.time()
		self.prev_ranges = None
		self.prev_speed = 0
		self.prev_turn = 0
		self.stuck = False
		self.sign_name = 'None'
		# self.free_ranges = []/
		#-----for Aman's code-----
		self.lidar_turn = 0.0
		# ------till here---------

		# self.my_vel_timer = self.create_timer(
		# 	0.1, 
		# 	self.my_timer_callback
		# 	)
		self.obstacle_detected = False
		
		#trying to get the free space using else
		# self.min = 1e5
		# self.max = 0


	def ramp_det_callback(self, msg: Bool) -> None:
		self.ramp_detected = msg.data
	# def my_timer_callback(self):
	# 	self.rover_move_manual_mode(self.speed, self.turn)
	
	def signs_callback(self, msg: String):
		self.sign_name = msg.data
	
	def sign_handler(self):
		speed = MIN_FWD_VEL
		turn = 0.0
		sign = self.sign_name

		if sign == "left":
			turn = LEFT_TURN
		elif sign == "right":
			turn = RIGHT_TURN
		elif sign == 'stop':
			speed = SPEED_STOP
		elif sign == "straight":
			turn = TURN_MIN

		return speed, turn

	# def vel_callback(self, msg: Twist):

	# 	self.speed += msg.linear.x
	# 	self.turn += msg.angular.z
	# 	self.get_logger().info(f"speed: {self.speed:.2f}, camera_turn: {self.turn:.3f}")
	# 	if self.speed>SPEED_MAX:
	# 		self.speed = SPEED_MAX
	# 	if self.speed<-SPEED_MAX:
	# 		self.speed = -SPEED_MAX
	# 	if self.turn>TURN_MAX:
	# 		self.turn = TURN_MAX
	# 	if self.turn< -TURN_MAX:
	# 		self.turn = -TURN_MAX


	""" Operates the rover in manual mode by publishing on /cerebri/in/joy.

		Args:
			speed: the speed of the car in float. Range = [-1.0, +1.0];
				Direction: forward for positive, reverse for negative.
			camera_turn: steer value of the car in float. Range = [-1.0, +1.0];
				Direction: left camera_turn for positive, right camera_turn for negative.

		Returns:
			None
	"""
	def rover_move_manual_mode(self, speed, camera_turn):
		msg = Joy()

		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]

		msg.axes = [0.0, speed, 0.0, camera_turn]

		self.publisher_joy.publish(msg)

	""" Analyzes edge vectors received from /edge_vectors to achieve line follower application.
		It checks for existence of ramps & obstacles on the track through instance members.
			These instance members are updated by the lidar_callback using LIDAR data.
		The speed and camera_turn are calculated to move the rover using rover_move_manual_mode.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/EdgeVectors.msg"

		Returns:
			None
	"""
	def edge_vectors_callback(self, message):
		speed = self.speed
		camera_turn = self.turn
		global SPEED_MAX

		vectors = message
		half_width = vectors.image_width / 2
		half_height = vectors.image_height / 2

		# self.get_logger().info(f"vectors:{vectors}")

		# NOTE: participants may improve algorithm for line follower.
		if (vectors.vector_count == 0) and (self.obstacle_detected==False):  # none.
			speed = 0.1
			# SPEED_MAX = 0.1

		if (vectors.vector_count == 1):  # curve.
			# Calculate the magnitude of the x-component of the vector.
			deviation_x = vectors.vector_1[1].x - vectors.vector_1[0].x
			# camera_turn = SPEED_MAX * deviation_x / vectors.image_width 
			camera_turn = deviation_x / vectors.image_width 
			# camera_turn *= 10**(camera_turn)

			# camera_turn += deviation_y / vectors.image_height
			# speed = max(MIN_FWD_VEL, 0.9 * SPEED_MAX - min(abs(camera_turn), SPEED_MAX * 0.9))
			speed = 0.2
			if (camera_turn)>0.3:
				camera_turn = TURN_MAX
			if camera_turn < -0.3:
				camera_turn = -TURN_MAX
			# SPEED_MAX = 1.0
			self.get_logger().info(f"one vector camera turn: {camera_turn}")
			# self.get_logger().info(f"deviation:{deviation_x}")

		if (vectors.vector_count == 2):  # straight.
			# Calculate the middle point of the x-components of the vectors
			middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
			middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
			middle_x = (middle_x_left + middle_x_right) / 2
			deviation_x = half_width - middle_x
			camera_turn = SPEED_MAX * deviation_x / half_width

			speed = 0.5 - min(abs(camera_turn), 0.3)
			speed = SPEED_MAX * speed 
			self.get_logger().info(f"two vector camera turn: {self.camera_turn}")

			# self.get_logger().info(f"middle_x_left:{middle_x_left:.2f}, middle_x_right:{middle_x_right:.2f}, middle_x:{middle_x:.2f}, deviation_x:{deviation_x:.2f}")


			# middle_y_left = (vectors.vector_1[0].y + vectors.vector_1[1].y) / 2
			# middle_y_right = (vectors.vector_2[0].y + vectors.vector_2[1].y) / 2
			# middle_y = (middle_y_left + middle_y_right) / 2
			# deviation_y = half_height - middle_y
			# speed = deviation_y / half_height

		# if self.ramp_detected is True: 
		# 	# TODO: participants need to decide action on detection of ramp/bridge.
		# 	speed = 0.4
		# 	SPEED_MAX = 0.8
		# 	self.get_logger().info("bridge detected")


		# if (self.traffic_status.stop_sign is True):
		# 	speed = SPEED_MIN


		if self.sign_name != 'None':
			speed, turn = self.sign_handler()

		# elif self.obstacle_detected is True:

		# 	self.get_logger().info("obstacle detected")
		# 	camera_turn = max(camera_turn, self.lidar_turn)
		# 	camera_turn = self.lidar_turn
		# 	speed = max(0.3, 0.9 - min(abs(camera_turn), 0.9))
		# 	self.get_logger().info(f"lidar_turn: {self.lidar_turn:.3f}\t, speed: {speed:.3f}")

		# else:
		# 	self.get_logger().info(f"camera_turn: {camera_turn:.3f}\t, speed: {speed:.3f}")\
			
		self.speed = self.beta * self.speed + (1-self.beta) * speed
		self.turn = self.beta * self.turn + (1-self.beta) * camera_turn

		# self.time_now = time.time()
		# if self.stuck:
		# 	self.get_logger().info("stuck")
		# 	self.speed = -self.speed
		# 	speed = -speed
		# 	camera_turn = -self.lidar_turn

		self.speed = min(0.0, self.speed)

		self.get_logger().info(f"self.speed: {self.speed:.3f}, self.turn: {self.turn:.3f}, camera_turn: {camera_turn:.3f}, self.lidar_turn: {self.lidar_turn:.3f}, speed: {speed:.3f}, self.ramp_detected: {self.ramp_detected}, self.obstacle_detected: {self.obstacle_detected}, self.stuck: {self.stuck}")
		
		self.rover_move_manual_mode(self.speed, camera_turn)

	""" Updates instance member with traffic status message received from /traffic_status.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/TrafficStatus.msg"

		Returns:
			None
	"""
	def traffic_status_callback(self, message):
		self.traffic_status = message
		

	""" Analyzes LIDAR data received from /scan topic for detecting ramps/bridges & obstacles.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html"

		Returns:
			None
	"""
	def norm_func(self,num):
		return math.exp(-num**2/2)/math.sqrt(1.2*math.pi)
	def lidar_callback(self, message: LaserScan):
		# TODO: participants need to implement logic for detection of ramps and obstacles.
		for i in range(len(message.ranges)):
			if math.isinf(message.ranges[i]):
				message.ranges[i] = 10.0
		shield_vertical = 3.5
		shield_horizontal = 1
		theta = math.atan(shield_vertical / shield_horizontal)
		length = float(len(message.ranges))

		# front:

		start_idx = int(length * theta / PI)
		end_idx = int(length * (PI - theta) / PI)

		resolution = 2*math.pi / len(message.ranges)

		# Get the middle half of the ranges array returned by the LIDAR.

		ranges = message.ranges[int(length / 4): int(3 * length / 4)]

		# Separate the ranges into the part in the front and the part on the sides.
		length = float(len(ranges))
		front_ranges = ranges[start_idx: end_idx]
		side_ranges_right = ranges[0: start_idx]
		side_ranges_left = ranges[end_idx:]

		if self.prev_ranges is None:
			self.prev_ranges = front_ranges

		# process front ranges.
		angle = theta - PI / 2
		# --------------Aman's trial code----------------------------
		value = 0.0
		self.lidar_turn = 0.0

		self.ramp_detected = False
		# self.get_logger().info(f"front ranges: {front_ranges}")

		deg = len(front_ranges)
		# for time when obstacle in front
		free_indices = []
		flag= False
		len_obstacles = 0
		distances_y = []
		for i in range(deg):
			angle_from_origin = resolution * i + theta
			dist_y = front_ranges[i] * np.sin(angle_from_origin)
			distances_y.append(dist_y)
			if(dist_y>FREE_RANGE): 
				free_indices.append(- int(deg/2) + i + 1)
			else:
				flag = True
				len_obstacles+=1
				self.obstacle_detected = True
				# free_indices.append(0)
			angle += message.angle_increment
		self.get_logger().info(f"front ranges (y): {distances_y}")

		if not flag:
			self.obstacle_detected = False
		value = sum(free_indices)/(deg*2) - 0.5
		self.lidar_turn = 0.5 * np.sign(value)
		if free_indices:
			self.ramp_detected = True 

		if(self.ramp_detected):
			self.obstacle_detected = False

		else:
			nd_front = np.array(front_ranges)
			self.stuck = len(nd_front[nd_front<0.25]) > 0
			# self.get_logger().info(f"value of camera_turn: {value}")
		self.get_logger().info("----------------------------------------")
		
		
		#for the time when obstacles and no line
		# left_avg = 0
		# right_avg = 0	
		# for i in range(len(side_ranges_left)):
		# 	if(side_ranges_left[i]<THRESHOLD_OBSTACLE_HORIZONTAL or side_ranges_left[i]<THRESHOLD_OBSTACLE_HORIZONTAL):
		# 		self.get_logger().info("obstacle detected on left")
		# 		self.obstacle_detected = True
		# 		left_avg += side_ranges_left[i]
		# 	angle += message.angle_increment
		# for i in range(len(side_ranges_right)):
		# 	if(side_ranges_right[i]<THRESHOLD_OBSTACLE_HORIZONTAL or side_ranges_right[i]<THRESHOLD_OBSTACLE_HORIZONTAL):
		# 		self.get_logger().info("obstacle detected on right")
		# 		self.obstacle_detected = True
		# 		right_avg += side_ranges_right[i]
		# 	angle += message.angle_increment
		# # value = -value
		# if((left_avg-right_avg)!=0) :
		# 	value = (-left_avg+right_avg)

		#---------commment till here to remove Aman's code-------------

def main(args=None):
	rclpy.init(args=args)

	line_follower = LineFollower()

	rclpy.spin(line_follower)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	line_follower.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()