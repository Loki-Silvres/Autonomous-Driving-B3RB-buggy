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
import math

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
SPEED_MIN = 0.0
SPEED_MAX = 1.0
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 1.0
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25
THRESHOLD_RAMP = 0.5

MIN_FWD_VEL = 0.4
MAX_FWD_VEL = 1.0

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
		self.subscription_lidar = self.create_subscription(
			LaserScan,
			'/scan',
			self.lidar_callback,
			QOS_PROFILE_DEFAULT)
		self.subscription_ramp_det = self.create_subscription(
			Bool,
			'/ramp_detected',
			self.ramp_det_callback,
			QOS_PROFILE_DEFAULT)
		self.vel_sub = self.create_subscription(
			Twist,
			'/my_vel',
			self.vel_callback,
			QOS_PROFILE_DEFAULT
		)

		self.traffic_status = TrafficStatus()

		self.obstacle_detected = False

		self.ramp_detected = False
		self.speed = 0.0
		self.turn = 0.0
		self.beta = 0.9
		self.time_now = -100

		self.my_vel_timer = self.create_timer(
			0.1, 
			self.my_timer_callback
			)
		
	def ramp_det_callback(self, msg: Bool) -> None:
		self.ramp_detected = msg.data
	def my_timer_callback(self):
		self.rover_move_manual_mode(self.speed, self.turn)

	def vel_callback(self, msg: Twist):

		self.speed += msg.linear.x
		self.turn += msg.angular.z
		self.get_logger().info(f"speed: {self.speed}, turn: {self.turn}")
		if self.speed>SPEED_MAX:
			self.speed = SPEED_MAX
		if self.speed<-SPEED_MAX:
			self.speed = -SPEED_MAX
		if self.turn>TURN_MAX:
			self.turn = TURN_MAX
		if self.turn< -TURN_MAX:
			self.turn = -TURN_MAX


	""" Operates the rover in manual mode by publishing on /cerebri/in/joy.

		Args:
			speed: the speed of the car in float. Range = [-1.0, +1.0];
				Direction: forward for positive, reverse for negative.
			turn: steer value of the car in float. Range = [-1.0, +1.0];
				Direction: left turn for positive, right turn for negative.

		Returns:
			None
	"""
	def rover_move_manual_mode(self, speed, turn):
		msg = Joy()

		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]

		msg.axes = [0.0, speed, 0.0, turn]

		self.publisher_joy.publish(msg)

	""" Analyzes edge vectors received from /edge_vectors to achieve line follower application.
		It checks for existence of ramps & obstacles on the track through instance members.
			These instance members are updated by the lidar_callback using LIDAR data.
		The speed and turn are calculated to move the rover using rover_move_manual_mode.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/EdgeVectors.msg"

		Returns:
			None
	"""
	def edge_vectors_callback(self, message):
		speed = SPEED_MAX
		turn = TURN_MIN

		vectors = message
		half_width = vectors.image_width / 2
		half_height = vectors.image_height / 2

		self.get_logger().info(f"vectors:{vectors}")

		# NOTE: participants may improve algorithm for line follower.
		if (vectors.vector_count == 0):  # none.
			speed = MIN_FWD_VEL	

		if (vectors.vector_count == 1):  # curve.
			# Calculate the magnitude of the x-component of the vector.
			deviation_x = vectors.vector_1[1].x - vectors.vector_1[0].x
			deviation_y = vectors.vector_1[1].y - vectors.vector_1[0].y
			turn = deviation_x / vectors.image_width
			# turn += deviation_y / vectors.image_height
			speed = max(MIN_FWD_VEL, 0.9 - min(abs(turn), 0.9))

			self.get_logger().info(f"deviation:{deviation_x}")


		if (vectors.vector_count == 2):  # straight.
			# Calculate the middle point of the x-components of the vectors.
			middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
			middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
			middle_x = (middle_x_left + middle_x_right) / 2
			deviation_x = half_width - middle_x
			turn = deviation_x / half_width

			speed = 1 - min(abs(turn), 0.9)

			self.get_logger().info(f"middle_x_left:{middle_x_left}, middle_x_right:{middle_x_right}, middle_x:{middle_x}, deviation_x:{deviation_x}")


			# middle_y_left = (vectors.vector_1[0].y + vectors.vector_1[1].y) / 2
			# middle_y_right = (vectors.vector_2[0].y + vectors.vector_2[1].y) / 2
			# middle_y = (middle_y_left + middle_y_right) / 2
			# deviation_y = half_height - middle_y
			# speed = deviation_y / half_height

		if (self.traffic_status.stop_sign is True):
			speed = SPEED_MIN
			print("stop sign detected")

		if self.ramp_detected is True: # or (time.time()-self.time_now) < 2:
			# TODO: participants need to decide action on detection of ramp/bridge.
			speed = SPEED_MAX/5
			self.time_now = time.time()
			print("ramp/bridge detected")
			# self.get_logger().info("ramp/bridge detected")

		if self.obstacle_detected is True:
			# TODO: participants need to decide action on detection of obstacle.
			print("obstacle detected")
		

		self.speed = self.beta * self.speed + (1-self.beta) * speed
		self.turn = self.beta * self.turn + (1-self.beta) * turn
		self.get_logger().info(f"self.speed: {self.speed}, self.turn: {self.turn}, turn: {turn}, speed: {speed}, self.ramp_detected: {self.ramp_detected}")
		self.rover_move_manual_mode(self.speed, self.turn)

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
	def lidar_callback(self, message):
		# TODO: participants need to implement logic for detection of ramps and obstacles.

		shield_vertical = 4
		shield_horizontal = 1
		theta = math.atan(shield_vertical / shield_horizontal)

		# Get the middle half of the ranges array returned by the LIDAR.
		length = float(len(message.ranges))
		ranges = message.ranges[int(length / 4): int(3 * length / 4)]

		# Separate the ranges into the part in the front and the part on the sides.
		length = float(len(ranges))
		front_ranges = ranges[int(length * theta / PI): int(length * (PI - theta) / PI)]
		side_ranges_right = ranges[0: int(length * theta / PI)]
		side_ranges_left = ranges[int(length * (PI - theta) / PI):]

		# process front ranges.
		angle = theta - PI / 2
		self.get_logger().info(f"range: {front_ranges}")

		for i in range(len(front_ranges)):
			# self.get_logger().info(f"range: {front_ranges[i]}")
			
			if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
				self.obstacle_detected = True
				return
			elif (front_ranges[i]< THRESHOLD_RAMP):
				self.ramp_detected = True
				self.get_logger().info("ramp/bridge detected")

			angle += message.angle_increment

		# process side ranges.
		side_ranges_left.reverse()
		for side_ranges in [side_ranges_left, side_ranges_right]:
			angle = 0.0
			for i in range(len(side_ranges)):
				if (side_ranges[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
					self.obstacle_detected = True
					return

				angle += message.angle_increment

		self.obstacle_detected = False


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
