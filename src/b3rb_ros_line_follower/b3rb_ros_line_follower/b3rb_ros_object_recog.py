
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
from synapse_msgs.msg import TrafficStatus
from std_msgs.msg import Bool

import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage

QOS_PROFILE_DEFAULT = 10


class ObjectRecognizer(Node):
	""" Initializes object recognizer node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('object_recognizer')

		# Subscription for camera images.
		self.subscription_camera = self.create_subscription(
			CompressedImage,
			'/camera/image_raw/compressed',
			self.camera_image_callback,
			QOS_PROFILE_DEFAULT)

		# Publisher for traffic status.
		self.publisher_traffic = self.create_publisher(
			TrafficStatus,
			'/traffic_status',
			QOS_PROFILE_DEFAULT)
		
		self.publisher_ramp_det = self.create_publisher(
			Bool,
			'/ramp_detected',
			QOS_PROFILE_DEFAULT)
		

	""" Analyzes the image received from /camera/image_raw/compressed to detect traffic signs.
		Publishes the existence of traffic signs in the image on the /traffic_status topic.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CompressedImage.html"

		Returns:
			None
	"""
	def camera_image_callback(self, message):
		# Convert message to an n-dimensional numpy array representation of image.
		np_arr = np.frombuffer(message.data, np.uint8)
		image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		traffic_status_message = TrafficStatus()

		# NOTE: participants need to implement logic for recognizing traffic signs.

		lower_yellow = np.array([22, 93, 0])
		upper_yellow = np.array([45, 255, 255])
		lower_green = np.array([0,230, 0])
		upper_green = np.array([15,255, 10])
		yellow_mask = cv2.inRange(image[int(image.shape[0]*0.65):], lower_yellow, upper_yellow)
		green_mask = cv2.inRange(image[int(image.shape[0]*0.65):], lower_green, upper_green)
		if (np.sum(yellow_mask)>10) and not (np.sum(green_mask)>10):
			bool_msg = Bool()
			bool_msg.data = True
			self.publisher_ramp_det.publish(bool_msg) 
		else:
			bool_msg = Bool()
			bool_msg.data = False
			self.publisher_ramp_det.publish(bool_msg) 

		self.publisher_traffic.publish(traffic_status_message)

		stop_sign_img = cv2.imread('/home/snap/foxglove-studio/103/stop.png')
		template = cv2.GaussianBlur(stop_sign_img, (3, 3), 0)
		template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
		template = cv2.Canny(template, 50, 170)
		blur = cv2.GaussianBlur(image, (3, 3), 0) 
		gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
		edged = cv2.Canny(gray, 50, 170)
		result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF_NORMED)
		(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

		if maxVal > 0.8:
			traffic_status_message.stop_sign = True
		# else:
		# 	traffic_status_message.stop_sign = False
		self.publisher_traffic.publish(traffic_status_message)
			
	
def main(args=None):
	rclpy.init(args=args)

	object_recognizer = ObjectRecognizer()

	rclpy.spin(object_recognizer)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	object_recognizer.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
