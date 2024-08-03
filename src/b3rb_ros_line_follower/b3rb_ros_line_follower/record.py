import rclpy 
import cv2 
import numpy as np
from sensor_msgs.msg import Image
from rclpy.node import Node

QOS_PROFILE_DEFAULT = 10
class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.subscription_camera = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_video_callback,
            QOS_PROFILE_DEFAULT)
        self.video_writer = None
    def camera_video_callback(self, message):
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if self.video_writer is None:
            self.video_writer = cv2.VideoWriter('video.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 10, (320, 240))
        self.video_writer.write(image)
    def __del__(self):
        if self.video_writer is not None:
            self.video_writer.release()


def main(args=None):
	rclpy.init(args=args)

	record = VideoRecorder()

	rclpy.spin(record)
	record.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()