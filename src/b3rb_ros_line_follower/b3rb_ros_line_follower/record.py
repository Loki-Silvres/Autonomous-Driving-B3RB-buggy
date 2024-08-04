import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Video writer setup
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec
        self.out = cv2.VideoWriter('output_video.mp4', self.fourcc, 10.0, (320, 240))  # Frame size

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.get_logger().info('Video Recorder Node has been started.')

    def listener_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Check if the frame is valid
            if cv_image is not None:
                # Write the image to the video file
                self.out.write(cv_image)
        
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def destroy_node(self):
        # Release video writer resources
        if self.out is not None:
            self.out.release()
        self.get_logger().info('Video file has been saved.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    video_recorder = VideoRecorderNode()
    
    try:
        rclpy.spin(video_recorder)
    except KeyboardInterrupt:
        video_recorder.get_logger().info('Keyboard interrupt received. Stopping video recording.')
    finally:
        video_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
