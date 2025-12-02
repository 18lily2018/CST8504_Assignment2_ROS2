import rclpy
from rclpy.node import Node

# Message type for sending images in ROS 2
from sensor_msgs.msg import Image

# Bridge between OpenCV images and ROS Image messages
from cv_bridge import CvBridge

# OpenCV for camera access and image handling
import cv2


class ImagePublisher(Node):
    """
    ImagePublisher node:
    - Opens the default camera (index 0)
    - Captures frames periodically
    - Converts them to ROS Image messages
    - Publishes them on the 'video_frames' topic
    """

    def __init__(self):
        # Call the parent Node constructor with the node name 'image_publisher'
        super().__init__('image_publisher')

        # Create a publisher that sends Image messages on 'video_frames'
        # Queue size 10 is fine for this assignment
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)

        # OpenCV: open the default camera (0 = laptop webcam)
        self.cap = cv2.VideoCapture(0)

        # Bridge used to convert between OpenCV images and ROS Image messages
        self.br = CvBridge()

        # Create a timer that calls timer_callback every 0.1 seconds (10 Hz)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Timer callback:
        - Reads a frame from the camera
        - Converts it to a ROS Image
        - Publishes it on the 'video_frames' topic
        """
        # Read one frame from the camera
        ret, frame = self.cap.read()

        # If we failed to get a frame, log a warning and return
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return

        # Convert the OpenCV BGR image to a ROS Image message
        img_msg = self.br.cv2_to_imgmsg(frame)

        # Publish the Image message
        self.publisher_.publish(img_msg)


def main(args=None):
    """
    Main entry point for the node.
    - Initializes rclpy
    - Creates the ImagePublisher node
    - Spins until shutdown (Ctrl + C)
    - Cleans up the camera and node
    """
    # Initialize the ROS 2 communication layer
    rclpy.init(args=args)

    # Create an instance of the ImagePublisher node
    image_publisher = ImagePublisher()

    # Keep the node running until it is manually shut down
    rclpy.spin(image_publisher)

    # Release the camera so it is not locked
    image_publisher.cap.release()

    # Clean up and shut down the node
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
