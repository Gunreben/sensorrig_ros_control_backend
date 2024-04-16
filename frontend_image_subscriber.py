import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FrontendImageSubscriber(Node):
    def __init__(self, topic):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            topic,  # Update this to your actual topic name
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.get_logger().info("Image Subscriber Node Started")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            is_success, buffer = cv2.imencode(".jpg", cv_image)
            if is_success:
                self.latest_image = buffer.tobytes()

        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber("/camera_image/cam_HR")
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
