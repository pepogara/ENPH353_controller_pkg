import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher:
    """!
    @brief      This class represents a publisher for the image.

    Used for Debugging
    """

    def __init__(self):
        """!
        @brief      Initializes the ImagePublisher object.

        This constructor sets up the publisher for the image.
        """
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.bridge = CvBridge()

    def publish(self, image):
        """!
        @brief      Publishes the image to the image_topic_2 topic.

        @param      image (Image): Image to be published.

        """
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)