# Exercise 1 - Display an image of the camera feed to the screen

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data

        #initate bridge 
        self.bridge = CvBridge()

        #subscription to image topic 
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        #initiate once
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL) 
        cv2.resizeWindow('camera_Feed', 320, 240) 
        self.sensitivity = 5 #want a range of 10 around colour
        
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        try:
            #convert from iomage to opencv
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #show image on a window
            # cv2.imshow('camera_Feed', cv_image)
            # cv2.waitKey(3) 
            
            #convert image to hue saturation value
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            #bounds for green in terms of colour saturation brightness
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            #repeat for blue and red (red either end of spectrum)
            hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
            
            hsv_red_lower = np.array([0, 100, 100])
            hsv_red_upper = np.array([self.sensitivity, 255, 255])
            hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
            hsv_red_upper2 = np.array([180, 255, 255])
            
            #filter for just green
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
            red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
            red_mask2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
            
            #combine masks
            r_mask = cv2.bitwise_or(red_mask, red_mask2)
            gb_mask = cv2.bitwise_or(blue_mask, green_mask)
            rgb_mask = cv2.bitwise_or(gb_mask, r_mask)
            
            #filter the imagte with the rgb mask and show on camera feed
            filtered_img = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)
            cv2.imshow('camera_Feed', filtered_img)
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        
    def callback(self, data):
        return
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        # Show the resultant images you have created.
        

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():        
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    cI = colourIdentifier()

    def signal_handler(sig, frame):
        cI.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()
    

# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
