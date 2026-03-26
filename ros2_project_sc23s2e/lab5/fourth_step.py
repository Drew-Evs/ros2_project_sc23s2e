# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

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


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 3rd Lab Session
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)

        # Initialise any flags that signal a colour has been detected (default to false)
        self.red_flag = False
        self.green_flag = False

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10

        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.forward_flag = False
        self.back_flag = False
        self.stop_flag = False
        
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        # We covered which topic to subscribe to should you wish to receive image data

    def callback(self, data):
        #reset flags
        self.green_flag = False
        self.red_flag = False
        green_area = 0
        self.stop_flag = False
        self.forward_flag = False
        self.back_flag = False

        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('camera_Feed',320,240)

        # Set the upper and lower bounds for the two colours you wish to identify
        #hue value = 0 to 179
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_red_lower = np.array([0, 100, 100])
        hsv_red_upper = np.array([self.sensitivity, 255, 255])
        hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
        hsv_red_upper2 = np.array([180, 255, 255])
        
        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour using the cv2.inRange() method
        red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        red_mask2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        r_mask = cv2.bitwise_or(red_mask, red_mask2)
        rg_mask = cv2.bitwise_or(r_mask, green_mask)

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
        filtered_img = cv2.bitwise_and(cv_image, cv_image, mask=rg_mask)

        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        contours, _ = cv2.findContours(green_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

        # Loop over the contours
        if len(contours)>0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            
            c = max(contours, key=cv2.contourArea)

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 500: #<What do you think is a suitable area?>
                green_area = cv2.contourArea(c)
                (x, y), radius = cv2.minEnclosingCircle(c)

                #convert to integer
                center = (int(x), int(y))
                radius = int(radius)
                
                cv2.circle(cv_image,center,radius,(255, 0, 0), 2)
                
                # Alter the value of the flag
                self.green_flag = True
                
        #repeat for red
        contours, _ = cv2.findContours(r_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)>0:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: 
                red_area = cv2.contourArea(c)
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(cv_image,center,radius,(255, 0, 0), 2)
                self.red_flag = True
        
        #initiate empty twist
        twist = Twist()
        
        #stop if seeing red
        if self.red_flag:
            self.get_logger().info('Stopping - seen red')
            twist.linear.x = 0.0

        #Check if a flag has been set = colour object detected - follow the colour object
        elif self.green_flag:
            if green_area < 50000:
                # Too close to object, need to move backwards
                self.get_logger().info('Following Green (Forward)')
                twist.linear.x = 0.1
                
            elif green_area > 80000:
                # Too far away from object, need to move forwards
                self.get_logger().info('Too close to Green (Backward)')
                twist.linear.x = -0.1
                
            else:
                self.get_logger().info('Perfect distance from Green. Waiting.')
                twist.linear.x = 0.0
        
        else:
            twist.linear.x = 0.0
            
        self.publisher.publish(twist)

        # Be sure to do this for the other colour as well
        # Setting the flag to  detect blue, and stop the turtlebot from moving if blue is detected

            
        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.imshow('camera_Feed', cv_image)
        cv2.waitKey(1)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args=None):
    # Instantiate your class
    rclpy.init(args=args)
    robot = Robot()

    try:
        # One single spin on the main thread. 
        # This handles the callbacks AND allows OpenCV to draw the window safely.
        rclpy.spin(robot)
    except KeyboardInterrupt:
        # Catches Ctrl+C smoothly
        pass
    finally:
        # Stop the robot before exiting
        stop_msg = Twist()
        robot.publisher.publish(stop_msg)
        
        # Clean up nodes and windows
        robot.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()