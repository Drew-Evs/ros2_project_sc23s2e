# Exercise 3 - If green object is detected, and above a certain size, then send a message (print or use lab2)

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
        # Initialise any flags that signal a colour has been detected (default to false)

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.bridge = CvBridge()
        self.sensitivity = 5 
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.green_flag = False
        self.count = 0


    def callback(self, data):


        # Convert the received image into a opencv image
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('camera_Feed',320,240)

        # But remember that you should always wrap a call to this conversion method in an exception handler

        # Set the upper and lower bounds for the colour you wish to identify - green
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour using the cv2.inRange() method
                # Set the upper and lower bounds for the two colours you wish to identify
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but particular colours using the cv2.inRange() method
        # Do this for each colour
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

        # Apply the mask to the original image using the cv2.bitwise_and() method
        filtered_img = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)

        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        contours, _ = cv2.findContours(green_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Loop over the contours
            # There are a few different methods for identifying which contour is the biggest:
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            c = max(contours, key=cv2.contourArea)

            #Moments can calculate the center of the contour
            M = cv2.moments(c)
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 500: #<What do you think is a suitable area?>

                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                (x, y), radius = cv2.minEnclosingCircle(c)

                #convert to integer
                center = (int(x), int(y))
                radius = int(radius)
                
                cv2.circle(cv_image,center,radius,(0, 0, 255), 2)

                # Then alter the values of any flags
                self.green_flag = True
                self.count += 1
            

        #if the flag is true (colour has been detected)
            #print the flag or colour to test that it has been detected
            #alternatively you could publish to the lab1 talker/listener
        if self.green_flag == True:
            print(f'Green found {self.count} times')
            
        self.green_flag = False

        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.imshow('camera_Feed', cv_image)
        cv2.waitKey(3)


# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
   
    cI = colourIdentifier()

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
