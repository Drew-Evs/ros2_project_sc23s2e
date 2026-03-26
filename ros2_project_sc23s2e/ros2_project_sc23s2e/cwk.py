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

#for reading the map && random sampling
import yaml
import random

#navigation imports
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        #publisher for movement
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)

        #for box detection - include sensitivity and distance from centering on object
        self.blue_flag = False
        self.blue_area = 0
        self.error_x = 0
        self.sensitivity = 10

        #movement flags
        self.forward_flag = False
        self.back_flag = False
        self.stop_flag = False
        
        #brige to camera subscriber
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription
        
        #get the safe map points for movement
        yaml_path = '/uolstore/home/users/sc23s2e/ros2_ws/src/ros2_project_sc23s2e/map/map.yaml'
        pgm_path = '/uolstore/home/users/sc23s2e/ros2_ws/src/ros2_project_sc23s2e/map/map.pgm'
        self.safe_points_x, self.safe_points_y = self.get_safe_map_points(yaml_path, pgm_path)
        
        #store latest image
        self.latest_image = None
        
        #navigation state flags
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.is_navigating = False
        self.goal_handle = None

    def callback(self, data):
        #convert to viewable image versoin & hsv image
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        #reset fglag
        self.blue_flag = False

        #colur sesitivities to detecdt colour from hue
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        hsv_red_lower = np.array([0, 100, 100])
        hsv_red_upper = np.array([self.sensitivity, 255, 255])
        hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
        hsv_red_upper2 = np.array([180, 255, 255])
        
        # Filter out everything but a particular colour using the cv2.inRange() method
        red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        red_mask2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
        r_mask = cv2.bitwise_or(red_mask, red_mask2)
        
        #detect blue contour
        contours, _ = cv2.findContours(blue_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)>0:
            #test if contour big enough to be considered
            c = max(contours, key=cv2.contourArea)
            self.blue_area = cv2.contourArea(c)
            if self.blue_area > 500:
                self.blue_flag = True
                
                #create a bounding box for detection
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                
                # Alter the value of the flag
                self.blue_flag = True
                
                #use moments to detects the centre of mass and point towards it
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    #how far from centre of masss it is
                    self.error_x = cx - 160.0
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                    
        #save latest image
        self.latest_image = cv_image
        
        #repeat for red - only need to detect
        contours, _ = cv2.findContours(r_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)>0:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: 
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                
        contours, _ = cv2.findContours(green_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)>0:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: 
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        
    #function to return the safe map points - avoid walls
    def get_safe_map_points(self, yaml_path, pgm_path):
        try:
            #load the meta data to get the origin point
            with open(yaml_path, 'r') as file:
                map_data = yaml.safe_load(file)
                
            self.resolution = map_data['resolution']
            self.origin_x = map_data['origin'][0]
            self.origin_y = map_data['origin'][1]
            
            #load pgm with cv2 - open as grayscale
            map_image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)  
            self.map_h, self.map_w = map_image.shape
                 
            kernel = np.ones((7,7), np.uint8)
            #reduce by any walls
            reduced_map = cv2.erode(map_image, kernel, iterations=1)
            
            #find all unoccupied pixels (white pixels)
            free_space_y, free_space_x = np.where(reduced_map >= 250)
            return free_space_x, free_space_y
        except Exception as e:
            self.get_logger().error(f"Could not load/reduce map: {e}")
            return [], []
        
    #find a random point to move to 
    def send_random_nav_goal(self):
        #if no map loaded
        if len(self.safe_points_x) == 0:
            return
        
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Nav2 Server')
            return
        
        #pick a random pixel to move to
        idx = random.randint(0, len(self.safe_points_x) - 1)
        px, py = self.safe_points_x[idx], self.safe_points_y[idx]
        
        #convert to world location (link to origin and flip y)
        target_x = self.origin_x + (px * self.resolution)
        target_y = self.origin_y + ((self.map_h - py) * self.resolution)
        self.get_logger().info(f"Navigating to random point: X={target_x:.2f}, Y={target_y:.2f}")
        
        #create and send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.orientation.w = 1.0
        
        #link to callback and wait
        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True
        
    #triggers to either accept or reject the goal by nav2
    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Nav2 rejected goal - need to retry')
            self.is_navigating = False
            return
        
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        
    #triggerst when at destination
    def get_result_callback(self, future):
        self.get_logger().info("Arrived at random point")
        self.is_navigating = False
        
# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args=None):
    # Instantiate your class
    rclpy.init(args=args)
    robot = Robot()
    
    #createe a opencv window
    cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('camera_Feed',320,240)

    #loop for control
    try:
        while rclpy.ok():
            #listen to camera to trigger callback
            rclpy.spin_once(robot, timeout_sec=0.1)
            
            #empty twist message
            twist = Twist()
            
            #check if blue is visible 
            if robot.blue_flag:
                #cancel navigation
                if robot.is_navigating and robot.goal_handle:
                    robot.goal_handle.cancel_goal_async()
                    robot.is_navigating = False
                    robot.get_logger().info(f'Cancelling Navigation')
                
                robot.get_logger().info(f'Blue spotted with area {robot.blue_area}')
                
                #want to steer towards it 
                twist.angular.z = -float(robot.error_x) / 300.0
                
                #determine distance by size of object
                if robot.blue_area < 40000:
                    twist.linear.x = 0.15
                elif robot.blue_area > 70000:
                    twist.linear.x = -0.15
                else:
                    twist.linear.x = 0.0
                    
                #publish movement
                robot.publisher.publish(twist)
            
            #navigate across the map
            else:
                #if no point - send a new one
                if not robot.is_navigating:
                    robot.send_random_nav_goal()
            
            #update camera
            if robot.latest_image is not None:
                cv2.imshow('camera_Feed', robot.latest_image)
                cv2.waitKey(1)
            
    except KeyboardInterrupt:
        pass
    
    #cleanly stop robot and exit
    finally:
        stop_msg = Twist()
        robot.publisher.publish(stop_msg)
        robot.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
    
        
if __name__ == '__main__':
    main()