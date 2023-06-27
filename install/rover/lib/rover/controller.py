#!/usr/bin/env python3

import cv2 as cv

import rclpy
from rclpy.node import Node
from rover.brain import movement
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class Controller(Node):
    
    def __init__(self):
        super().__init__('controller')
        
        # Initialize the video capture object
        self.cap_ = cv.VideoCapture(0)
        
        # Create a timer that calls the timer_callback function every 0.1 seconds
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        
        # Create a publisher for sending Twist messages to the 'Scmd_vel' topic
        self.controller_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a publisher for sending Image messages to the 'frames' topic
        self.image_publisher_ = self.create_publisher(Image, "frames", 10)
        
        # Create a subscriber for receiving Image messages from the 'frames' topic
        self.image_subscriber_ = self.create_subscription(Image, "frames", self.show_image, 10)
        
        # Initialize the Twist message and CvBridge object
        self.velocity_ = Twist()
        self.bridge_ = CvBridge()
        
        # Clearing the contents of the history file
        self.history_ = open("history.txt", "w").close()
        
    def timer_callback(self):
        # Read a frame from the video capture object
        success, image = self.cap_.read()
        
        # Opening a file to append the history of movements
        self.history_ = open("history.txt", "a")
        
        if success:      
            # Set the velocity based on the direction of movement
            match movement(image):
                case "left":
                    self.velocity_.linear.x = 1.0
                    self.velocity_.angular.z = 1.0
                    self.history_.write("left\n")
                case 'sleft':
                    self.velocity_.linear.x = 0.0
                    self.velocity_.angular.z = 1.0
                    self.history_.write("straight and left\n")
                case 'right':
                    self.velocity_.linear.x = 1.0
                    self.velocity_.angular.z = -1.0
                    self.history_.write("right\n")
                case 'sright':
                    self.velocity_.linear.x = 0.0
                    self.velocity_.angular.z = -1.0
                    self.history_.write("straight and right\n")
                case 'straight':
                    self.velocity_.linear.x = 1.0
                    self.velocity_.angular.z = 0.0
                    self.history_.write("straight\n")
                case 'stop':
                    self.velocity_.linear.x = 0.0
                    self.velocity_.angular.z = 0.0
                    self.history_.write("stop\n")
                
            # Publish the Twist message to the '/cmd_vel' topic
            self.controller_.publish(self.velocity_)
            image = cv.flip(image, 1)

            # Convert the image to a ROS message and publish it to the 'frames' topic
            self.image_publisher_.publish(self.bridge_.cv2_to_imgmsg(image))
            
        self.history_.close()
            
    def show_image(self, data):
        ''' Displays the image received from the 'frames' topic '''
        
        # Convert the ROS message to an OpenCV image
        frame = self.bridge_.imgmsg_to_cv2(data)

        # Display the image
        cv.imshow('Controller', frame)
        cv.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    