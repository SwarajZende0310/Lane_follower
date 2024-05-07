#! usr/bin/env python3
import cv2
import numpy as np
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class cmd_vel_pub(Node):
    def __init__(self):
        super().__init__('cmd_vel_pub')
        
        self.linear_vel = 0.4
        self.angular_vel = 0.3
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def velocity_pub(self,direction:str):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = 0.0
        if(direction == 'LEFT'):
            msg.angular.z = self.angular_vel
        elif(direction == 'RIGHT'):
            msg.angular.z = -self.angular_vel
        self.publisher_.publish(msg)
    
    def sweep(self):
        pass

def mask_yellow(frame):
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  lower_yellow = np.array([16,105,76])
  upper_yellow = np.array([255,255,255])

  mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

  masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

  return masked_frame


def main():
    rclpy.init()
    velocity_publisher = cmd_vel_pub()

    cap = cv2.VideoCapture(2)
    
    theta=0
    minLineLength = 5
    maxLineGap = 10

    while True:
        ret,frame = cap.read()

        image = frame
        image = cv2.resize(image,(500,300))
        gray = mask_yellow(frame)
        cv2.imshow('gray',gray)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 85, 85)
        lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)
        if lines is not None:
            for x in range(0, len(lines)):
                for x1,y1,x2,y2 in lines[x]:
                    cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
                    theta=theta+math.atan2((y2-y1),(x2-x1))
                    print(theta)
        else:
            print('Line not detected. Enter Sweep')

        threshold=6
        if(theta>threshold):
            print("left")
            velocity_publisher.velocity_pub('LEFT')
        if(theta<-threshold):
            print("right")
            velocity_publisher.velocity_pub('RIGHT')
        if(abs(theta)<threshold):
            print("straight")
            velocity_publisher.velocity_pub('STRAIGHT')

        theta = 0

        cv2.imshow('frame',frame)
        if cv2.waitKey(1)==ord('q'):
            break

    cv2.destroyAllWindows()
    velocity_publisher.destroy_node()

if __name__ == '__main__':
    main()