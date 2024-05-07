#! usr/bin/env python3
import cv2
import numpy as np
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Lane_follower_node(Node):
    def __init__(self):
        super().__init__('lane_follower_node')
    
        self.theta=0
        self.minLineLength = 5
        self.maxLineGap = 10

        self.error = 0.0

        self.linear_vel = 0.5
        self.angular_vel = 0.5
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cap = cv2.VideoCapture(2)
        
        self.timer = self.create_timer(0.05,self.driver_function)

    def driver_function(self):
        ret,frame = self.cap.read()

        image = frame
        image = cv2.resize(image,(500,300))
        gray = mask_yellow(frame)
        cv2.imshow('gray',gray)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 85, 85)
        lines = cv2.HoughLinesP(edged,1,np.pi/180,10,self.minLineLength,self.maxLineGap)
        if lines is not None:
            for x in range(0, len(lines)):
                for x1,y1,x2,y2 in lines[x]:
                    cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
                    self.theta=self.theta+math.atan2((y2-y1),(x2-x1))
                    print(self.theta)
        else:
            print('Line not detected. Enter Sweep')
            self.sweep()
            return

        threshold=6
        if(self.theta>threshold):
            self.error = self.error - 1
            if(self.error < -10):
                self.error = -10
        if(self.theta<-threshold):
            self.error = self.error + 1
            if(self.error > 10):
                self.error = 10
        if(abs(self.theta)<threshold):
            if(self.error < 0):
                self.error = self.error + 1
            elif(self.error > 0):
                self.error = self.error - 1

        if(self.error < -3):
            print("left")
            self.velocity_pub('LEFT')
        elif(self.error > 3):
            print("right")
            self.velocity_pub('RIGHT')
        else:
            print("straight")
            self.velocity_pub('STRAIGHT')

        self.theta = 0

        cv2.imshow('frame',frame)
        if cv2.waitKey(1)==ord('q'):
            cv2.destroyAllWindows()
            exit()

    def velocity_pub(self,direction:str):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = 0.0
        if(direction == 'LEFT'):
            msg.angular.z = self.angular_vel
        elif(direction == 'RIGHT'):
            msg.angular.z = -self.angular_vel
        self.cmd_vel_pub.publish(msg)
    
    def sweep(self):
        msg = Twist()
        # msg.linear.x = 0.0
        # msg.angular.z = 0.0
        # self.cmd_vel_pub.publish(msg)
        # return 
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.cmd_vel_pub.publish(msg)
        time.sleep(1)
        msg.angular.z = -self.angular_vel
        self.cmd_vel_pub.publish(msg)
        time.sleep(1)
        return 

def mask_yellow(frame):
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  lower_yellow = np.array([16,105,76])
  upper_yellow = np.array([255,255,255])

  mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

  masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

  return masked_frame


def main():
    rclpy.init()
    lane_follower_node = Lane_follower_node()
    rclpy.spin(lane_follower_node)
    cv2.destroyAllWindows()
    lane_follower_node.destroy_node()

if __name__ == '__main__':
    main()