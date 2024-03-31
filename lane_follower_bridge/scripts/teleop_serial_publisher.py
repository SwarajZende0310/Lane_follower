#! usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class ArduinoSerialPublisher(Node):
    def __init__(self):
        super().__init__('arduino_serial_publisher')
        self.serial_port = '/dev/ttyACM0'  # Adjust this based on your Arduino's serial port
        self.serial_baudrate = 9600  # Adjust this based on your Arduino's baudrate
        self.serial = serial.Serial(self.serial_port, self.serial_baudrate, timeout=1)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            1000
        )
        #self.get_logger().info('Arduino Serial Publisher Node Initialized')
        self.get_logger().info(f'Serial Port: {self.serial_port} at Baud rate of {self.serial_baudrate}')
        self.linear_x = 0.0
        self.angular_z = 0.0

        # self.create_timer(0.5, self.serial_pub)

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        cmd_string = f"{self.linear_x},{self.angular_z}\n"
        self.serial.write(cmd_string.encode('utf-8'))
        self.get_logger().info(f'Sending command: {cmd_string}')
        self.linear_x = 0.0
        self.angular_z = 0.0
    
    def serial_pub(self):
        cmd_string = f"{self.linear_x},{self.angular_z}\n"
        self.serial.write(cmd_string.encode('utf-8'))
        self.get_logger().info(f'Sending command: {cmd_string}')
        self.linear_x = 0.0
        self.angular_z = 0.0

def main(args=None):
    rclpy.init(args=args)
    arduino_serial_publisher = ArduinoSerialPublisher()
    rclpy.spin(arduino_serial_publisher)
    arduino_serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()