'''import rclpy
from rclpy.node import Node

import spidev
import RPi.GPIO as GPIO

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from .adis16460 import *

from collections import deque
from rclpy.node import Node

class ImuPublisher(Node):

    def __init__(self):
        super().__init__('ImuPublisher')
        
        self.get_logger().info('Creating ImuPublisher')
        # SPI Config
        dev = 0
        spi_freq = 1000000
        spi_mode = 3
        
        # Device 0
        cs0 = 0
        dr0 = 25
        rst0 = 12
        
        # Device 1
        cs1 = 1
        dr1 = 26
        rst1 = 13
        
        self.dev0 = ADIS16460(dev, cs0, spi_mode, spi_freq, dr0, rst0)
        self.dev1 = ADIS16460(dev, cs1, spi_mode, spi_freq, dr1, rst1)
        
        self.dev0_publisher = self.create_publisher(Imu, 'adis16460_dev0', 10)
        self.dev1_publisher = self.create_publisher(Imu, 'adis16460_dev1', 10)
    
    
        size = 1000
        self.dev0_data = np.zeros(6) # deque(maxlen=size)
        self.dev1_data = np.zeros(6) # deque(maxlen=size)
        self.dev0_var = np.zeros(6)
        self.dev1_var = np.zeros(6)
        
        
        self.alpha = 0.99
    
        self.read()
        # self.timer = self.create_timer(1e-9, self.read)

    def read(self):
        self.get_logger().info('Starting SPI reading')
        while True:
            # GPIO.wait_for_edge(self.dev0.dr, GPIO.RISING)
            dev0_read = self.dev0.read()
            if dev0_read is not None:                
                self.dev0_data = self.alpha * self.dev0_data + (1 - self.alpha) * dev0_read
                self.dev0_var = self.alpha * self.dev0_var + (1 - self.alpha) * (self.dev0_data - dev0_read) ** 2
                dev0_msg = self.create_msg(dev0_read, self.dev0_var, 'imu0')
                self.dev0_publisher.publish(dev0_msg)

            # GPIO.wait_for_edge(self.dev0.dr, GPIO.RISING)
            dev1_read = self.dev1.read()
            if dev1_read is not None:                
                self.dev1_data = self.alpha * self.dev1_data + (1 - self.alpha) * dev1_read
                self.dev1_var = self.alpha * self.dev1_var + (1 - self.alpha) * (self.dev1_data - dev1_read) ** 2
                dev1_msg = self.create_msg(dev1_read, self.dev1_var, 'imu1')
                self.dev1_publisher.publish(dev1_msg)


    def create_msg(self, data, var_data, child_frame='imu'):
        
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = child_frame

        msg.angular_velocity.x = data[0]
        msg.angular_velocity.y = data[1]
        msg.angular_velocity.z = data[2]
        msg.angular_velocity_covariance = [var_data[0], 0., 0.,
                                           0., var_data[1], 0.,
                                           0., 0., var_data[2]]
        
        msg.linear_acceleration.x = data[3]
        msg.linear_acceleration.y = data[4]
        msg.linear_acceleration.z = data[5]
        msg.linear_acceleration_covariance = [var_data[3], 0., 0.,
                                              0., var_data[4], 0.,
                                              0., 0., var_data[5]]

        return msg


def main(args=None):
    rclpy.init(args=args)

    GPIO.setwarnings(False)
    publisher = ImuPublisher()
    
    rclpy.spin(publisher)

    publisher.destroy_node()
    
    GPIO.cleanup()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()'''


import rclpy
from rclpy.node import Node

import spidev
import RPi.GPIO as GPIO
import numpy as np
import time

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from .adis16460 import *

class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        
        self.get_logger().info('Initializing ImuPublisher node')
        
        # SPI Config
        dev = 0
        spi_freq = 1000000
        spi_mode = 3
        
        # Device 0
        cs0, dr0, rst0 = 0, 25, 12
        
        # Device 1
        cs1, dr1, rst1 = 1, 26, 13

        # log rate
        self.log_counter = 0
        self.log_rate = 500
        
        self.dev0 = ADIS16460(dev, cs0, spi_mode, spi_freq, dr0, rst0)
        self.dev1 = ADIS16460(dev, cs1, spi_mode, spi_freq, dr1, rst1)
        
        self.dev0_publisher = self.create_publisher(Imu, 'adis16460_dev0', 100)
        self.dev1_publisher = self.create_publisher(Imu, 'adis16460_dev1', 100)
        
        self.dev0_data = np.zeros(6)
        self.dev1_data = np.zeros(6)
        self.dev0_var = np.zeros(6)
        self.dev1_var = np.zeros(6)
        
        self.alpha = 0.99
        
        self.timer = self.create_timer(0.002, self.read)
        self.get_logger().info('Timer initialized with 10ms interval')


    def safe_read(self, device, device_name):
        max_retries = 10
        for attempt in range(max_retries):
            data = device.read()
            if data is not None:
                return data
            self.get_logger().warn(f"Read failed for {device_name}, retrying {attempt + 1}/{max_retries}")
            time.sleep(0.001)  # Small delay before retrying
        self.get_logger().error(f"Device {device_name} failed to respond after {max_retries} retries.")
        return None
    
    def read(self):
        self.log_counter += 1
         
        start_time = time.time()
        dev0_read = self.safe_read(self.dev0, 'imu0')
        end_time = time.time()
        if dev0_read is not None:
            self.dev0_data = self.alpha * self.dev0_data + (1 - self.alpha) * dev0_read
            self.dev0_var = self.alpha * self.dev0_var + (1 - self.alpha) * (self.dev0_data - dev0_read) ** 2
            dev0_msg = self.create_msg(dev0_read, self.dev0_var, 'imu0')
            self.dev0_publisher.publish(dev0_msg)
            if self.log_counter % self.log_rate == 0:
                self.get_logger().info(f'Published IMU data for device 0 (Read time: {end_time - start_time:.6f} sec)')

        start_time = time.time()
        dev1_read = self.safe_read(self.dev1, 'imu1')
        end_time = time.time()
        if dev1_read is not None:
            self.dev1_data = self.alpha * self.dev1_data + (1 - self.alpha) * dev1_read
            self.dev1_var = self.alpha * self.dev1_var + (1 - self.alpha) * (self.dev1_data - dev1_read) ** 2
            dev1_msg = self.create_msg(dev1_read, self.dev1_var, 'imu1')
            self.dev1_publisher.publish(dev1_msg)
            if self.log_counter % self.log_rate == 0:
                self.get_logger().info(f'Published IMU data for device 1 (Read time: {end_time - start_time:.6f} sec)')

    def create_msg(self, data, var_data, child_frame='imu'):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = child_frame

        msg.angular_velocity.x = data[0]
        msg.angular_velocity.y = data[1]
        msg.angular_velocity.z = data[2]
        msg.angular_velocity_covariance = [var_data[0], 0., 0.,
                                           0., var_data[1], 0.,
                                           0., 0., var_data[2]]
        
        msg.linear_acceleration.x = data[3]
        msg.linear_acceleration.y = data[4]
        msg.linear_acceleration.z = data[5]
        msg.linear_acceleration_covariance = [var_data[3], 0., 0.,
                                              0., var_data[4], 0.,
                                              0., 0., var_data[5]]

        return msg


def main(args=None):
    rclpy.init(args=args)

    GPIO.setwarnings(False)
    publisher = ImuPublisher()
    
    rclpy.spin(publisher)

    publisher.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
