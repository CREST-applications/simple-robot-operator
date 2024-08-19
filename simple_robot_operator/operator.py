import time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from enum import IntEnum
import math
from typing import Optional


class Angle(IntEnum):
    LEFT = 90
    RIGHT = -90
    INVERSE = 180


class SimpleOperator(Node):
    def __init__(self, name: str):
        """SimpleOperator

        Args:
            name (str): Name of the node
        """

        super().__init__(name)

        self.__twist = Twist()
        self.__publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.__odom: Odometry
        self.create_subscription(Odometry, "/odom", self.__odom_callback, 10)

    def __odom_callback(self, twist: Odometry):
        self.__odom = twist
        print(f"Odom: {self.__odom}")

    # def increase_speed(self, speed: float):
    #     speed = self.__odom.twist.twist.linear.x + speed
    #     self.__publisher.publish(speed)
    #     self._logger.info(f"Increased speed to: {speed}")

    # def enhance_rotation(self, angle: int):
    #     self.__twist.angular.z = self.__odom.twist.twist.angular.z + angle
    #     self.__publisher.publish(self.__twist)
    #     self._logger.info(f"Enhanced rotation to: {angle}")

    def set_speed(self, speed: float):
        twist = Twist()
        twist.linear.x = speed

        self.__publisher.publish(twist)
        self._logger.debug(f"Set speed to: {speed}")

    def set_angle(self, angle: int):
        degree = angle * (math.pi / 180)

        twist = Twist()
        twist.angular.z = degree

        self.__publisher.publish(twist)
        self._logger.debug(f"Set angle to: {angle}")

    def forward(self, speed: float, duration: Optional[float] = None):
        """forward

        Args:i
            speed (float): Speed of the robot
            duration (Optional[float], optional): Duration of the movement. Defaults to None.
        """

        if 0.2 < speed:
            self._logger.warning("Speed is too high. Setting to 0.2")
            speed = 0.2

        if speed < -0.2:
            self._logger.warning("Speed is too high. Setting to -0.2")
            speed = -0.2

        self.__twist.linear.x = float(speed)
        self.__twist.linear.y = 0.0
        self.__twist.linear.z = 0.0

        self.__twist.angular.x = 0.0
        self.__twist.angular.y = 0.0
        self.__twist.angular.z = 0.0

        self.__publisher.publish(self.__twist)

        self._logger.info(
            f"Published: Linear x: {self.__twist.linear.x}, Angular z: {self.__twist.angular.z}"
        )

        if duration is not None:
            time.sleep(duration)
            self.stop()

    def rotate(self, angle: int, duration: float):
        # def rotate(self, speed, duration):
        """rotate

        Args:
            angle (int): Angle to rotate
            duration (float): Duration of the rotation
        """

        rotation_speed = (angle * (math.pi / 180)) / duration

        # Linear and Angular values for rotation
        self.__twist.linear.x = 0.0
        self.__twist.linear.y = 0.0
        self.__twist.linear.z = 0.0

        self.__twist.angular.x = 0.0
        self.__twist.angular.y = 0.0
        self.__twist.angular.z = rotation_speed

        # First publish to robot for rotation
        self.__publisher.publish(self.__twist)

        # Wait for the duration of rotation
        self._logger.info(
            f"Published: Linear x: {self.__twist.linear.x}, Angular z: {self.__twist.angular.z}"
        )

        print(f"Duration: {duration}")
        time.sleep(duration)

        # Stop the robot rotation
        self.stop()

    def stop(self):
        self.__twist.linear.x = 0.0
        self.__twist.angular.z = 0.0
        self.__publisher.publish(self.__twist)
        self._logger.info("Stopping the robot")
