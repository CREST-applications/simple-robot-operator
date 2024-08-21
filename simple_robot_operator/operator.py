import time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from enum import IntEnum
import math

# import threading
import rclpy


class Angle(IntEnum):
    LEFT = -90
    RIGHT = 90
    INVERSE = 180


class Position:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


class SimpleOperator(Node):
    def __init__(self, name: str):
        """SimpleOperator

        Args:
            name (str): Name of the node
        """

        # Initialize the node
        super().__init__(name)

        # Create a publisher for Twist
        self.__pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Create a Subscriber for Odometry
        self.__odom: Odometry = None
        self.create_subscription(Odometry, "/odom", self.__odom_callback, 10)

        # Twist
        self.__twist = Twist()

        # initial position
        self.__pos_init: Position = None

        # Thread for subscribing to Odometry
        # threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

        # Wait for receiving the Odometry message
        self.__wait_for_odom()

    def __odom_callback(self, odom: Odometry):
        self.__odom = odom
        self._logger.debug(f"Subscribe Odometry: {odom}")

    # getters

    @property
    def odom(self) -> Odometry:
        """odom

        Odometry is a message type that contains the position and velocity of the robot.
        You can use this value to control the robot in a more precise way.

        Returns:
            Odometry: Odometry message
        """

        return self.__odom

    @property
    def pos(self) -> Position:
        """pos

        Position of the robot in the world frame.

        Returns:
            Position: position of the robot in meters
        """

        return Position(
            self.__pos_init.x - self.__odom.pose.pose.position.x,
            self.__pos_init.y - self.__odom.pose.pose.position.y,
        )

    @property
    def speed(self) -> float:
        """speed

        Returns:
            float: Speed of the robot in m/s
        """

        return self.__odom.twist.twist.linear.x

    @property
    def angle_velocity(self) -> float:
        """angle_velocity

        ~~Angle of the robot in **degrees**.~~
        ~~The value is based on the initial position of the robot.~~

        The angle velocity of the robot in **degrees**.

        Returns:
            float: Angle of the robot in **degrees**
        """

        return math.degrees(self.__odom.twist.twist.angular.z)

    # setters

    def set_speed(self, speed: float) -> "SimpleOperator":
        """set_speed

        Args:
            speed (float): Speed of the robot in m/s

        Returns:
            SimpleOperator: Instance of the SimpleOperator
        """

        speed = self.__validate_speed(speed)
        self.__twist.linear.x = float(speed)

        self.__pub.publish(self.__twist)
        self._logger.info(f"Set speed to: {speed} m/s")

        return self

    def set_angle_velocity(self, degree: int) -> "SimpleOperator":
        """set_angle_velocity

        Args:
            degree (int): Angle velocity in degree/s

        Returns:
            SimpleOperator: Instance of the SimpleOperator
        """

        self.__twist.angular.z = math.radians(float(degree))
        self.__pub.publish(self.__twist)
        self._logger.info(f"Set angle velocity to: {degree} degree/s")

        return self

    # adders

    def add_speed(self, speed: float) -> "SimpleOperator":
        """add_speed

        Args:
            speed (float): Speed to add to the current speed

        Returns:
            SimpleOperator: Instance of the SimpleOperator
        """

        current = self.speed
        self.set_speed(current + speed)

        return self

    def add_angle_velocity(self, degree: int) -> "SimpleOperator":
        """add_angle_velocity

        Args:
            degree (int): Angle velocity to add to the current angle velocity

        Returns:
            SimpleOperator: Instance of the SimpleOperator
        """

        current = self.angle_velocity
        self.set_angle_velocity(current + degree)

        return self

    # utils

    def forward(self, distance: float, duration: float) -> "SimpleOperator":
        """forward

        You can move the robot forward by providing the distance and duration of the movement.
        When the movement is finished, the robot will automatically stop.

        Args:
            distance (float): Distance to move forward in meters
            duration (float): Duration of the movement in seconds

        Returns:
            SimpleOperator: Instance of the SimpleOperator
        """

        speed = distance / duration

        start_pos = self.pos

        while True:
            self.set_speed(speed)

            current = self.pos
            distance_moved = math.sqrt(
                (current.x - start_pos.x) ** 2 + (current.y - start_pos.y) ** 2
            )

            if math.fabs(distance) <= distance_moved:
                print("break")
                break

        self.stop()

    def rotate_by(self, degree: int, duration: float) -> "SimpleOperator":
        """rotate_by

        You can rotate the robot by providing the degree and duration of the rotation.
        When the movement is finished, the robot will automatically stop.

        Args:
            degree (int): Degree to rotate
            duration (float): Duration of the rotation

        Returns:
            SimpleOperator: Instance of the SimpleOperator
        """

        velocity = degree / duration

        self.set_angle_velocity(velocity)
        time.sleep(duration)
        self.stop()

        self._logger.info(f"Rotated by: {degree} degrees")

        return self

    def stop(self):
        """stop

        Stop the robot by setting the speed and angle velocity to 0.
        """

        self.__twist.linear.x = 0.0
        self.__twist.angular.z = 0.0

        self.__pub.publish(self.__twist)

    # others

    def __validate_speed(self, speed: float) -> float:
        if 0.2 < speed:
            self._logger.warning("Speed is too high. Setting to 0.2")
            return 0.2

        if speed < -0.2:
            self._logger.warning("Speed is too high. Setting to -0.2")
            return -0.2

        return speed

    def __wait_for_odom(self):
        self._logger.info("Waiting for /odom topic")

        while not self.__odom:
            time.sleep(1)
            rclpy.spin_once(self)

        self.__pos_init = Position(
            self.__odom.pose.pose.position.x,
            self.__odom.pose.pose.position.y,
        )

        self._logger.info("Received /odom topic. Ready to move the robot")
