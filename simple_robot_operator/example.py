import rclpy
from simple_robot_operator import SimpleOperator, Angle


def main():
    try:
        rclpy.init()

        operator = SimpleOperator("operator_node")

        # Do not use rclpy.spin() in a script
        # rclpy.spin(operator)

        # basic methods

        operator.forward(0.1, 3.0)  # move forward 0.1 m/s for 3 seconds
        operator.forward(-0.1, 5.0)  # move backward 0.1 m/s for 5 seconds

        operator.rotate_by(-90, 5.0)  # rotate left 90 degrees for 5 seconds
        operator.rotate_by(Angle.RIGHT, 3.0)  # rotate right (90 degrees) for 3 seconds

        # extra methods

        operator.set_speed(0.1)  # set speed to 0.1 m/s
        operator.add_speed(0.1)  # add 0.1 m/s to the current speed

        current_speed = operator.speed  # get current speed
        print(f"Current speed: {current_speed}")

        x, y = operator.pos  # get current position
        print(f"Current position: ({x}, {y})")

        operator.stop()  # stop the robot

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
