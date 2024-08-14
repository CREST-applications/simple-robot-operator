import rclpy
from simple_robot_operator.simple_robot_operator.operator import SimpleOperator


def main():
    try:
        rclpy.init()
        operator = SimpleOperator("simple_operator")
        operator.forward(-0.01, 3.0)
        operator.forward(0.01, 3.0)

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
