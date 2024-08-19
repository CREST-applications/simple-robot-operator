import rclpy
from simple_robot_operator import SimpleOperator


def main():
    try:
        rclpy.init()
        operator = SimpleOperator("operator_node")
        operator.forward(-0.1, 3.0)
        operator.forward(0.1, 3.0)

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
