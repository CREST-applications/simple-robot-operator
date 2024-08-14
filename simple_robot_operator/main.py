import rclpy
from .operator import SimpleOperator
import math

def main():
    operator: SimpleOperator
    try:
        rclpy.init()
        operator = SimpleOperator("simple_operator")
        operator.forward(-0.01, 3.0)
        operator.forward(0.01, 3.0)
        operator.rotate(360 * 3, 3.0 * 3)
        print("Rotated")

        operator.forward(0.1, None)
        input = input("some words")
        operator.stop()

        operator.rotate(360, 3.0)
        input = input("some words")
        operator.stop()

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
