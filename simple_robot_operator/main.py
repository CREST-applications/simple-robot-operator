import rclpy
from .operator import SimpleOperator


def main():
    operator: SimpleOperator
    try:
        rclpy.init()
        operator = SimpleOperator("simple_operator")
        operator.forward(-0.01, 3.0)
        operator.increase_speed(0.05)
        operator.set_speed(0.1)
        operator.forward(0.01, 3.0)
        # operator.rotate(360 * 3, 3.0 * 3)

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
