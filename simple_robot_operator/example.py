import rclpy
from simple_robot_operator import SimpleOperator, Angle
import threading


class Example(SimpleOperator):
    def __init__(self, name: str = "operator_node"):
        super().__init__(name)

        threading.Thread(target=self.operate, daemon=True).start()

    def operate(self):
        # forward, backward

        print(self.pos)  # Position(x=0.0, y=0.0)
        self.forward(0.3, 3.0)  # move forward 0.3 m for 3 sec (0.1 m/s)

        print(self.pos)  # Position(x~=0.3, y=0.0)
        self.forward(-0.3, 3.0)  # move backward 0.3 m for 3 sec (-0.1 m/s)

        # rotate

        self.rotate_by(90, 5.0) # rotate 90 degree for 5 sec (18 deg/s)
        self.rotate_by(Angle.LEFT, 5.0) # rotate 90 degree for 5 sec (18 deg/s)


def main():
    try:
        rclpy.init()

        operator = Example("operator_node")

        rclpy.spin(operator)

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
