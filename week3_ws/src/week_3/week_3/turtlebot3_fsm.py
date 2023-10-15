import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
from enum import Enum
from rclpy.qos import qos_profile_sensor_data
import math
class State(Enum):
    FORWARD = 0
    TURNING = 1

class TurtleBot3FSM(Node):

    def __init__(self):
        super().__init__('turtlebot3_fsm')

        self.state = State.FORWARD
        self.counter = 0

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.ranges = []
    
    def scan_callback(self, msg):
        self.ranges = msg.ranges



    def control_loop(self):
        if self.ranges:
            if any(r < 0.5 for r in self.ranges[0:10] + self.ranges[-10:]):
                self.state = State.TURNING
                self.counter = 0
                self.get_logger().info("Obstacle detected!")
            match self.state:
                case State.FORWARD:
                    
                    min_distance_obstacle = min(self.ranges[0:10] + self.ranges[-10:])
                    if (min_distance_obstacle > 1.5):
                        speed = 0.5
                    elif (min_distance_obstacle > 1.0):
                        speed = 0.3
                    elif (min_distance_obstacle > 0.5):
                        speed = 0.2
                    else:
                        speed = 0.1
                    msg = Twist()
                    msg.linear.x = speed
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Forward: {msg}")
                    if self.counter > 4 * (1 / self.timer_period):
                        self.counter = 0
                    self.counter += 1
                case State.TURNING:
                    msg = Twist()
                    max_index = self.ranges.index(max(self.ranges))
                    target_angle = math.pi * (max_index / len(self.ranges))
                    kp = 2.0
                    msg.angular.z = kp * target_angle
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Turning: {msg.angular.z}")
                    if self.counter > 1 * (1 / self.timer_period):
                        self.state = State.FORWARD
                        self.counter = 0
                    self.counter += 1
                case _:
                    pass

def main(args=None):
    rclpy.init(args=args)

    turtlebot3_fsm = TurtleBot3FSM()

    rclpy.spin(turtlebot3_fsm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtlebot3_fsm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()