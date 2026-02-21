"""
Node for avoiding obstacles.
"""

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# Helpers
def is_valid(idx):
    """
    An helper function that filters out 0.0 and inf values on laser scan which are invalid values.
    """

    if idx != 0.0 and idx != math.inf:
        return idx


class ObstacleAvoidanceNode(Node):
    """
    Obstacle avoidance node that publishes Twist messages and listenes LaserScan message.
    """

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Pub
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Sub
        self.sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Other variables
        # Twist
        self.linear_speed = 0.3
        self.angular_speed = 0.5

        # LaserScan
        self.ranges = list()
        self.range_min = 0.12                    # LIDAR can't measure closer than 0.12 m (12 cm)
        self.range_max = 3.5                     # LIDAR can't measure further than 3.5 m
        self.angle_min = 0.0                     # In front of the robot
        self.angle_max = 6.28                    # 360°
        self.angle_increment = 0.01749           # 0.0175 rad ≈ 1°

    
    def scan_callback(self, msg):
        """
        A callback funtion for obstacle avoidance.
        """

        # Define message to publish and safe distance threshold
        msg_pub = Twist()
        safe_distance = 0.5
        
        # Identify front, right and left indexes by slicing degrees in ranges
        # In tb3 LIDAR
        front = msg.ranges[0:30] + msg.ranges[330:360]
        right = msg.ranges[270:330]
        left = msg.ranges[30:90]
        # Normally, index = (angle - angle_min) / angle_increment

        # Filter out invalid values
        front = list(filter(is_valid, front))
        right = list(filter(is_valid, right))
        left = list(filter(is_valid, left))

        # Threshold min distances by angle
        front_min = min(front) if front else self.range_max
        right_min = min(right) if right else self.range_max
        left_min = min(left) if left else self.range_max

        # Decision logic
        if front_min > safe_distance:
            # No obstacle, go forward
            msg_pub.linear.x = self.linear_speed
        else:
            # Obstacle, turn
            if left_min > right_min:
                # Turn left
                msg_pub.angular.z = self.angular_speed
            else:
                # Turn right
                msg_pub.angular.z = -self.angular_speed

        # Publish the message
        self.pub.publish(msg_pub)


def main():
    """
    Main function that starts and ends node lifecycle.
    """

    rclpy.init()
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()