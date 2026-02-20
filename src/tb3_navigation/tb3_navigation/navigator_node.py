"""
Navigation node that goes to predefined waypoints across the map.
"""

import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


# Helper functions
def create_pose(navigator, x, y, yaw):
    """
    A helper function that creates pose using PoseStamped message.
    """

    # Create pose and assign values to it's fields
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0  # For yaw = 0 ---- Actual formula (math.cos(yaw/2))

    return pose


# Main function
def main():
    """
    Using nav2_simple_commander API with predefined waypoints.
    """

    # Initialize rclpy
    rclpy.init()

    # Define arbitrary waypoints behind -2.5, 2.5 also in (x, y, yaw) format
    waypoints = [
        (0.5, -0.5, 0.0),
        (1.5,  0.5, 0.0),
        (0.0,  1.0, 0.0),
        (-1.0, 0.5, 0.0),
        (-1.9, -0.5, 0.0),
    ]

    # Define navigator
    nav = BasicNavigator()

    # Set initial pose and wait until nav2 is active
    initial_pose = create_pose(nav, -1.9, -0.5, 0.0)
    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()

    # After nav2 is active, iterate trough waypoints
    for waypoint in waypoints:
        # Create goal pose
        goal_pose = create_pose(nav, waypoint[0], waypoint[1], waypoint[2])

        # Send goal
        nav.goToPose(goal_pose)

        # Wait until the goal is reached
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback and feedback.navigation_time.sec > 600:
                nav.cancelTask()
        
        # Take the result
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f'Goal succeeded! reached waypoint {waypoint}')
        elif result == TaskResult.CANCELED:
            nav.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            nav.get_logger().info('Goal failed!')

    # End the node lifecycle
    nav.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()