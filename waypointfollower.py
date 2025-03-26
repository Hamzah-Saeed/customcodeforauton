import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowGPSWaypoints
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

class GPSWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')
        self.client = ActionClient(self, FollowGPSWaypoints, 'follow_gps_waypoints')

    def send_goal(self, gps_waypoints):
        goal_msg = FollowGPSWaypoints.Goal()
        goal_msg.gps_waypoints = gps_waypoints

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result:
            self.get_logger().info(f"Waypoints followed successfully!")
        else:
            self.get_logger().info(f"Failed to follow waypoints.")

def main():
    rclpy.init()
    follower = GPSWaypointFollower()

    # Example GPS waypoints (latitude, longitude, and heading)
    gps_waypoints = [
        [37.7749, -122.4194, 0.0],  # Example 1: San Francisco (latitude, longitude, heading)
        [37.7750, -122.4183, 0.0]   # Example 2: Another location nearby
    ]

    # Send the waypoints to the action server
    follower.send_goal(gps_waypoints)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
