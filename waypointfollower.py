import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # Facing forward

        goal_msg.pose = pose

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result:
            self.get_logger().info('Waypoint Reached!')
        else:
            self.get_logger().info('Failed to reach the waypoint.')

def main():
    rclpy.init()
    follower = WaypointFollower()
    
    # Example Waypoints in map coordinates (after converting from GPS)
    waypoints = [
        (1.0, 2.0, 0.0),
        (2.0, 3.5, 0.0),
        (-1.0, -1.0, 0.0)
    ]

    for wp in waypoints:
        follower.send_goal(*wp)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
