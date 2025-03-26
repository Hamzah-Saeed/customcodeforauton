import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose
from geometry_msgs.msg import Twist
import threading

class GpsWaypointCommander(Node):
    def __init__(self):
        super().__init__('gps_wp_commander')
        self.navigator = BasicNavigator("basic_navigator")
        self.get_logger().info("GPS Waypoint Commander initialized")
        
        # Start the input thread for manual waypoints
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='robot_localization'):
        """Block until the full navigation system is up and running."""
        
        # Wait for the localization service to be available
        if localizer != 'robot_localization':  # non-lifecycle node
            self._waitForServiceToActivate(localizer)
        
        # If using AMCL, wait for the initial pose
        if localizer == 'amcl':
            self._waitForInitialPose()
        
        # Wait for the navigation action server to be available
        self._waitForServiceToActivate(navigator)
        
        self.get_logger().info('Nav2 is ready for use!')
        return

    def _waitForServiceToActivate(self, node_name):
        """Wait for a service to activate."""
        while not self.service_is_ready(node_name):
            self.get_logger().info(f'Waiting for {node_name} to activate...')
            rclpy.sleep(1.0)  # Sleep 1 second before retrying

    def service_is_ready(self, service_name):
        """Check if the service is available."""
        try:
            # If service is available, return True
            return self.count_non_diagnostic_services(service_name) > 0
        except Exception as e:
            return False

    def _waitForInitialPose(self):
        """Wait until the robot's initial pose is received."""
        while not self.is_initial_pose_received():
            self.get_logger().info('Waiting for initial pose...')
            rclpy.sleep(1.0)  # Sleep 1 second before retrying

    def send_waypoint(self, lat: float, lon: float):
        """
        Sends a GPS waypoint to Nav2.
        """
        try:
            # Ensure Nav2 and localization are active
            self.waitUntilNav2Active(localizer='robot_localization')

            wp = [latLonYaw2Geopose(lat, lon)]
            self.get_logger().info(f'Sending waypoint: Lat {lat}, Lon {lon}')
            self.navigator.followGpsWaypoints(wp)

            # Wait for completion and log result
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.navigator.isTaskComplete():
                self.get_logger().info("Waypoint reached successfully")
            else:
                self.get_logger().warning("Navigation failed or was interrupted")
                
        except Exception as e:
            self.get_logger().error(f'Error sending waypoint: {str(e)}')

    def input_loop(self):
        """
        Continuously accepts user input for new waypoints.
        """
        while rclpy.ok():
            try:
                print("\nEnter GPS coordinates (or 'q' to quit):")
                print("Latitude: ", end='')
                lat_input = input()

                if lat_input.lower() == 'q':
                    rclpy.shutdown()
                    break
                
                print("Longitude: ", end='')
                lon_input = input()

                if lon_input.lower() == 'q':
                    rclpy.shutdown()
                    break

                lat = float(lat_input)
                lon = float(lon_input)

                # Validate coordinates
                if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                    print("Invalid coordinates! Latitude must be between -90 and 90, Longitude between -180 and 180")
                    continue

                self.send_waypoint(lat, lon)

            except ValueError:
                print("Invalid input! Please enter numerical values.")
            except Exception as e:
                print(f"Error: {str(e)}")

def main():
    rclpy.init()
    gps_wpf = GpsWaypointCommander()

    try:
        rclpy.spin(gps_wpf)
    except KeyboardInterrupt:
        pass
    finally:
        gps_wpf.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
