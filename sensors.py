import rclpy
import math
from rclpy.node import Node
from rclpy.timer import Timer

from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Int64

import utils.pyum982 as pyum982
from vnpy import *
import time, os


class SensorPub(Node):
    def __init__(self):
        super().__init__('sensors_node')
        
        sudoPassword = 'xav'
        command = 'chmod 666 /dev/ttyUSB0'
        devpath = '/dev/ttyUSB0'
        self.parser = None

        while 1:
            """
            Add port check function
            """
            try:
                
                os.system('echo %s|sudo -S %s' % (sudoPassword, command))
                break
            except:
                self.get_logger().error("Error getting port access permissions from Linux")
                time.sleep(1)

        while 1:
            try:
                self.parser = pyum982.UM982()
                self.parser.connect()
                break
            except:
                self.get_logger().error("failed to connect to H-RTK, trying again...")
                time.sleep(1)

        self.publisher = self.create_publisher(
            Point,
            'rover_pose_msg',
            10)
        
        self.gps_publisher = self.create_publisher(
            NavSatFix,
            'gps/fix',
            10 #queue size 
        )

        self.heading_publisher = self.create_publisher(
            Imu,
            '/imu/data',
            10 #queue size 
        )

        timer_period  = 0.010
        self.create_timer(timer_period, self.timer_callback)
        
        self.node_sub = self.create_subscription(
            Int64,
            'node_test',
            self.node_callback,
            10
        )

        self.zed2inode_sub = self.create_subscription(
            Imu,
            '/zed2i/zed_node/imu/data',
            self.imu_callback,
            10
        )

        
        
    def imu_callback(self, msg: Imu):
        self.heading_publisher.publish(msg)


        
    def timer_callback(self):
        
        """READ SENSOR DATA"""
        data = self.parser.read()
        if (data != None): 

            lat = data.bestpos_lat
            lon = data.bestpos_lon
            alt = self.attitude()
            # yaw = data.heading_degree <---- This is what was used in the old code, im using YPR instead

            yaw, pitch, roll = self.ypr()
            yaw_rad = math.radians(yaw)
            pitch_rad = math.radians(pitch)
            roll_rad = math.radians(roll)


            #create NavSatFix Message#
            nsm = NavSatFix()
            nsm.header.stamp = self.get_clock().now().to_msg()
            nsm.header.frame_id = self.base_frame 
            nsm.latitude = float(lat)
            nsm.longitude = float(lon)
            nsm.altitude = float(alt)
            nsm.status.status = 0 #this is assuming that the gps has gotten a fix
            nsm.status.service = 1 # gps service is being used

            self.gps_publisher.publish(nsm)


            #print(lat, lon, yaw, type(lat), type(lon), type(yaw))

            """PUBLISH SENSOR DATA"""
            msg = Point()
            msg.x = float(lat)
            msg.y = float(lon)
            msg.z = float(yaw)
        
            self.publisher.publish(msg)
            print(msg)
        else:
            self.get_logger().error("Failed to read GPS data")

    
    def ypr(self):
        return self.vn.read_yaw_pitch_roll()
    
    def ins_pos(self):
        return self.vn.read_ins_solution_lla().pos
    
    def gps_lla(self):
        return self.vn.read_gps_solution_lla().lla
    
    def attitude(self):
        return self.vn.any_attitude()
    
    def gps_compass(self):
        return self.vn.read_gps_compass_estimated_baseline()
    
    def node_callback(self, msg):
        None


def main(args=None):
    rclpy.init(args=args)

    sensor_pub = SensorPub()

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(sensor_pub, executor=executor)
    except:
        rclpy.shutdown()

if __name__ == '__main__':
    
    main()
