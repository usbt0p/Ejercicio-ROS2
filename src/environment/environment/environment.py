import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory
import csv
import cv2
import numpy as np
import math

from auria_msgs.msg import PointArray

class Environment(Node):
    def __init__(self):
        super().__init__('environment')

        self.cones = self.read_cones('small_track.csv')
        self.car_position = None
        self.car_yaw = 0.0

        self.create_subscription(Point, '/env/car_position', self.update_car_position, 10)
        self.create_subscription(Float32, '/env/car_yaw', self.update_car_yaw, 10)

        self.timer = self.create_timer(0.1, self.update_image)

        self.speed_pub = self.create_publisher(Float32, '/can/speed', 10)
        self.yaw_pub = self.create_publisher(Float32, '/can/yaw', 10)

        self.pub_all_blue = self.create_publisher(PointArray, '/vision/all_cones/blue', 10)
        self.pub_all_yellow = self.create_publisher(PointArray, '/vision/all_cones/yellow', 10)
        self.pub_near_blue = self.create_publisher(PointArray, '/vision/nearby_cones/blue', 10)
        self.pub_near_yellow = self.create_publisher(PointArray, '/vision/nearby_cones/yellow', 10)

        self.get_logger().info("Environment node running successfully")

    def read_cones(self, file_name):
        pkg_share_dir = get_package_share_directory('environment')
        file_path = os.path.join(pkg_share_dir, 'tracks', file_name)

        self.get_logger().info(f'target path: {file_path}')
        cones = []
        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                try:
                    if len(row) != 3:
                        continue
                    color, x, y = row
                    cones.append((color.strip().upper(), float(x), float(y)))
                except ValueError:
                    pass
        return cones

    def update_car_position(self, msg):
        self.car_position = msg

    def update_car_yaw(self, msg):
        self.car_yaw = msg.data

    def update_image(self):
        if not self.cones or self.car_position is None:
            self.get_logger().warn("WARNING: Missing cones or car position")
            return

        img = np.ones((800, 800, 3), dtype=np.uint8) * 255
        scale = 80
        offset = (-self.car_position.x * scale + 400, self.car_position.y * scale + 400)

        def to_pixel_coords(x, y):
            px = int(x * scale + offset[0])
            py = int(-y * scale + offset[1])
            return px, py

        blue_all = []
        yellow_all = []
        blue_near = []
        yellow_near = []

        for color, x, y in self.cones:
            px, py = to_pixel_coords(x, y)
            if color == 'BLUE':
                cv2.circle(img, (px, py), 5, (255, 0, 0), -1)
            elif color == 'YELLOW':
                cv2.circle(img, (px, py), 5, (0, 255, 255), -1)

            point = Point(x=x, y=y, z=0.0)
            if color == 'BLUE':
                blue_all.append(point)
            elif color == 'YELLOW':
                yellow_all.append(point)

            distance = math.hypot(x - self.car_position.x, y - self.car_position.y)
            if distance < 8.0:
                if color == 'BLUE':
                    blue_near.append(point)
                elif color == 'YELLOW':
                    yellow_near.append(point)

        # Draw car and direction
        px, py = to_pixel_coords(self.car_position.x, self.car_position.y)
        cv2.circle(img, (px, py), 8, (0, 0, 255), -1)
        cv2.putText(img, "Car", (px + 10, py), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        dx = int(20 * math.cos(self.car_yaw))
        dy = int(-20 * math.sin(self.car_yaw))
        cv2.arrowedLine(img, (px, py), (px + dx, py + dy), (0, 0, 0), 2, tipLength=0.3)

        cv2.imshow("Mapa de conos", img)
        cv2.waitKey(1)

        # Speed
        speed = Float32()
        speed.data = math.hypot(self.car_position.x, self.car_position.y)
        self.speed_pub.publish(speed)

        # Yaw
        yaw = Float32()
        yaw.data = self.car_yaw
        self.yaw_pub.publish(yaw)

        # All cones
        msg_all_blue = PointArray()
        msg_all_blue.points = blue_all
        self.pub_all_blue.publish(msg_all_blue)

        msg_all_yellow = PointArray()
        msg_all_yellow.points = yellow_all
        self.pub_all_yellow.publish(msg_all_yellow)

        # Nearby cones
        msg_near_blue = PointArray()
        msg_near_blue.points = blue_near
        self.pub_near_blue.publish(msg_near_blue)

        msg_near_yellow = PointArray()
        msg_near_yellow.points = yellow_near
        self.pub_near_yellow.publish(msg_near_yellow)

def main(args=None):
    rclpy.init(args=args)
    node = Environment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
