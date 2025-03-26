import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import csv
import cv2
import numpy as np
import math

class Environment(Node):
    def __init__(self):
        super().__init__('environment')

        self.cones = self.read_cones('conos.csv')
        self.car_position = None
        self.car_yaw = 0.0

        self.create_subscription(Point, '/env/car_position', self.update_car_position, 10)
        self.create_subscription(Float32, '/env/car_yaw', self.update_car_yaw, 10)

        self.timer = self.create_timer(0.1, self.update_image)

        self.get_logger().info("Environment node running successfully")

    def read_cones(self, file_name):
        pkg_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(pkg_dir, file_name)        

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
        if not self.cones:
            self.get_logger().warn("WARNING: No cones received")
            return

        if self.car_position is None:
            self.get_logger().warn("WARNING: No car position received")
            return

        img = np.ones((800, 800, 3), dtype=np.uint8) * 255
        scale = 80

        # Camera follows the car: keep car at center of image (400, 400)
        offset = (-self.car_position.x * scale + 400, self.car_position.y * scale + 400)

        def to_pixel_coords(x, y):
            px = int(x * scale + offset[0])
            py = int(-y * scale + offset[1])
            return px, py

        for color, x, y in self.cones:
            px, py = to_pixel_coords(x, y)
            if color == 'BLUE':
                cv2.circle(img, (px, py), 5, (255, 0, 0), -1)
            elif color == 'YELLOW':
                cv2.circle(img, (px, py), 5, (0, 255, 255), -1)

        px, py = to_pixel_coords(self.car_position.x, self.car_position.y)
        cv2.circle(img, (px, py), 8, (0, 0, 255), -1)
        cv2.putText(img, "Car", (px + 10, py), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        dx = int(20 * math.cos(self.car_yaw))
        dy = int(-20 * math.sin(self.car_yaw))
        cv2.arrowedLine(img, (px, py), (px + dx, py + dy), (0, 0, 0), 2, tipLength=0.3)

        cv2.imshow("Mapa de conos", img)
        cv2.waitKey(1)

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
