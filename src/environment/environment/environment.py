import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import csv
import cv2
import numpy as np
import threading

class ConeVisualizer(Node):
    def __init__(self):
        super().__init__('cone_visualizer')

        self.cones = self.read_cones('conos.csv')
        self.car_position = Point()

        self.create_subscription(Point, 'car_position', self.position_callback, 10)

        self.timer = self.create_timer(0.1, self.update_image)

    def read_cones(self, file_path):
        cones = []
        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) != 3:
                    continue
                color, x, y = row
                cones.append((color.strip().upper(), float(x), float(y)))
        return cones

    def position_callback(self, msg):
        self.car_position = msg

    def update_image(self):
        img = np.ones((500, 500, 3), dtype=np.uint8) * 255

        scale = 50
        offset = (250, 250)

        def to_pixel_coords(x, y):
            px = int(x * scale + offset[0])
            py = int(-y * scale + offset[1])
            return px, py

        for color, x, y in self.cones:
            px, py = to_pixel_coords(x, y)
            if color == 'AZUL':
                cv2.circle(img, (px, py), 5, (255, 0, 0), -1)
            elif color == 'AMARILLO':
                cv2.circle(img, (px, py), 5, (0, 255, 255), -1)

        px, py = to_pixel_coords(self.car_position.x, self.car_position.y)
        cv2.circle(img, (px, py), 8, (0, 0, 255), -1)
        cv2.putText(img, "Car", (px + 10, py), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow("Mapa de conos", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ConeVisualizer()
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
