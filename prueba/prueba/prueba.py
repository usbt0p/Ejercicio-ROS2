import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String

class Prueba(Node):

    HZ = 10 # 10 Hz publishing rate

    def __init__(self):
        super().__init__('prueba')
        #self.get_logger().info("Prueba node running successfully")

        # tipo de mensaje, 
        self.acc_pub = self.create_publisher(Float32, '/car/acceleration', self.HZ)

        self.timer = self.create_timer(1.0 / self.HZ, self.send_acceleration)

    # now send acdeleration to /car/acceleration
    def send_acceleration(self):
        msg = Float32()
        msg.data = 0.1
        self.acc_pub.publish(msg)

    def send_steering(self):
        msg = Float32()
        msg.data = 0.1
        self.steering_pub.publish(msg)
    
    def send_state(self):
        msg = String()
        msg.data = "running"
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    prueba = Prueba()

    try:
        rclpy.spin(prueba) # keep the node running until it is shutdown
        
    except KeyboardInterrupt:
        pass
    finally: 
        prueba.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    