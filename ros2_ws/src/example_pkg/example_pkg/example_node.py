import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import json

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Float32, 'heading', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Ready")

    def timer_callback(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                data = json.loads(line)
                self.get_logger().info(f'Data: {json.dumps(data)}')
                if 'heading' in data:
                    msg = Float32()
                    msg.data = float(data['heading'])
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error reading from serial port: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
