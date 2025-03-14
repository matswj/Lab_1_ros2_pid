import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # PID-parametere
        self.p = 1.0
        self.i = 0.0
        self.d = 0.0
        self.reference = 0.0
        self.voltage = 0.0
        self.previous_error = 0.0
        self.integral = 0.0

        # ROS2 Publisher og Subscriber
        self.publisher_ = self.create_publisher(Float64, 'voltage', 10)
        self.subscription = self.create_subscription(
            Float64,
            'measured_angle',
            self.measurement_listener,
            10)
        
        self.get_logger().info("PID Controller Node started!")

    def measurement_listener(self, msg):
        """Callback-funksjon som mottar målt vinkel og beregner PID-korreksjon"""
        measured_angle = msg.data
        error = self.reference - measured_angle

        # PID-algoritme
        self.integral += error
        derivative = error - self.previous_error
        self.voltage = self.p * error + self.i * self.integral + self.d * derivative
        self.previous_error = error

        # Publiserer spenning til aktuator
        voltage_msg = Float64()
        voltage_msg.data = self.voltage
        self.publisher_.publish(voltage_msg)
        self.get_logger().info(f'Published voltage: {self.voltage:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
