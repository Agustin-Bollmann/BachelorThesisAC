import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
import can


class ControlToCANBridge(Node):
    def __init__(self):
        super().__init__('control_to_can_bridge')

        # Initialize the CAN bus (vcan0)
        self.can_bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

        # Subscribe to the control command topic
        self.control_sub = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.control_callback,
            10  # Default QoS
        )

    def control_callback(self, msg):
        """
        Process and send control messages over CAN.
        """
        raw_brake = msg.longitudinal.acceleration
        raw_steering = msg.lateral.steering_tire_angle
        raw_throttle = msg.longitudinal.speed

        # Translate the values to the corresponding byte ranges
        brake_byte = int((raw_brake - (-5)) * 255 / (5 - (-5)))
        steering_byte = int((raw_steering - (-0.75)) * 255 / (1 - (-0.75)))
        throttle_byte = int(raw_throttle * 255 / 15)

        # Create a CAN message with all three values
        can_msg = can.Message(
            arbitration_id=0x110,  # Example CAN ID for control data
            data=[brake_byte, steering_byte, throttle_byte],
            is_extended_id=False
        )

        self.send_can_message(can_msg)

    def send_can_message(self, can_msg):
        """
        Send a CAN message and log the result.
        """
        try:
            self.can_bus.send(can_msg)
            self.get_logger().info(f"Sent Control CAN Message: {can_msg}")
        except can.CanError as e:
            self.get_logger().error(f"Failed to send CAN message: {e}")


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlToCANBridge()

    try:
        rclpy.spin(control_node)
    finally:
        control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
