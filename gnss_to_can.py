import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # Import for GNSS data
import can


class GNSSToCANBridge(Node):
    def __init__(self):
        super().__init__('gnss_to_can_bridge')

        # Initialize the CAN bus (vcan0)
        self.can_bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

        # Subscribe to the GNSS topic
        self.gnss_sub = self.create_subscription(
            NavSatFix,
            '/sensor/gnss',
            self.gnss_callback,
            10  # Default QoS
        )

    def gnss_callback(self, msg):
        """
        This callback processes the GNSS message and sends it as a single CAN message.
        """
        self.get_logger().info(f"Received GNSS Message: Lat={msg.latitude}, Lon={msg.longitude}, Alt={msg.altitude}")

        # Scale latitude, longitude, and altitude to fit in 1 byte (0-255 range)
        lat_byte = int((msg.latitude - (-0.001)) * 255 / (0.001 - (-0.001)))
        lon_byte = int((msg.longitude - (-0.001)) * 255 / (0.001 - (-0.001)))
        alt_byte = int(msg.altitude * 255 / 5.0)

        # GNSS status (1 byte) and service (1 byte)
        status_byte = msg.status.status & 0xFF
        service_byte = msg.status.service & 0xFF

        # Ensure all values are within the 0-255 range
        lat_byte = max(0, min(255, lat_byte))
        lon_byte = max(0, min(255, lon_byte))
        alt_byte = max(0, min(255, alt_byte))

        # Pack all data into the CAN message
        can_msg = can.Message(
            arbitration_id=0x400,  # Example CAN ID for GNSS data
            data=[lat_byte, lon_byte, alt_byte, status_byte, service_byte, 0, 0, 0],  # Padding for 8 bytes
            is_extended_id=False
        )

        # Send the CAN message
        self.send_can_message(can_msg)

    def send_can_message(self, can_msg):
        """
        Send a CAN message and log the result.
        """
        try:
            self.can_bus.send(can_msg)
            self.get_logger().info(f"Sent CAN Message: {can_msg}")
        except can.CanError as e:
            self.get_logger().error(f"Failed to send CAN message: {e}")


def main(args=None):
    rclpy.init(args=args)
    gnss_to_can_node = GNSSToCANBridge()

    try:
        rclpy.spin(gnss_to_can_node)
    finally:
        gnss_to_can_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
