import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image  # Import for Image data
import can
import array  # For array operations


class CameraToCANBridge(Node):
    def __init__(self):
        super().__init__('camera_to_can_bridge')

        # Initialize the CAN bus (vcan0)
        self.can_bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

        # Define QoS profile for camera topic
        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Adjusted to BEST_EFFORT
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribe to the camera topic
        self.camera_sub = self.create_subscription(
            Image,
            '/sensing/camera/traffic_light/image_raw',  # Topic for camera data
            self.camera_callback,
            qos_profile=camera_qos
        )

    def camera_callback(self, msg):
        """
        Process and send camera messages over CAN.
        """
        raw_data = msg.data  # Extract the raw data field
        for i in range(0, len(raw_data), 8):
            # Create chunks of 8 bytes (CAN frame size)
            chunk = raw_data[i:i + 8]
            # Pad the chunk to ensure it has 8 bytes
            padded_chunk = list(chunk) + [0] * (8 - len(chunk))
            padded_chunk_array = array.array('B', padded_chunk)
            # Create CAN message
            can_msg = can.Message(
                arbitration_id=0x500,  # Example CAN ID for camera data
                data=padded_chunk_array[:8],
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
            self.get_logger().info(f"Sent Camera CAN Message: {can_msg}")
        except can.CanError as e:
            self.get_logger().error(f"Failed to send CAN message: {e}")


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraToCANBridge()

    try:
        rclpy.spin(camera_node)
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
