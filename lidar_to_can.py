import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import can
import array  # For array operations


class LiDARToCANBridge(Node):
    def __init__(self):
        super().__init__('lidar_to_can_bridge')

        # Initialize the CAN bus (vcan0)
        self.can_bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

        # Define QoS profile for LiDAR with VOLATILE durability
        lidar_qos = QoSProfile(depth=10)
        lidar_qos.durability = DurabilityPolicy.VOLATILE
        lidar_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribe to the LiDAR topic
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/concatenated/pointcloud',
            self.lidar_callback,
            qos_profile=lidar_qos
        )

    def lidar_callback(self, msg):
        """
        Process and send LiDAR messages over CAN.
        """
        raw_data = msg.data  # Extract the raw data field
        for i in range(0, len(raw_data), 8):
            chunk = raw_data[i:i + 8]
            padded_chunk = list(chunk) + [0] * (8 - len(chunk))
            padded_chunk_array = array.array('B', padded_chunk)
            can_msg = can.Message(
                arbitration_id=0x300,  # Example CAN ID for LiDAR data
                data=padded_chunk_array[:8],
                is_extended_id=False
            )
            self.send_can_message(can_msg)

    def send_can_message(self, can_msg):
        """
        Send a CAN message and log the result.
        """
        try:
            self.can_bus.send(can_msg)
            self.get_logger().info(f"Sent LiDAR CAN Message: {can_msg}")
        except can.CanError as e:
            self.get_logger().error(f"Failed to send CAN message: {e}")


def main(args=None):
    rclpy.init(args=args)
    lidar_node = LiDARToCANBridge()

    try:
        rclpy.spin(lidar_node)
    finally:
        lidar_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
