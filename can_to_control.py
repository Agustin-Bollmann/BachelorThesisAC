import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import can
from autoware_auto_control_msgs.msg import AckermannControlCommand  # For control command

class CANToROSBridge(Node):
    def __init__(self):
        super().__init__('can_to_ros_bridge')

        # Define a QoS profile with TRANSIENT_LOCAL durability
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Publisher for translating CAN messages back to ROS
        self.control_pub = self.create_publisher(
            AckermannControlCommand, '/control/command/control_cmd', qos)

        # Initialize the CAN bus (vcan0)
        self.can_bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

        # Start listening for incoming CAN messages
        self.listen_to_can()

    def listen_to_can(self):
        """
        Continuously listen to CAN messages and translate them into ROS messages.
        """
        self.get_logger().info("Listening to CAN messages on vcan0...")
        try:
            while True:
                can_msg = self.can_bus.recv(timeout=1.0)  # Wait for a CAN message
                if can_msg:
                    self.translate_can_to_ros(can_msg)
        except KeyboardInterrupt:
            self.get_logger().info("Stopped listening to CAN messages.")
        except can.CanError as e:
            self.get_logger().error(f"Error while receiving CAN message: {e}")

    def translate_can_to_ros(self, can_msg):
        """
        Translate a received CAN message into the appropriate ROS message and publish it.
        """
        if can_msg.arbitration_id == 0x110:  # Unified CAN message for Brake, Steering, and Throttle
            ros_msg = AckermannControlCommand()

            # Decode brake, steering, and throttle from the CAN message
            raw_brake = can_msg.data[0]
            raw_steering = can_msg.data[1]
            raw_throttle = can_msg.data[2]

            # Translate the values to their respective ranges
            translated_brake = raw_brake * (5 - (-5)) / 255 + (-5)
            translated_steering = raw_steering * (0.75 - (-0.75)) / 255 + (-0.75)
            translated_throttle = raw_throttle * (15 - 0) / 255

            # Populate the ROS message
            ros_msg.longitudinal.acceleration = round(translated_brake, 2)
            ros_msg.lateral.steering_tire_angle = round(translated_steering, 5)
            ros_msg.longitudinal.speed = round(translated_throttle, 2)

            # Publish the ROS message
            self.control_pub.publish(ros_msg)

            # Log the translated values
            self.get_logger().info(f"Translated CAN Message: Brake={ros_msg.longitudinal.acceleration}, "
                                   f"Steering={ros_msg.lateral.steering_tire_angle}, "
                                   f"Throttle={ros_msg.longitudinal.speed}")


def main(args=None):
    rclpy.init(args=args)
    can_to_ros = CANToROSBridge()

    try:
        rclpy.spin(can_to_ros)
    finally:
        can_to_ros.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
